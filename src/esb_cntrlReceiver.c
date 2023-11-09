#include <stdint.h>
#include <string.h>
#include <app_error.h>
#include <pstorage_platform.h>
#include <nrf.h>
#include <nrf_esb.h>
#include <nrf_soc.h>

#include "bt_nRF51822Rev3_lib.h"
#include "timer.h"
#include "watchdog.h"
#include "esb.h"
#include "ble_events.h"
#include "dp2.h"
#include "misc.h"

#ifndef ESB_PRESENT
#error ESB_PRESENT needs to be defined!
// ESB_PRESENT is used both by our library and also by the Nordic libraries.
#endif

#define TIMESLOT_BEGIN_IRQn         LPCOMP_IRQn
#define TIMESLOT_BEGIN_IRQHandler   LPCOMP_IRQHandler
#define TIMESLOT_BEGIN_IRQPriority  1
#define TIMESLOT_END_IRQn           QDEC_IRQn
#define TIMESLOT_END_IRQHandler     QDEC_IRQHandler
#define TIMESLOT_END_IRQPriority    1

/*****************************************************************************************************************
 * Timing
 *****************************************************************************************************************/

#ifndef ESB_STANDBY_DELAY_MS
#define ESB_STANDBY_DELAY_MS 200
#endif

#define TS_LEN_US                 (5000UL)                           /**< The timeslot lentgh should not be much smaller than 4ms to make sure that wake-up packets can be received in stand-by-mode. */
#define TX_LEN_EXTENSION_US       (TS_LEN_US)
#define TS_SAFETY_MARGIN_US       (1650UL)                           /**< The timeslot activity should be finished with this much to spare. */
#define TS_EXTEND_MARGIN_US       (2600UL)                           /**< The timeslot activity should request an extension this long before end of timeslot. */
#define TS_REQ_AND_END_MARGIN_US  (350UL)                            /**< Used to send "request and end" before closing the timeslot. */
#define TS_STANDBY_DELAY_US       (1000ul * ESB_STANDBY_DELAY_MS)    /**< Delay between 2 Timeslots in stand-by. */

#if TS_STANDBY_DELAY_US > NRF_RADIO_DISTANCE_MAX_US
#error The stand-by delay is too large!
#endif
#if (TS_EXTEND_MARGIN_US >= TS_LEN_US) || (TS_SAFETY_MARGIN_US >= TS_EXTEND_MARGIN_US) || (TS_REQ_AND_END_MARGIN_US >= TS_SAFETY_MARGIN_US)
#error The following condition must be true: TS_LEN_US > TS_EXTEND_MARGIN_US > TS_SAFETY_MARGIN_US > TS_REQ_AND_END_MARGIN_US 
#endif

#define ESB_BUTTON_TIMEOUT          150  ///< teach-button timeout - default: 1,5 seconds
#define ESB_DELETE_BUTTON_TIMEOUT   500  ///< teach-button timeout for deleting ESB address - default: 5 seconds
#define ESB_TEACHMODE_TIMEOUT       6000 ///< teach-mode timeout - default: 60 seconds
#define ESB_TEACHMODE_EXTRA_TIMEOUT 200  ///< Extra timeout if teach-mode is almost finished. default: 2 seconds

#define ESB_RADIO_ACTIVE_TIMEOUT (10000 / 10) ///< Time before entering stand-by mode, 10ms intervall, default: 10 seconds

/*****************************************************************************************************************
 * Default Addresses
 *****************************************************************************************************************/
 
#define teachAddr         0x96699669
#define teachAddrPrefix   (0x90 + 4) // default is to use address of pipe 4
#define defaultAddr       0x817e817e
#define defaultAddrPrefix 0x80
#define defaultChannel    4

#ifndef defaultPipe
#define defaultPipe 4
#endif


#define RF_EMPTY_FRAME  0x00 ///<	This 'command' is used for the tx_frame to mark it as empty. The frame will no be send if the command-field is set to this value.

#define MAX_RF_CHANNEL   84  // höchster beim Teach-Vorgang zu durchsuchender Funk-Kanal (vergleiche auch MAX_CHANNEL)
#define MIN_RF_CHANNEL    0  // niedrigster beim Teach-Vorgang zu durchsuchender Funk-Kanal (vergleiche auch MIN_CHANNEL)
#define MIN_CHANNEL       3  // niedrigster Kanal der verwendet werden soll. (vergleiche auch MIN_RF_CHANNEL)
#define MAX_CHANNEL      78  // höchster Kanal der verwendet werden soll. (vergleiche auch MAX_RF_CHANNEL)
                             // (Die Kanäle 79 und 80 wurden f?r FCC entfernt.)
#define KEY_CODE_TIMEOUT 30  // ca. 300 ms. - default is 50 -> 500ms

union Frame
{
	__packed struct
	{
		unsigned char length;   // length of datafield - max: 29 = NRF_ESB_CONST_MAX_PAYLOAD_LENGTH - 3
		unsigned char sequence; // reserved for future use...
		unsigned char command;
		unsigned char data[NRF_ESB_CONST_MAX_PAYLOAD_LENGTH - 3];
	} packet;
	__packed unsigned char rawData[NRF_ESB_CONST_MAX_PAYLOAD_LENGTH];
};

/*****************************************************************************************************************
 * global variables
 *****************************************************************************************************************/

uint32_t 						timeslotRunning  = 2;

uint32_t            esb_keyCode      = 0;
static uint32_t     esb_keyCodeTimer = 0;

static int_fast8_t   addrHasChanged   = 0;
member_type_t        esbAddress;
static uint_fast16_t teachModeTimer = 0;

enum rfMode_t rfMode = rfMode_normal;

static uint32_t       m_total_timeslot_length = 0;
static signed int     esbStop = 0;
static uint_fast16_t  esbRadioActiveTimer = ESB_RADIO_ACTIVE_TIMEOUT;

uint_fast8_t m_flash_interval = 0;
uint_fast8_t m_flash_count    = 0;

static union Frame tx_frame;  ///< this union holds the last frame for the acknowledge packet.
static uint32_t    tx_pipe;
static bool        m_forced_teach = false;

#ifndef NO_SOCKET_SUPPORT
#define HS_TIMEOUT          100 // 1 second
#define FEEDBACK_SYNC_TIMER 200 // every 2 second
#define FEEDBACK_TIMEOUT    10  // 100ms
#define SOCKET_PIPE         1
#define SOCKET_KEYCODE      0x00000200

static uint_fast16_t m_hs_timeout            = 0;
static uint_fast16_t m_feedback_timeout      = 0;
static uint_fast16_t m_feedback_sync_timer   = FEEDBACK_SYNC_TIMER;
static uint_fast8_t  m_feedback_error_count  = 0;
extern volatile ble_flags_t m_ble_flags;
#endif

/*****************************************************************************************************************
 * global constants
 *****************************************************************************************************************/

#if TS_STANDBY_DELAY_US != 0
static nrf_radio_request_t	radioRequestOptionsStandBy =
{
	.request_type = NRF_RADIO_REQ_TYPE_NORMAL,
	.params.normal.hfclk = NRF_RADIO_HFCLK_CFG_FORCE_XTAL,
	.params.normal.priority = NRF_RADIO_PRIORITY_NORMAL,
	.params.normal.distance_us = TS_STANDBY_DELAY_US,
	.params.normal.length_us = TS_LEN_US,
};
#endif

static nrf_radio_request_t	radioRequestOptions =
{
	.request_type               = NRF_RADIO_REQ_TYPE_EARLIEST,
	.params.earliest.hfclk      = NRF_RADIO_HFCLK_CFG_FORCE_XTAL,
	.params.earliest.length_us  = TS_LEN_US,
	.params.earliest.priority   = NRF_RADIO_PRIORITY_NORMAL,
	.params.earliest.timeout_us = NRF_RADIO_EARLIEST_TIMEOUT_MAX_US
};

static nrf_radio_signal_callback_return_param_t callbackReturnNoAction =
{
	.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE
};

/**< This will be used at the end of each timeslot to request the next timeslot. */
static nrf_radio_signal_callback_return_param_t callbackReturnScheduleNext =
{
	.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END,
	.params.request  = &radioRequestOptions
};

/**< This will be used at the end of each timeslot to request an extension of the timeslot. */
static nrf_radio_signal_callback_return_param_t callbackReturnExtend = {
	.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND,
	.params.extend   = TX_LEN_EXTENSION_US
};

static nrf_radio_signal_callback_return_param_t callbackReturnStopESB =
{
	.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END
};

/*****************************************************************************************************************
 * macros and prototypes
 *****************************************************************************************************************/

#define timeslotIsActive()	(m_total_timeslot_length != 0)
nrf_radio_signal_callback_return_param_t* radio_signal_callback(uint8_t signal_type);
void RADIO_IRQHandler(void);

/*****************************************************************************************************************
 * functions
 *****************************************************************************************************************/

void esb_teachmode_start(void)
{
	if(rfMode == rfMode_normal)
	{
		rfMode = rfMode_teach_pending;
		teachModeTimer = ESB_TEACHMODE_TIMEOUT;
		ble_rf_teach_mode_event(true);
	}
}

void esb_teachmode_checkTimeout(void)
{
	if(teachModeTimer < ESB_TEACHMODE_EXTRA_TIMEOUT)
		teachModeTimer = ESB_TEACHMODE_EXTRA_TIMEOUT;
}

void esb_teachmode_stop(void)
{
#ifndef NO_SOCKET_SUPPORT
	if((rfMode != rfMode_normal) && (rfMode != rfMode_tx) && !m_ble_flags.central_connected && (m_feedback_error_count == 0)) 
#else
	if(rfMode != rfMode_normal)
#endif
	{
		rfMode = rfMode_normal;
		ble_esb_led_off_event();
		ble_rf_teach_mode_event(false);
	};
	m_forced_teach = false;
}

void esb_init(bool start_tech_mode)
{
	int err_code;
	err_code = rf_member_manager_init();
	APP_ERROR_CHECK(err_code);
	
	err_code = member_info_load_from_flash(&esbAddress);
	APP_ERROR_CHECK(err_code);
	
	esbStop = 0;
	tx_frame.packet.command = RF_EMPTY_FRAME;
	
#if (MIN_RF_CHANNEL > 0)
	if ((esbAddress.channel < MIN_RF_CHANNEL)
	 || (esbAddress.channel > MAX_RF_CHANNEL))
#else
	if (esbAddress.channel > MAX_RF_CHANNEL)
#endif
	{
		memset(&esbAddress, 0xff, sizeof(esbAddress));
		esbAddress.channel    = defaultChannel;
		esbAddress.addr       = defaultAddr;
		esbAddress.addrPrefix = defaultAddrPrefix;
		esbAddress.pipe       = defaultPipe;
	}
	
	// Using avilable interrupt handlers for interrupt level management
	// These can be any available IRQ as we're not using any of the hardware,
	// simply triggering them through software
	sd_nvic_ClearPendingIRQ(TIMESLOT_END_IRQn);
	sd_nvic_SetPriority(TIMESLOT_END_IRQn, TIMESLOT_END_IRQPriority);
	sd_nvic_EnableIRQ(TIMESLOT_END_IRQn);

	sd_nvic_ClearPendingIRQ(TIMESLOT_BEGIN_IRQn);
	sd_nvic_SetPriority(TIMESLOT_BEGIN_IRQn, TIMESLOT_BEGIN_IRQPriority);
	sd_nvic_EnableIRQ(TIMESLOT_BEGIN_IRQn);

	sd_radio_session_open(radio_signal_callback);
	sd_radio_request(&radioRequestOptions);
	
	// Startup in teach-mode
	if(start_tech_mode)
		esb_teachmode_start();
};

void esb_generateRandomAddress(void)
{
	uint32_t randomData;
	uint8_t  randomDataLength;
	
	uint32_t err_code = sd_rand_application_bytes_available_get(&randomDataLength);
	APP_ERROR_CHECK(err_code);
	
	if(randomDataLength > 4)
		randomDataLength = 4;

	if(randomDataLength)
	{
		err_code = sd_rand_application_vector_get((uint8_t*)&randomData, randomDataLength);
		APP_ERROR_CHECK(err_code);
		
		esbAddress.channel = (randomData % (MAX_RF_CHANNEL - MIN_RF_CHANNEL)) + MIN_RF_CHANNEL;
		esbAddress.addr = randomData;
	}
	else
	{
		esbAddress.channel++;
		esbAddress.addr += 0x04030201;
	}
	
	if(esbAddress.channel > MAX_RF_CHANNEL)
		esbAddress.channel = MIN_RF_CHANNEL;
	
	esbAddress.addrPrefix = defaultAddrPrefix;
	esbAddress.pipe       = defaultPipe;
	
	addrHasChanged = -1;
};

void esb_sys_event_hanlder(uint32_t sys_evt)
{
	switch (sys_evt)
	{
		//case NRF_EVT_RADIO_SESSION_IDLE:
			// The session has no remaining scheduled timeslots. If this event is triggered the application ends the session.
		case NRF_EVT_RADIO_BLOCKED:
			// The requested timeslot could not be scheduled due to a collision with other activities.
			// The application should request a new timeslot either at the earliest possible, or at the next normal position.
		case NRF_EVT_RADIO_CANCELED:
			// The scheduled timeslot was cancelled by higher priority activity. The application should request a new timeslot.
			if(esbStop == 0)
			{
				sd_radio_request(&radioRequestOptions);
			}
			break;
			
		case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
			// The last signal handler return value contained invalid parameters. The application should assert.
			APP_ERROR_CHECK(NRF_ERROR_INVALID_PARAM);
			break;
		//case NRF_EVT_RADIO_SESSION_CLOSED:
			// The session is closed and all acquired resources are released.
	}
}

/**@brief   Function for handling timeslot events.
 */
nrf_radio_signal_callback_return_param_t* radio_signal_callback(uint8_t signal_type)
{
	if(esbStop != 0)
	{
		return &callbackReturnStopESB;
	};
	
	// NOTE: This callback runs at lower-stack priority (the highest priority possible).
	switch (signal_type)
	{
		case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
			// TIMER0 is pre-configured for 1Mhz.
			NRF_TIMER0->TASKS_STOP          = 1;
			NRF_TIMER0->TASKS_CLEAR         = 1;
			NRF_TIMER0->MODE                = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);
			NRF_TIMER0->EVENTS_COMPARE[0]   = 0;
			NRF_TIMER0->EVENTS_COMPARE[1]   = 0;
			NRF_TIMER0->EVENTS_COMPARE[2]   = 0;
			NRF_TIMER0->INTENSET            = (TIMER_INTENSET_COMPARE0_Msk | TIMER_INTENSET_COMPARE1_Msk | TIMER_INTENSET_COMPARE2_Msk);
			NRF_TIMER0->CC[0]               = (TS_LEN_US - TS_SAFETY_MARGIN_US);
			NRF_TIMER0->CC[1]               = (TS_LEN_US - TS_EXTEND_MARGIN_US);
			NRF_TIMER0->CC[2]               = (TS_LEN_US - TS_REQ_AND_END_MARGIN_US);
			NRF_TIMER0->BITMODE             = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos);
			NRF_TIMER0->TASKS_START         = 1;

			NRF_RADIO->POWER                = (RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos);

			m_total_timeslot_length = TS_LEN_US;

			//sd_nvic_EnableIRQ(TIMER0_IRQn);
			NVIC_EnableIRQ(TIMER0_IRQn);

			// ESB packet receiption and transmission are synchronized at the beginning of timeslot extensions. 
			// Ideally we would also transmit at the beginning of the initial timeslot, not only extensions,
			// but this is to simplify a bit. 
			//sd_nvic_SetPendingIRQ(TIMESLOT_BEGIN_IRQn);
			NVIC_SetPendingIRQ(TIMESLOT_BEGIN_IRQn);
			break;

		case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
			if(NRF_TIMER0->EVENTS_COMPARE[0] &&
			  (NRF_TIMER0->INTENSET & (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENCLR_COMPARE0_Pos)))
			{
				NRF_TIMER0->EVENTS_COMPARE[0] = 0;
			
				// This is the "timeslot is about to end" timeout

				// Disabling ESB is done in a lower interrupt priority 
				NVIC_SetPendingIRQ(TIMESLOT_END_IRQn);
		 
				// Return with no action request. request_and_end is sent later
				return &callbackReturnNoAction;
			}

			if(NRF_TIMER0->EVENTS_COMPARE[1] &&
			  (NRF_TIMER0->INTENSET & (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENCLR_COMPARE1_Pos)))
			{
				NRF_TIMER0->EVENTS_COMPARE[1] = 0;
			
				// This is the "try to extend timeslot" timeout
			
				if((m_total_timeslot_length < (128000000UL - 1UL - TX_LEN_EXTENSION_US))
#if TS_STANDBY_DELAY_US != 0
				    && ((rfMode != rfMode_normal) || (esbRadioActiveTimer != 0)) // if there is no communication enter stand-by mode -> thus do not try to extend this timeslot...
#endif
				    && (rfMode != rfMode_teach_pending)) // If rfMode is rfMode_teach_pending do not extend but restart into teach mode...
				{
					// Request timeslot extension if total length does not exceed 128 seconds
					return &callbackReturnExtend;
				}
				else
				{
					// Return with no action request
					return &callbackReturnNoAction;
				}
			}
		
			if(NRF_TIMER0->EVENTS_COMPARE[2] &&
			  (NRF_TIMER0->INTENSET & (TIMER_INTENSET_COMPARE2_Enabled << TIMER_INTENCLR_COMPARE2_Pos)))
			{
				NRF_TIMER0->EVENTS_COMPARE[2] = 0;
#if TS_STANDBY_DELAY_US != 0
				if(esbRadioActiveTimer == 0)
				{
					// if there is no ongoing communication turn off the receiver to go into sleep mode... - request the next timeslot after a short delay.
					callbackReturnScheduleNext.params.request.p_next = &radioRequestOptionsStandBy;
				} 
				else
#endif
				{
					// if there is ongoing communication request the next timeslot as soon as possible...
					callbackReturnScheduleNext.params.request.p_next = &radioRequestOptions;
			
					if(rfMode != rfMode_normal)
					{
						radioRequestOptions.params.earliest.priority = NRF_RADIO_PRIORITY_HIGH;
					}
					else
					{
						radioRequestOptions.params.earliest.priority = NRF_RADIO_PRIORITY_NORMAL;
					};
				};
				// Schedule next timeslot
				return &callbackReturnScheduleNext;
			}
			break;
		
		case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
			// Call the esb IRQHandler
			RADIO_IRQHandler();
			break;

		case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
			// Don't do anything. Our timer will expire before timeslot ends
			break;

		case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
			// Extension succeeded: update timer
			NRF_TIMER0->TASKS_STOP         = 1;
			NRF_TIMER0->EVENTS_COMPARE[0]  = 0;
			NRF_TIMER0->EVENTS_COMPARE[1]  = 0;
			NRF_TIMER0->CC[0]             += (TX_LEN_EXTENSION_US - 25);
			NRF_TIMER0->CC[1]             += (TX_LEN_EXTENSION_US - 25);
			NRF_TIMER0->CC[2]             += (TX_LEN_EXTENSION_US - 25);
			NRF_TIMER0->TASKS_START        = 1;

			// Keep track of total length
			m_total_timeslot_length += TX_LEN_EXTENSION_US;

#ifdef WDT_ON
			watchdog_reset_timeslot();
#endif
			break;
	};

	// Fall-through return: return with no action request
	return &callbackReturnNoAction;
}

/**@brief IRQHandler used for execution context management. 
  *        Any available handler can be used as we're not using the associated hardware.
  *        This handler is used to stop and disable ESB
  */
void TIMESLOT_END_IRQHandler(void)
{
	timeslotRunning=0;
	m_total_timeslot_length = 0;
	nrf_esb_disable();
}

void esb_stop(void)
{
	esbStop = -1;
	rfMode = rfMode_normal;
	m_total_timeslot_length = 0;
	nrf_esb_disable();
};

#define TEACHMODE_MAX_TXCOUNT 0
uint_fast8_t teachModeTxCount;

void esb_send_moduleID(void)
{
	union Frame tx_frame;

	tx_frame.packet.length = 10;
	tx_frame.packet.command = RF_MODULID;
	tx_frame.packet.data[0] = 0; // Modul-ID
	tx_frame.packet.data[1] = 0; // Version-ID
	tx_frame.packet.data[2] = 0; // Revision-ID
	tx_frame.packet.data[3] = 0; // Controller-ID
	tx_frame.packet.data[4] = esbAddress.pipe;
	tx_frame.packet.data[5] = esbAddress.addrPrefix;
	tx_frame.packet.data[6] = (uint8_t)esbAddress.addr;
	tx_frame.packet.data[7] = (uint8_t)(esbAddress.addr >> 8);
	tx_frame.packet.data[8] = (uint8_t)(esbAddress.addr >> 16);
	tx_frame.packet.data[9] = (uint8_t)(esbAddress.addr >> 24);

	esb_teachmode_checkTimeout();
	
	if(nrf_esb_get_mode() != NRF_ESB_MODE_PTX)
	{
		nrf_esb_set_mode(NRF_ESB_MODE_PTX);
	};
	
	if(nrf_esb_add_packet_to_tx_fifo(0, tx_frame.rawData, 10 + 3, NRF_ESB_PACKET_USE_ACK) == false)
	{
		__nop();
	};
	
	teachModeTxCount++;
}

#ifndef NO_SOCKET_SUPPORT
void esb_send_socket_cmd(void)
{
	if((rfMode == rfMode_teach_pending) || (rfMode == rfMode_teach_rx))
		return;
	
	rfMode = rfMode_tx;
	
	union Frame tx_frame;
	uint32_t    keycode = 0;
	
	if(ble_keyCode)
		keycode = ble_keyCode;
	else if(esb_keyCode)
		keycode = esb_keyCode;
	else
	{
		if(esb_keyCode && m_hs_timeout)
			keycode = 0x00000200;
		else
			keycode = 0x08000000;
	}
	
	tx_frame.packet.length 	= 4;
	tx_frame.packet.command = RF_BUTTON;
	tx_frame.packet.data[0] = (keycode >> 24) & 0xff;	
	tx_frame.packet.data[1] = (keycode >> 16) & 0xff;	
	tx_frame.packet.data[2] = (keycode >> 8)  & 0xff;	
	tx_frame.packet.data[3] = 	keycode        & 0xff;	
	
	if(nrf_esb_get_mode() != NRF_ESB_MODE_PTX)
	{
		nrf_esb_set_mode(NRF_ESB_MODE_PTX);
	};
	
	if(nrf_esb_add_packet_to_tx_fifo(1, tx_frame.rawData, tx_frame.packet.length + 3, NRF_ESB_PACKET_USE_ACK))
	{
		__nop();
	};
}
#endif


/**@brief IRQHandler used for execution context management. 
  *        Any available handler can be used as we're not using the associated hardware.
  *        This handler is used to initiate ESB RX/TX
  */
void TIMESLOT_BEGIN_IRQHandler(void)
{
	timeslotRunning=1;
	static uint_fast8_t timeslotBeginCounter = 0;
	
#ifdef WDT_ON
	watchdog_reset_timeslot();
#endif
	
#ifndef NO_SOCKET_SUPPORT
	if ((rfMode == rfMode_teach_tx) || ((rfMode == rfMode_tx) && (m_feedback_error_count == 0) && m_ble_flags.central_connected))
#else
	if (rfMode == rfMode_teach_tx)
#endif
	{
		nrf_esb_init(NRF_ESB_MODE_PTX); // Primary Transmitter mode
	} 
	else
	{
		nrf_esb_init(NRF_ESB_MODE_PRX); // Primary Receiver mode
	};
	
	if(rfMode == rfMode_teach_pending)
	{
		rfMode = rfMode_teach_rx;
	}
	
	nrf_esb_set_channel(esbAddress.channel);
#ifndef ESB_DYN_ACK
	nrf_esb_disable_dyn_ack();
#endif
	
#ifndef NO_SOCKET_SUPPORT
	if((rfMode != rfMode_normal) && (rfMode != rfMode_tx) && (m_feedback_error_count == 0))
#else
	if (rfMode != rfMode_normal)
#endif
	{
#ifdef ESB_DYN_ACK
		nrf_esb_disable_dyn_ack();
#endif
		nrf_esb_set_base_address_0(teachAddr);
		nrf_esb_set_base_address_1(esbAddress.addr);
		nrf_esb_set_address_prefix_byte(0, teachAddrPrefix);
		nrf_esb_set_address_prefix_byte(1, esbAddress.addrPrefix + 1);
		nrf_esb_set_address_prefix_byte(2, esbAddress.addrPrefix + 2);
		nrf_esb_set_address_prefix_byte(3, esbAddress.addrPrefix + 3);
		nrf_esb_set_address_prefix_byte(4, esbAddress.addrPrefix + 4);
		nrf_esb_set_address_prefix_byte(5, esbAddress.addrPrefix + 5);
		nrf_esb_set_enabled_prx_pipes((1 << 0) | (1 << esbAddress.pipe));
		
		if((esbAddress.addr == defaultAddr) && (esbAddress.channel == defaultChannel) && (esbAddress.addrPrefix == defaultAddrPrefix))
		{
			m_forced_teach = true;
		}
		ble_esb_led_on_event();
	} 
	else
	{
#ifdef ESB_DYN_ACK
		nrf_esb_enable_dyn_ack();
#endif
		nrf_esb_set_base_address_length(NRF_ESB_BASE_ADDRESS_LENGTH_4B);
		nrf_esb_set_base_address_0(esbAddress.addr);
		nrf_esb_set_base_address_1(esbAddress.addr);
		nrf_esb_set_address_prefix_byte(0, esbAddress.addrPrefix);
		nrf_esb_set_address_prefix_byte(1, esbAddress.addrPrefix + 1);
		nrf_esb_set_address_prefix_byte(2, esbAddress.addrPrefix + 2);
		nrf_esb_set_address_prefix_byte(3, esbAddress.addrPrefix + 3);
		nrf_esb_set_address_prefix_byte(4, esbAddress.addrPrefix + 4);
		nrf_esb_set_address_prefix_byte(5, esbAddress.addrPrefix + 5);

#ifndef NO_SOCKET_SUPPORT		
		if(m_ble_flags.central_connected && (rfMode == rfMode_tx) && (m_feedback_error_count == 0))
			nrf_esb_set_enabled_prx_pipes(1 << esbAddress.pipe | 1 << SOCKET_PIPE);
		else
			nrf_esb_set_enabled_prx_pipes(1 << esbAddress.pipe);
#else
		nrf_esb_set_enabled_prx_pipes(1 << esbAddress.pipe);
#endif
	};
	nrf_esb_set_crc_length(NRF_ESB_CRC_LENGTH_2_BYTE);
	nrf_esb_set_datarate(NRF_ESB_DATARATE_1_MBPS);
	nrf_esb_set_max_number_of_tx_attempts(8);
	nrf_esb_set_output_power(NRF_ESB_OUTPUT_POWER_0_DBM);
	nrf_esb_set_retransmit_delay(1000); // minimum ist 900us bei 1Mbps, 32 byte payload im TX und RX frame...
	//nrf_esb_set_xosc_ctl(NRF_ESB_XOSC_CTL_AUTO);
	nrf_esb_set_xosc_ctl(NRF_ESB_XOSC_CTL_MANUAL);
	
	nrf_esb_flush_tx_fifo(0);
	nrf_esb_flush_tx_fifo(1);
	nrf_esb_flush_tx_fifo(2);
	nrf_esb_flush_tx_fifo(3);
	nrf_esb_flush_tx_fifo(4);
	nrf_esb_flush_tx_fifo(5);

	nrf_esb_enable();
	timeslotBeginCounter++;
	
	if((tx_frame.packet.command != RF_EMPTY_FRAME)
	   && (rfMode == rfMode_normal)
	   && ((esb_keyCode != 0) || (timeslotBeginCounter & 1))) // if esb_keycode equals zero we skip the packet every 2nd time. This is needed for wake-up, as acknowledge-packets with data are too long and take too much time for wake-up.
	{
		nrf_esb_add_packet_to_tx_fifo(tx_pipe, tx_frame.rawData, tx_frame.packet.length + 3, NRF_ESB_PACKET_USE_ACK);
	};
	
	if(rfMode == rfMode_teach_tx)
	{
		esb_send_moduleID();
	};
	
#ifndef NO_SOCKET_SUPPORT
	if((rfMode == rfMode_tx) && (m_feedback_error_count == 0) && m_ble_flags.central_connected)
	{
		esb_send_socket_cmd();
	};
#endif
}

static uint_fast8_t newChannel;
static uint_fast8_t newPipe;
static uint32_t newAddr;
static uint_fast8_t newAddrPrefix;

void nrf_esb_rx_data_ready(uint32_t rx_pipe, int32_t rssi)
{
	static uint_fast8_t feedbackState = 0;
	Dp2_packet_t dp2Packet;
	
	if((rx_pipe != 0) && (rfMode == rfMode_teach_rx) && !m_forced_teach)
	{
		esb_teachmode_stop();
	}
	
	bool success;
	union Frame rx_frame;
	uint32_t length;
	
	success = nrf_esb_fetch_packet_from_rx_fifo(rx_pipe, (uint8_t *)&rx_frame, &length);
	
	if((success == true)
	  && (((rx_frame.packet.length + 3) <= length)
	  || ((rx_frame.packet.length == NRF_ESB_CONST_MAX_PAYLOAD_LENGTH) && (length == NRF_ESB_CONST_MAX_PAYLOAD_LENGTH)))) // workaround for buggy handsets that report a wrong packet length...
	{
		esbRadioActiveTimer     = ESB_RADIO_ACTIVE_TIMEOUT;
		tx_frame.packet.command = RF_EMPTY_FRAME;  // mark the tx-frame as empty to avoid sending it twice.
		
		switch(rx_frame.packet.command)
		{
			case RF_MODULID:
				teachModeTxCount = 0;
				esb_send_moduleID();
				rfMode = rfMode_teach_tx;
				break;
			
			case RF_BUTTON:
				esb_keyCode = rx_frame.packet.data[3]
				            | rx_frame.packet.data[2] << 8
				            | rx_frame.packet.data[1] << 16
				            | rx_frame.packet.data[0] << 24;
				esb_keyCodeTimer = KEY_CODE_TIMEOUT;

				// This function is called by an interrupt routine.
				// We will call ble_keyCode_received_event() from esb_handler() which is called from the main loop...

				static int tx_fifo_count;
				tx_fifo_count = nrf_esb_get_tx_fifo_packet_count(rx_pipe);
				if(tx_fifo_count == 0)
				{
					if(m_flash_count != 0)
					{
						tx_frame.packet.command = RF_LED_AUTO;
						tx_frame.packet.length  = 6;
						tx_frame.packet.data[0] = 0xFF; // Setting LED-code to 0x000000FF is needed by a couple of buggy RF remotes.
						tx_frame.packet.data[1] = 0;
						tx_frame.packet.data[2] = 0;
						tx_frame.packet.data[3] = 0;
						tx_frame.packet.data[4] = m_flash_interval;
						tx_frame.packet.data[5] = m_flash_count;
					
						m_flash_count    = 0;
						m_flash_interval = 0;
					}
					else
					{
						if ((feedbackState & 1)
						 && (dp2_getNextPacket(&dp2Packet) != 0))
						{
							tx_frame.packet.command = dp2Packet.command;
							tx_frame.packet.length  = dp2Packet.length;
							if (dp2Packet.length > sizeof(tx_frame.packet.data))
								dp2Packet.length = sizeof(tx_frame.packet.data);
							memcpy(tx_frame.packet.data, dp2Packet.data, dp2Packet.length);
						}
						else
						{
							tx_frame.packet.command = RF_STATUS;
							tx_frame.packet.length  = ble_read_feedback_event(tx_frame.packet.data);
#ifdef USE_OLD_STYLE_LED_ON_OFF
							static uint_fast8_t oldStyleLedIndex;
							oldStyleLedIndex++;
							if (oldStyleLedIndex > 2)
								oldStyleLedIndex = 0;
							
							switch(oldStyleLedIndex)
							{
								default:
								//case 0:
									tx_frame.packet.command = RF_LED_ON;
									*(__packed uint32_t*)tx_frame.packet.data &= *(__packed uint32_t*)(tx_frame.packet.data + (tx_frame.packet.length / 2));
									break;
								case 1:
									tx_frame.packet.command = RF_LED_OFF;
									*(__packed uint32_t*)tx_frame.packet.data = ~*(__packed uint32_t*)tx_frame.packet.data;
									*(__packed uint32_t*)tx_frame.packet.data &= *(__packed uint32_t*)(tx_frame.packet.data + (tx_frame.packet.length / 2));
									break;
								case 2:
									tx_frame.packet.command = RF_LED_AUTO;
									*(__packed uint32_t*)tx_frame.packet.data = ~*(__packed uint32_t*)(tx_frame.packet.data + (tx_frame.packet.length / 2));
									break;
							}
							
							if (tx_frame.packet.length & 1)
							{
								tx_frame.packet.data[4] = tx_frame.packet.data[tx_frame.packet.length - 1];	//	copy status byte if it is available
								tx_frame.packet.length = 5;
							} else
							{
								tx_frame.packet.length = 4;
							}
#endif
						}
						feedbackState ++;
						feedbackState &= 1;
					}
					
					tx_pipe = rx_pipe;
					nrf_esb_add_packet_to_tx_fifo(rx_pipe, tx_frame.rawData, tx_frame.packet.length + 3, NRF_ESB_PACKET_USE_ACK);
				}
				break;
				
			case RF_WAKE_UP:
				// todo...
				break;
			
			case RF_CHANGE_ADDR:
				newChannel    = rx_frame.packet.data[0];
				newPipe       = rx_frame.packet.data[1];
				newAddrPrefix = rx_frame.packet.data[2];
				newAddr       = rx_frame.packet.data[3]
				              | rx_frame.packet.data[4] << 8
				              | rx_frame.packet.data[5] << 16
				              | rx_frame.packet.data[6] << 24;
				break;
			
			case RF_SET_CURRENT_ADDR:
				if(rfMode != rfMode_normal)
				{
					rfMode = rfMode_teach_change_channel;
					esb_teachmode_checkTimeout();
					nrf_esb_disable();
				}
				break;
			
#ifndef NO_SOCKET_SUPPORT
			case RF_LED_ON:
			case RF_LED_OFF:
			{
				if(m_ble_flags.central_connected)
				{
					uint32_t led_code = 0;
			
					led_code = rx_frame.packet.data[0]
					         | rx_frame.packet.data[1] << 8
					         | rx_frame.packet.data[2] << 16
					         | rx_frame.packet.data[3] << 24;
				
					ble_esb_feedback_event(rx_frame.packet.command, led_code, 0);
				}
			} break;
#endif
			default:
			{
				Dp2_packet_t request;
				request.length = rx_frame.packet.length;
				request.command = rx_frame.packet.command;
				BUILD_BUG_ON(sizeof(request.data) < NRF_ESB_CONST_MAX_PAYLOAD_LENGTH - 3); // length cannot be larger than NRF_ESB_CONST_MAX_PAYLOAD_LENGTH - see beginning of switch statement
				memcpy(request.data, rx_frame.packet.data, request.length);
				
				Dp2_packet_t answer;
				answer.length = sizeof(tx_frame.packet.data);
				
				int errCode = dp2_packetReceived(&request, &answer);
				
				if (errCode == DP2_SUCCESS)
				{
					tx_frame.packet.command = answer.command;
					tx_frame.packet.length = answer.length;
					memcpy(tx_frame.packet.data, answer.data, answer.length);
					
					tx_pipe = rx_pipe;
					nrf_esb_add_packet_to_tx_fifo(rx_pipe, tx_frame.rawData, tx_frame.packet.length + 3, NRF_ESB_PACKET_USE_ACK);
				}
			} break;
		}
	}
}


void nrf_esb_tx_success(uint32_t tx_pipe, int32_t rssi)
{
	if(nrf_esb_get_mode() == NRF_ESB_MODE_PTX)
	{
		nrf_esb_set_mode(NRF_ESB_MODE_PRX);
		
		switch(rfMode)
		{
#ifndef NO_SOCKET_SUPPORT
			case rfMode_tx:
				rfMode = rfMode_normal;
				m_feedback_error_count = 0;
				break;
#endif
			
			case rfMode_teach_tx:
				rfMode = rfMode_teach_rx;
				esb_teachmode_checkTimeout();
				break;
			
			default:
				break;
		}
	}
}

void nrf_esb_tx_failed(uint32_t tx_pipe)
{
	if(nrf_esb_get_mode() == NRF_ESB_MODE_PTX)
	{
		switch(rfMode)
		{
#ifndef NO_SOCKET_SUPPORT
			case rfMode_tx:
				nrf_esb_set_mode(NRF_ESB_MODE_PRX);
				rfMode = rfMode_normal;
				
				nrf_esb_flush_tx_fifo(SOCKET_PIPE);
				m_feedback_error_count++;
				break;
#endif
			
			case rfMode_teach_tx:
#if (TEACHMODE_MAX_TXCOUNT > 0)
				if(teachModeTxCount < TEACHMODE_MAX_TXCOUNT)
				{
					esb_send_moduleID();
				} else
#endif
				{
					nrf_esb_set_mode(NRF_ESB_MODE_PRX);
					rfMode = rfMode_teach_rx;
				}
				esb_teachmode_checkTimeout();
				break;
				
			default:
				nrf_esb_set_mode(NRF_ESB_MODE_PRX);
				break;
		}
	}
}

void nrf_esb_disabled (void)
{
	switch (rfMode)
	{
		case rfMode_teach_change_channel:
			memset(&esbAddress, 0xff, sizeof(esbAddress));
			esbAddress.addr       = newAddr;
			esbAddress.addrPrefix = newAddrPrefix;
			esbAddress.channel    = newChannel;
			esbAddress.pipe       = newPipe;
			
			// Storing the new address to flash memory can take quite a long time.
			// Such a long task will kill the Bluetooth Smart stack, therby storeing the new address
			// to flash has to be done in the main-loop and with lower priority...
			addrHasChanged = -1;
			
			nrf_esb_set_channel(esbAddress.channel);
			nrf_esb_set_base_address_0(esbAddress.addr);
			nrf_esb_set_base_address_1(esbAddress.addr);
			nrf_esb_set_address_prefix_byte(0, esbAddress.addrPrefix);
			nrf_esb_set_address_prefix_byte(1, esbAddress.addrPrefix + 1);
			nrf_esb_set_address_prefix_byte(2, esbAddress.addrPrefix + 2);
			nrf_esb_set_address_prefix_byte(3, esbAddress.addrPrefix + 3);
			nrf_esb_set_address_prefix_byte(4, esbAddress.addrPrefix + 4);
			nrf_esb_set_address_prefix_byte(5, esbAddress.addrPrefix + 5);
			nrf_esb_set_enabled_prx_pipes(1 << esbAddress.pipe);
			
			if(timeslotIsActive())
				nrf_esb_enable();
			
			esb_teachmode_stop();
			ble_esb_teach_finished_succesfull_event();
			break;
			
		case rfMode_teach_pending:
			nrf_esb_set_base_address_0(teachAddr);
			nrf_esb_set_address_prefix_byte(0, teachAddrPrefix);
			nrf_esb_set_enabled_prx_pipes(0x01);

			if(timeslotIsActive())
				nrf_esb_enable();

			ble_esb_led_on_event();
			break;
		
		default:
			break;
	};
}

void esb_timer_10ms(void)
{
	static bool         lastButton    = false;
	static uint_fast8_t buttonCounter = 0;
	static uint_fast8_t buttonTimer   = ESB_BUTTON_TIMEOUT;
	bool                button        = ble_is_btn_pressed_request_event();
	
	if(esbStop != 0)
		return;
	
#ifdef WDT_ON
	watchdog_reset_esbTimer();
#endif
	
	if(button != lastButton)
	{
		buttonTimer = ESB_BUTTON_TIMEOUT;
		if(button)
		{
			buttonCounter++;
		}
	}
	
	if(buttonTimer == 0)
	{
		switch(buttonCounter)
		{
			case 1:
				if(rfMode != rfMode_normal)
				{
					esb_teachmode_stop();
				}
				buttonCounter = 0;
				break;
				
			case 2:
				m_forced_teach = true;
				esb_teachmode_start();
				buttonCounter = 0;
				break;
			
			case 4:
				buttonCounter = 252;
				buttonTimer = ESB_DELETE_BUTTON_TIMEOUT;
				ble_esb_led_on_event();
				break;
			
			case 253:
				esb_generateRandomAddress();
				ble_esb_address_deleted_event();
			
			case 252:
			case 254:
			case 255:
				buttonCounter = 0;
				ble_esb_led_off_event();
				break;
			
			default:
				buttonCounter = 0;
				break;
		}
	};
	
	if(buttonCounter == 6)
		buttonCounter = 5;
	
	if(buttonCounter == 255)
		buttonCounter = 254;
	
	if(buttonTimer != 0)
		buttonTimer--;
	
	if(teachModeTimer == 0)
	{
		esb_teachmode_stop();
	} 
	else
		teachModeTimer--;
	
	lastButton = button;
	
	if(esb_keyCodeTimer == 0)
	{
		esb_keyCode = 0;
		// We will call ble_keyCode_received_event() from esb_handler() which is called from the main loop...
		tx_frame.packet.command = RF_EMPTY_FRAME; // Delete the last tx-frame as it is obsolete now. In addition this is needed for (faster) wake-up.
	}
	else
	{
		esb_keyCodeTimer--;
	};
	
	if(esbRadioActiveTimer != 0)
		esbRadioActiveTimer--;
	
#ifndef NO_SOCKET_SUPPORT
	if(m_ble_flags.central_connected && (m_feedback_error_count == 0))
	{
		if(m_feedback_timeout)
			m_feedback_timeout--;
	
		if(m_feedback_sync_timer && (ble_keyCode == 0) && (esb_keyCode == 0))
			m_feedback_sync_timer--;
		
		if(m_hs_timeout)
			m_hs_timeout--;
	}
#endif
}

void esb_flash_feedback(uint_fast8_t interval, uint_fast8_t count)
{
	m_flash_interval = interval;
	m_flash_count    = count;
}

void esb_handler(void)
{
	static volatile int txCount;
	static typeof(esb_keyCode) last_esb_keyCode = 0;
	
	if(esbStop != 0)
		return;
	
	if(addrHasChanged != 0)
	{
		addrHasChanged = 0;
		rf_member_store(&esbAddress);
	};
	
	// esb_keyCode is changed by interrupt routines.
	// We need to call ble_keyCode_received_event() from here, this function is called from the main loop.
	if(last_esb_keyCode != esb_keyCode)
	{
		last_esb_keyCode = esb_keyCode;
		ble_keyCode_received_event(last_esb_keyCode | ble_keyCode | dp2GetKeycode());
	}

	txCount = nrf_esb_get_tx_fifo_packet_count(0);
	
	if(rfMode == rfMode_teach_tx)
	{
		if(txCount == 0)
		{
			esb_send_moduleID();
		};
	}
	
#ifndef NO_SOCKET_SUPPORT
	if(m_ble_flags.central_connected && (rfMode != rfMode_teach_pending) && (rfMode != rfMode_teach_rx) && (timeslotRunning == 1))  // timeslot must be running 
	{
		static uint32_t old_socket_keycode = 0;
		
		if(((esb_keyCode & SOCKET_KEYCODE) != 0) || ((ble_keyCode & SOCKET_KEYCODE) != 0))
		{
			if((esb_keyCode & SOCKET_KEYCODE) != 0)
				m_hs_timeout = HS_TIMEOUT;
			
			if(old_socket_keycode == 0)
			{
				old_socket_keycode     = SOCKET_KEYCODE;
				m_feedback_error_count = 0;
			}
		}
		else
			old_socket_keycode = 0;
				
		if(m_feedback_error_count == 0)
		{
			static bool get_feedbacks_flag = false;
		
			if(rfMode == rfMode_tx)
			{
				if(txCount == 0)
				{
						esb_send_socket_cmd();
				};
			}
			
			if((ble_keyCode > 0) && (rfMode == rfMode_normal))
			{
				rfMode	= rfMode_tx;
			
				if((ble_keyCode & SOCKET_KEYCODE) != 0)
				{
					m_feedback_timeout = FEEDBACK_TIMEOUT;
					get_feedbacks_flag = true;
				}
			}
			else if(get_feedbacks_flag && (esb_keyCode == 0))
				rfMode = rfMode_tx;
		
			if((esb_keyCode > 0) && (m_hs_timeout == 0))
			{
				m_feedback_timeout = FEEDBACK_TIMEOUT;
				get_feedbacks_flag = true;
			}
		
			if(m_feedback_timeout == 0)
			{
				get_feedbacks_flag = false;
				rfMode = rfMode_normal;
			}
		
			if((m_feedback_sync_timer == 0) && (rfMode == rfMode_normal))
			{
				get_feedbacks_flag    = true;
				m_feedback_sync_timer = FEEDBACK_SYNC_TIMER;
				m_feedback_timeout    = FEEDBACK_TIMEOUT;
			}
		}
	}
	else
		m_feedback_error_count = 0;
#endif
	
	static volatile int pipes;
	pipes = nrf_esb_get_enabled_prx_pipes();
	static volatile int enabled;
	enabled = nrf_esb_is_enabled();
	if(pipes | enabled)
	{
		__nop();
	}

}

