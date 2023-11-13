/**
******************************************************************************
* @file    esb_driver.c
* @author  Nils Grosse-Goeddinghaus
* @version V1.0
* @date    15.01.20
* @brief   this module handles the esb functioalty. 
******************************************************************************
* @attention
*
*
*
******************************************************************************
*/

#include <string.h>

#include "esb_hsTransmitter.h"
#include "nrf_drv_rng.h"
#include "nrf_gpio.h"


#include "gpio_rf56_hal.h"
#include "hardware_information.h"
#if !(defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) && !(defined (_USE_DO_TIMER_MODULE_FOR_ESB)) 
#include "esb_timer_driver.h"
#include "rf_storage.h"
#else
#include "event_dispatcher.h" 
#include "key_event.h"

#include "persistent_storage.h"
#include "do_timer.h"
#endif

#include "esb_constants.h"

#include "led_event.h"
#include "io_driver.h"
#include "nrf_esb.h"
#include "led_control_api.h"


typedef struct
{
	uint8_t base_address[RF_LENGTH_OF_ADDRESS];       ///< @brief  base adress
	uint8_t pre_address;                              ///< @brief  pre adress
} rf_address_t;

typedef struct
{
	rf_address_t address;                             ///< @brief  rf adress 
	uint8_t channel;                                  ///< @brief  rf channel 
//#ifdef FEATURE_TEACH_TWO_DEVICES
	uint8_t numberOfDevices;
//#endif
} rf_communication_data_t;

typedef struct
{
	uint8_t length;                                   ///< @brief data length
	uint8_t sequenz;                                  ///< @brief sequence number
	uint8_t command;                                  ///< @brief command
	uint8_t payload[29];                              ///< @brief payload
} frame_t;

typedef union
{
	frame_t frame;                                    ///< @brief  rf esb frame
	uint8_t daten[sizeof(frame_t)];                   ///< @brief  rf esb daten 
} rf_frame_t;

typedef enum
{
	TEACH_STATE_IDLE = 0,                             ///< @brief  teach state idle
	TEACH_STATE_START,                                ///< @brief  teach state start
	TEACH_STATE_SEND_ID,                              ///< @brief  teach state send id 
	TEACH_STATE_WAIT_ID,                              ///< @brief  teach state wait for id
	TEACH_STATE_WAIT_ACK,                             ///< @brief  teach state wait for ack
	TEACH_STATE_SEND_ADDRESS,                         ///< @brief  teach state send address
	TEACH_STATE_WAIT_ADDRESS,                         ///< @brief  teach state wait for address
	TEACH_STATE_SEND_CURR_ADDRESS,                    ///< @brief  teach state send current address
	TEACH_STATE_WAIT_CURR_ADDRESS,                    ///< @brief  teach state wait for current address
	TEACH_STATE_DONE,                                 ///< @brief  teach state send done 
	TEACH_STATE_RESET,                                ///< @brief  teach state send reset 
} teach_status_t;

typedef enum
{
	TEACH_REQUEST_NEW_DEVICE,                         ///< @brief  teach reuqest new device 
	TEACH_REQUEST_STOP_TEACHING,                      ///< @brief  teach request stop teaching
	TEACH_REQUEST_RESET_DEVICES,                      ///< @brief  teach request reset device
	TEACH_REQUEST_NO_REQUEST,                         ///< @brief  teach request do noting 
	TEACH_REQUEST_WAIT_FOR_UBB
} esb_teaching_request_t;


static bool          _ledMemoryActive    = false;                    ///< @brief  led set control 
static const uint8_t _pipeArray[]        = {0,4,5,1};                ///< @brief  pipe array
static const uint8_t _teachingAddress[4] = {0x69,0x96,0x69,0x96};    ///< @brief  teach address
static const uint8_t _teachingPreaddress = 0x90;                     ///< @brief  pre adress 

static void esbEventHandler(nrf_esb_evt_t const * event);

const nrf_esb_config_t _esbDefaultTxConfiguration = {
	/**< Enhanced ShockBurst protocol. */
	.protocol = NRF_ESB_PROTOCOL_ESB_DPL,
	/**< Enhanced ShockBurst mode. */
	.mode = NRF_ESB_MODE_PTX,
	/**< Enhanced ShockBurst event handler. */
	.event_handler = esbEventHandler,
	/**< Enhanced ShockBurst bitrate mode. */
	.bitrate = NRF_ESB_BITRATE_1MBPS,
	/**< Enhanced ShockBurst CRC modes. */
	.crc = NRF_ESB_CRC_16BIT,
	/**< Enhanced ShockBurst radio transmission power mode.*/
	.tx_output_power = NRF_ESB_TX_POWER_4DBM,
	/**< The delay between each retransmission of unacknowledged packets. */
	.retransmit_delay = 1000,
	/**< The number of retransmissions attempts before transmission fail. */
	.retransmit_count = 6,
	/**< Enhanced ShockBurst transmission mode. */
	.tx_mode = NRF_ESB_TXMODE_AUTO,
	/**< nRF radio interrupt priority. */
	.radio_irq_priority = 1,
	/**< ESB event interrupt priority. */
	.event_irq_priority = 2,
	/**< Length of the payload (maximum length depends on the platforms that are used on each side). */
	.payload_length = 32,
	/**< Enable or disable selective auto acknowledgment. */
	.selective_auto_ack = false
};

static uint8_t _esbFlags = ESB_FLAG_NONE;                                          ///< @brief  esb flags
static uint8_t _rxPayload[NRF_ESB_MAX_PAYLOAD_LENGTH];                             ///< @brief  payload value 
static uint8_t _useDefaultAddress = false;                                         ///< @brief  flag to keep track if default address is used or not


static bool    _communicationPartnerAwake = false;                                 ///< @brief  communication partner awake flag
static bool    _keycodeTransmissionPending = false;                                ///< @brief  pending for communication flag
static uint8_t _wakeupSuccessCounter = 0;                                          ///< @brief  wake up counter 

static bool                   _teachingLedActive    = false;                       ///< @brief  teaching activated flag 
static esb_teaching_request_t _requestBeeingHandled = TEACH_REQUEST_NO_REQUEST;    ///< @brief  teaching request 

static rf_communication_data_t _rfCommunicationData;                               ///< @brief  communication frame

#ifdef FEATURE_TEACH_TWO_DEVICES
static uint8_t _newDeviceNumber = 0;                                               ///< @brief  deive number for second device 
#endif

#ifndef _USE_DO_TIMER_MODULE_FOR_ESB
static uint32_t _esbContiniousTimerIndex = TIMER_INVALID_TIMER;                     ///< @brief  esb continios timer 
static uint32_t _esbWakeupTimerIndex = TIMER_INVALID_TIMER;                         ///< @brief  esb wakeup timer 
static uint32_t _ledTimerIndex = TIMER_INVALID_TIMER;                               ///< @brief  led timer 

#define RF_PARTNER_AWAKE_TIEMOUT           1000    ///< @brief  communication timeout value
#define RF_CONTINIOUS_TIMEOUT              3000    ///< @brief  continious timeout value 
#endif 

// prototypes 
static bool sendRfFrame(uint32_t,rf_frame_t*,uint8_t, bool forceFlush);
static void generateNewChannel(uint8_t* channelBuffer);
static void generateNewAddress(rf_address_t* adressPointer);
static uint8_t generateRandomByte(void);
static void esbServiceTimer(void);

#if (defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) || (defined (_USE_DO_TIMER_MODULE_FOR_ESB)) 
#define ESB_SALT1 1 
PERSTORE_GENERATE_STORAGE_INFORMATION(ESB_SALT1)

#define ESB_TIMER_SPEED 1
DO_TIMER_REGISTER_HANDLER(esbServiceTimer, ESB_TIMER_SPEED);

#define ESB_TIME_25MS                      (25/ESB_TIMER_SPEED)
#define ESB_SLEEP_DELAY_TIME               (5000/ESB_TIMER_SPEED) 
#define RF_PARTNER_AWAKE_TIEMOUT           (1000/ESB_TIMER_SPEED)    ///< @brief  communication timeout value
#define RF_CONTINIOUS_TIMEOUT              (3000/ESB_TIMER_SPEED)    ///< @brief  continious timeout value 
#define ESB_LED_TIMER                      (200/ESB_TIMER_SPEED)

#define TIMER_INVALID_TIMER                UINT16_MAX 

static uint16_t _esbContiniousTimerIndex = TIMER_INVALID_TIMER;                     ///< @brief  esb continios timer 
static uint16_t _esbWakeupTimerIndex = TIMER_INVALID_TIMER;                         ///< @brief  esb wakeup timer 
static uint16_t _ledTimerIndex = TIMER_INVALID_TIMER;                               ///< @brief  led timer 
static uint16_t _sleepTimer = ESB_SLEEP_DELAY_TIME; 

static void esbServiceTimer(void)
{
	static uint8_t time25ms = ESB_TIME_25MS; 
	
	bool keepHandheldAwake = false; 
	bool isTeachModeActive = false; 
	
	isTeachModeActive = esb_teachModeHandler();
	
	if(isTeachModeActive == false)
	{
		esb_wakeUpControls();
	}
	
	if(time25ms == 0)
	{
		time25ms = ESB_TIME_25MS; 

		bool continiousModeActive = false;
		
		// set Flag if necessary to keep the remote awake
		bool buttonPressActive = io_checkButtons();
		if(buttonPressActive == false)
		{
			// set Flag if necessary to keep the remote awake
			continiousModeActive = esb_checkContiniousMode();
		}

		// set Flag if necessary to keep the remote awake
		if((buttonPressActive == true) ||
			 (continiousModeActive == true))
		{
			keepHandheldAwake = true;
		}
		else
		{
			keepHandheldAwake = false; 
		}
	}
	else
	{
		time25ms--; 
	}
	
	if (keepHandheldAwake == true)
	{
		_sleepTimer = ESB_SLEEP_DELAY_TIME; 
//		timer_resetTimer(_sleepTimerIndex, SLEEP_DELAY_TIME);
	}
	else
	{
		_sleepTimer--; 
		if(_sleepTimer == 0)
			ioCheckSleep();
	}
	
	if((_esbContiniousTimerIndex != TIMER_INVALID_TIMER) && (_esbContiniousTimerIndex > 0))
		_esbContiniousTimerIndex--; 
	
	if((_esbWakeupTimerIndex != TIMER_INVALID_TIMER) && (_esbWakeupTimerIndex > 0))
		_esbWakeupTimerIndex--; 

}

static void esb_send_led_event (uint8_t number, uint8_t state)
{
	///< LED ON/OFF events
	typedef struct
	{
		led_set_event_t config;
		uint8_t ledNumber[PROJECT_NUMBER_OF_LEDS];
	} __attribute__((packed)) led_set_event_t;
	
	led_set_event_t ledEvt;

	uint8_t sizeOfLedsEvent = sizeof(led_set_event_t) + PROJECT_NUMBER_OF_LEDS;
	
	ledEvt.config.synchronized = false;
	ledEvt.config.command = state; 
	ledEvt.ledNumber[number] = number; 
	ERROR_CHECK(eventDispatcher_UpdateEvent(FEAT_ID_LOCAL_NODE,
																					EVENT_TYPE_LEDS,
																					&ledEvt,
																					sizeOfLedsEvent,
																					sizeOfLedsEvent));
}    
#endif 

/***************************************************************************//**
 *
 * @fn saveAddressConfiguration(rf_communication_data_t* dataForStorage)
 *
 * @brief  save adress to the storage  
 *
 * @param  rf_communication_data_t* dataForStorage  -> date to save 
 *
 * @return false    
 * 
 ******************************************************************************/  
static bool saveAddressConfiguration(rf_communication_data_t* dataForStorage)
{
	#if !(defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) && !(defined (_USE_DO_TIMER_MODULE_FOR_ESB)) 
	uint8_t dataBuffer[sizeof(rf_communication_data_t)];
	memcpy((void*)dataBuffer, (void*)dataForStorage, sizeof(rf_communication_data_t));

	rf_storage_set_rf_add_table(dataBuffer, sizeof(rf_communication_data_t));
	#endif 
	return false;
}


/***************************************************************************//**
 *
 * @fn bool loadAddressConfiguration(rf_communication_data_t* dataFromStorage)
 *
 * @brief  load adress from the storage  
 *
 * @param  rf_communication_data_t* dataForStorage  -> date to save 
 *
 * @return false    
 *
 ******************************************************************************/  
static bool loadAddressConfiguration(rf_communication_data_t* dataFromStorage)
{
	uint8_t dataBuffer[sizeof(rf_communication_data_t)];
	rf_storage_get_rf_add_table(dataBuffer, sizeof(rf_communication_data_t));

	memcpy((void*)dataFromStorage, (void*)dataBuffer, sizeof(rf_communication_data_t));

	return false;
}


/***************************************************************************//**
 *
 * @fn static void generateNewAddress(rf_address_t* adressPointer)
 *
 * @brief  generates a rf address pointer   
 *
 * @param  rf_address_t* adressPointer
 *
 * @return none     
 * 
 ******************************************************************************/  
static void generateNewAddress(rf_address_t* adressPointer)
{
	uint8_t randomByteIndex = 0;
	uint8_t randomBuffer[RF_LENGTH_OF_ADDRESS + RF_LENGHT_OF_PRE_ADDRESS] = {0};
	nrf_drv_rng_rand(randomBuffer, RF_LENGTH_OF_ADDRESS);
	for (randomByteIndex = 0; randomByteIndex < RF_LENGTH_OF_ADDRESS; ++randomByteIndex)
	{
		adressPointer->base_address[randomByteIndex] = randomBuffer[randomByteIndex];
	}
	adressPointer->pre_address = (RF_PREADDRESS_MASK & randomBuffer[randomByteIndex]);
}


/***************************************************************************//**
 *
 * @fn static void generateNewChannel(uint8_t* channelBuffer)
 *
 * @brief  generates a random channel from 3...80
 *
 * @param  uint8_t* channelBuffer -> pointer to channel buffer
 *
 * @return none     
 * 
 ******************************************************************************/ 
static void generateNewChannel(uint8_t* channelBuffer)
{
	*channelBuffer = (generateRandomByte() % 78) + 3;
}


/***************************************************************************//**
 *
 * @fn static uint8_t generateRandomByte(void)
 *
 * @brief  generates a random byte  
 *
 * @param  none 
 *
 * @return random byte     
 * 
 ******************************************************************************/ 
static uint8_t generateRandomByte(void)
{
	uint8_t randomByte = 0;
	nrf_drv_rng_rand(&randomByte, 1);
	return randomByte;
}


/***************************************************************************//**
 *
 * @fn static bool sendRfFrame(uint32_t pipe,rf_frame_t* p_frame,uint8_t ack, bool forceFlush)
 *
 * @brief  send a RF Frame 
 *
 * @param  uint32_t pipe  -> current pipe 
 * @param  rf_frame_t* p_frame -> the rf frame / date 
 * @param  uint8_t ack   -> ack flag
 * @param  bool forceFlush -> marker for flusing rx & tx buffer 
 *
 * @return true when done     
 * 
 ******************************************************************************/ 
static bool sendRfFrame(uint32_t pipe, rf_frame_t* p_frame, uint8_t ack, bool forceFlush)
{
	static uint8_t txCount = 0;

	if (forceFlush == true)
	{
		nrf_esb_flush_tx();
		nrf_esb_flush_rx();
	}

	p_frame->frame.sequenz = txCount;
	
	nrf_esb_payload_t payload = {
		.length = p_frame->frame.length + RF_ESB_HEADER_LENGTH,
		.pipe   = pipe,
		.rssi   = 0,// initialize value 0 just as a test-value
		.noack  = (ack != 0) ? 0 : 1
	};
	memcpy(&payload.data, p_frame->daten, (p_frame->frame.length) + RF_ESB_HEADER_LENGTH);
	
	if (nrf_esb_write_payload(&payload)== NRF_SUCCESS)
	{
		txCount++;
		nrf_esb_start_tx();
		return true;
	}
	return false;
}


/***************************************************************************//**
 *
 * @fn bool esb_teachNewDevice(uint8_t newDeviceNumber)
 *
 * @brief  teaches a new device 
 *
 * @param  uint8_t newDeviceNumber -> new device number  
 * 
 * @return true when done     
 * 
 ******************************************************************************/ 
bool esb_teachNewDevice(uint8_t newDeviceNumber)
{
	bool requestWillBeHandled = false;
	if ((_requestBeeingHandled == TEACH_REQUEST_NO_REQUEST) &&
	    (newDeviceNumber > 0) &&
	    (newDeviceNumber <= RF_MAX_NUMBER_OF_TEACHABLE_DEVICES))
	{
#ifdef  _USE_TWO_KEYS_FOR_UBB
			_requestBeeingHandled = TEACH_REQUEST_WAIT_FOR_UBB;
#else
			_requestBeeingHandled = TEACH_REQUEST_NEW_DEVICE; 
#endif
		
#ifdef FEATURE_TEACH_TWO_DEVICES
		_newDeviceNumber = newDeviceNumber;
#endif		
		requestWillBeHandled = true;
	}
	return requestWillBeHandled;
}


/***************************************************************************//**
 *
 * @fn bool esb_resetTeaching(void)
 *
 * @brief  reset teaching 
 *
 * @param  none 
 * 
 * @return true when done     
 * 
 ******************************************************************************/ 
bool esb_resetTeaching(void)
{
	bool requestWillBeHandled = true;
	
	_requestBeeingHandled = TEACH_REQUEST_RESET_DEVICES;
	
	return requestWillBeHandled;
}


/***************************************************************************//**
 *
 * @fn bool esb_stopTeaching(void)
 *
 * @brief  stop teaching 
 *
 * @param  none 
 * 
 * @return true when done     
 * 
 ******************************************************************************/ 
bool esb_stopTeaching(void)
{
	bool requestWillBeHandled = true;
	
	_requestBeeingHandled = TEACH_REQUEST_STOP_TEACHING;
	
	return requestWillBeHandled;
}


/***************************************************************************//**
 *
 * @fn void esb_teachModeHandler()
 *
 * @brief  the esb handler
 *
 * @param  none 
 * 
 * @return True if active, otherwise false
 * 
 ******************************************************************************/

#define TEACH_TIMEOUT     500
#define TEACH_DELAY       500

bool esb_teachModeHandler(void)
{
	static uint16_t delay = SYS_TICK_TIME_BASE_5MS * TEACH_DELAY; 
	static uint8_t    pipeIndex = 0;
	static uint32_t   channelIndex = 0;
	static rf_frame_t frame;

	static rf_address_t   newRfAddress;
	static uint8_t        newRfChannel;
	static teach_status_t teachStatus = TEACH_STATE_IDLE;

	static uint32_t       timeout = TEACH_TIMEOUT; 
	
	uint32_t errCode;
	
	if(timeout > 0)
		timeout--;
	
	switch(_requestBeeingHandled)
	{
		case TEACH_REQUEST_WAIT_FOR_UBB:
		#ifdef  _USE_TWO_KEYS_FOR_UBB
		{
			if(io_getWaitForUbbFlag())
			{
				_requestBeeingHandled = TEACH_REQUEST_NEW_DEVICE; 
			}
		}
		#else
			_requestBeeingHandled = TEACH_REQUEST_NEW_DEVICE; 
		#endif
		break; 
		
		case TEACH_REQUEST_NEW_DEVICE:
		{
			if (teachStatus == TEACH_STATE_IDLE)
			{
				_requestBeeingHandled = TEACH_REQUEST_NO_REQUEST;
				teachStatus           = TEACH_STATE_START;
				_teachingLedActive    = true;
			}
		} break;
			
		case TEACH_REQUEST_STOP_TEACHING:
		{
			teachStatus           = TEACH_STATE_RESET;
			_teachingLedActive    = false;
			_requestBeeingHandled = TEACH_REQUEST_NO_REQUEST;
		} break;
		
		case TEACH_REQUEST_RESET_DEVICES:
		{
			// TODO: implement reset of data
			_requestBeeingHandled = TEACH_REQUEST_NO_REQUEST;
		} break;
		
		default:
			break;
	}
	
	if (_ledMemoryActive == true)
	{
		if (_esbContiniousTimerIndex == 0) // timer_timeoutReached(_esbContiniousTimerIndex) == true)
		{
			_ledMemoryActive = false;
			#if !(defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) && !(defined (_USE_DO_TIMER_MODULE_FOR_ESB)) 
			led_conrol_halSetTeachFeedbackLed(LED_COMMAND_DEACTIVATE);
			#else
			esb_send_led_event(LED_LED1_PIN_NUMBER, LED_COMMAND_DEACTIVATE);
			#endif 
		}
	}
	else if ((_ledTimerIndex == 0) &&
	         (_teachingLedActive == true) &&
	         (teachStatus != TEACH_STATE_IDLE))
	{
		#ifdef _USE_DO_TIMER_MODULE_FOR_ESB
//		nrf_gpio_pin_toggle(FCT_LED);
		esb_send_led_event(LED_LED1_PIN_NUMBER, LED_COMMAND_TOGGLE);
		_ledTimerIndex = ESB_LED_TIMER;
		#else
		led_conrol_halSetTeachFeedbackLed(LED_COMMAND_TOGGLE);
		timer_resetTimer(_ledTimerIndex, 200);
		#endif 
	}
	else if (_teachingLedActive == false)
	{
		#if !(defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) && !(defined (_USE_DO_TIMER_MODULE_FOR_ESB)) 
//		led_conrol_halSetTeachFeedbackLed(LED_COMMAND_DEACTIVATE);
		#else
		esb_send_led_event(LED_LED1_PIN_NUMBER, LED_COMMAND_DEACTIVATE);
		#endif 
	}
	
	switch(teachStatus)
	{
		case TEACH_STATE_IDLE:
			break;
		
		case TEACH_STATE_START:
		{
			if (nrf_esb_is_idle() == false)
			{
				break;
			}
			led_control_halClrLedActiveBit(LED_BACKLIGHT_PIN_NUMBER);
			delay = SYS_TICK_TIME_BASE_5MS * TEACH_DELAY; 
			timeout = TEACH_TIMEOUT; 
			pipeIndex    = 0;
			channelIndex = RF_MIN_CHANNEL_NUMBER;
			
			errCode = nrf_esb_flush_tx();
			APP_ERROR_CHECK(errCode);
			
			errCode = nrf_esb_flush_rx();
			APP_ERROR_CHECK(errCode);

			errCode = nrf_esb_set_base_address_0(_teachingAddress);
			APP_ERROR_CHECK(errCode);
			
			errCode = nrf_esb_set_base_address_1(_teachingAddress);
			APP_ERROR_CHECK(errCode);
			
			for (uint32_t pipe = 0; pipe  < RF_NUMBER_OF_USED_PIPES; pipe++)
			{
				errCode = nrf_esb_update_prefix(pipe, _teachingPreaddress + pipe);
				APP_ERROR_CHECK(errCode);
			}

			errCode = nrf_esb_enable_pipes(1 << _pipeArray[pipeIndex]);
			APP_ERROR_CHECK(errCode);

			teachStatus = TEACH_STATE_SEND_ID;
		} break;
			
		case TEACH_STATE_SEND_ID:
		{
			_esbFlags &= ~ESB_FLAG_ACK;
			
			errCode = nrf_esb_set_rf_channel(channelIndex);
			APP_ERROR_CHECK(errCode);

			frame.frame.command = RF_CMD_MODULID;
			frame.frame.length = 0;

			if (sendRfFrame(_pipeArray[pipeIndex], &frame, 1, true) == true)
			{
				teachStatus = TEACH_STATE_WAIT_ACK;
			}
		} break;

		case TEACH_STATE_WAIT_ACK:
		{
			if ((_esbFlags & ESB_FLAG_ACK) != 0)
			{
				_esbFlags &= ~ESB_FLAG_ACK;
				
				(void)nrf_esb_stop_rx();
				
				errCode = nrf_esb_flush_tx();
				APP_ERROR_CHECK(errCode);
			
				errCode = nrf_esb_flush_rx();
				APP_ERROR_CHECK(errCode);
				
				// switch to receiving Interface
				errCode = nrf_esb_start_rx();
				APP_ERROR_CHECK(errCode);
				timeout = TEACH_TIMEOUT; 
				teachStatus = TEACH_STATE_WAIT_ID;
				break;
			}
			
			if((_esbFlags & ESB_FLAG_DATA) != 0)
			{
				_esbFlags &= ~ESB_FLAG_DATA;
				
				if (_rxPayload[2] == RF_CMD_MODULID)
				{
					(void)nrf_esb_stop_rx();
					teachStatus   = TEACH_STATE_SEND_ADDRESS;
					_rxPayload[2] = 0xFF;
					break;
				}
			}
			
			if ((_esbFlags & ESB_FLAG_NACK) != 0)
			{
				_esbFlags &= ~ESB_FLAG_NACK;
				
				channelIndex++;
				if (channelIndex > RF_MAX_CHANNEL_NUMBER)
				{
					pipeIndex++;
					if (pipeIndex == 4)
					{
						pipeIndex = 0;
					}
					
					while (nrf_esb_enable_pipes(1 << _pipeArray[pipeIndex]) == NRF_ERROR_BUSY); 
					
					errCode = nrf_esb_enable_pipes(1 << _pipeArray[pipeIndex]);
					if(errCode != NRF_SUCCESS)
					{
						APP_ERROR_CHECK(errCode);
					}
					
					channelIndex = RF_MIN_CHANNEL_NUMBER;
				}
				teachStatus = TEACH_STATE_SEND_ID;
				
				if(timeout == 0)
					esb_stopTeaching();  
			}
		} break;

		case TEACH_STATE_WAIT_ID:
		{
			if ((_esbFlags & ESB_FLAG_DATA) != 0)
			{
				_esbFlags &= ~ESB_FLAG_DATA;
				
				if (_rxPayload[2] == RF_CMD_MODULID)
				{
					_rxPayload[2] = 0xFF;
					
					// switch to transmitting Interface
					errCode = nrf_esb_stop_rx();
					APP_ERROR_CHECK(errCode);

					teachStatus = TEACH_STATE_SEND_ADDRESS;
				}
			}
			
			if(timeout == 0)
				esb_stopTeaching(); 
			
		} break;
			
		case TEACH_STATE_SEND_ADDRESS:
		{
			uint8_t devicePipe;
			
#ifdef FEATURE_TEACH_TWO_DEVICES
			if (_newDeviceNumber == 1)
			{
#endif
				generateNewChannel(&newRfChannel);
				generateNewAddress(&newRfAddress);
				newRfAddress.pre_address = (generateRandomByte() & RF_PREADDRESS_MASK);
				devicePipe = RF_KEYCODE_TRANSMIT_PIPE;
#ifdef FEATURE_TEACH_TWO_DEVICES
			}
			else
			{
				newRfChannel = _rfCommunicationData.channel;
				newRfAddress = _rfCommunicationData.address;
				devicePipe = RF_KEYCODE_ALTERNATE_TRANSMIT_PIPE;
			}
#endif

			frame.frame.command = RF_CMD_CHANGE_ADDR;
			frame.frame.length = 7;
			frame.frame.payload[0] = newRfChannel;
			frame.frame.payload[1] = devicePipe;
			frame.frame.payload[2] = newRfAddress.pre_address;
			frame.frame.payload[3] = newRfAddress.base_address[0];
			frame.frame.payload[4] = newRfAddress.base_address[1];
			frame.frame.payload[5] = newRfAddress.base_address[2];
			frame.frame.payload[6] = newRfAddress.base_address[3];

			_esbFlags &= ~ESB_FLAG_ACK;
			
			if (sendRfFrame(_pipeArray[pipeIndex], &frame, 1, true) == true)
			{
				teachStatus = TEACH_STATE_WAIT_ADDRESS;
			}
		} break;
			
		case TEACH_STATE_WAIT_ADDRESS:
		{
			if ((_esbFlags & ESB_FLAG_ACK) != 0)
			{
				_esbFlags &= ~ESB_FLAG_ACK;
				teachStatus = TEACH_STATE_SEND_CURR_ADDRESS;
				break;
			}
			
			if ((_esbFlags & ESB_FLAG_NACK) != 0)
			{
				_esbFlags &= ~ESB_FLAG_NACK;
				teachStatus = TEACH_STATE_SEND_ADDRESS;
			}
		} break;
			
		case TEACH_STATE_SEND_CURR_ADDRESS:
		{
			frame.frame.command = RF_CMD_SET_CURRENT_ADDR;
			frame.frame.length = 1;
			frame.frame.payload[0] = 1;
		
			_esbFlags &= ~ESB_FLAG_ACK;

			if (sendRfFrame(_pipeArray[pipeIndex], &frame, 1, true) == true)
			{
				teachStatus = TEACH_STATE_WAIT_CURR_ADDRESS;
			}
		} break;
			
		case TEACH_STATE_WAIT_CURR_ADDRESS:
		{
			if ((_esbFlags & ESB_FLAG_ACK) != 0)
			{
				_esbFlags &= ~ESB_FLAG_ACK;
				
#ifdef FEATURE_TEACH_TWO_DEVICES
				if (_newDeviceNumber == 1)
				{
					_rfCommunicationData.numberOfDevices = 1;
				}
				else
				{
					if(_rfCommunicationData.numberOfDevices < RF_MAX_NUMBER_OF_TEACHABLE_DEVICES)
					{
						_rfCommunicationData.numberOfDevices++;
					}
				}
#endif
				_rfCommunicationData.address = newRfAddress;
				_rfCommunicationData.channel = newRfChannel;
				
				// change to new adress
				while(nrf_esb_is_idle() != true){};
				
					// save new Data in Memory	
				#if !(defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) && !(defined (_USE_DO_TIMER_MODULE_FOR_ESB)) 
				saveAddressConfiguration(&_rfCommunicationData);
				#else
				PERSTORE_SET_FLAG_STORE_WHEN_READY_HARD (ESB_SALT1) ;
				#endif 
				
				errCode = nrf_esb_set_base_address_0(newRfAddress.base_address);
				APP_ERROR_CHECK(errCode);
				
				errCode = nrf_esb_set_base_address_1(newRfAddress.base_address);
				APP_ERROR_CHECK(errCode);
				
				for (uint32_t pipe = 0; pipe  < RF_NUMBER_OF_USED_PIPES; pipe++)
				{
					errCode = nrf_esb_update_prefix(pipe, _rfCommunicationData.address.pre_address + pipe);
					APP_ERROR_CHECK(errCode);
				}

#ifdef FEATURE_TEACH_TWO_DEVICES
				errCode = nrf_esb_enable_pipes((1 << RF_KEYCODE_TRANSMIT_PIPE) | (1 << RF_KEYCODE_ALTERNATE_TRANSMIT_PIPE));
				APP_ERROR_CHECK(errCode);
#else
				errCode = nrf_esb_enable_pipes(1 << RF_KEYCODE_TRANSMIT_PIPE);
				APP_ERROR_CHECK(errCode);
#endif
				
				errCode = nrf_esb_set_rf_channel((uint32_t)newRfChannel);
				APP_ERROR_CHECK(errCode);

				teachStatus = TEACH_STATE_DONE;
				break;
			}
			
			if ((_esbFlags & ESB_FLAG_NACK) != 0)
			{
				_esbFlags &= ~ESB_FLAG_NACK;
				teachStatus = TEACH_STATE_SEND_CURR_ADDRESS;
			}
		} break;
			
		case TEACH_STATE_DONE:
		{
			if(delay == SYS_TICK_TIME_BASE_5MS * TEACH_DELAY)
			{
				#if !(defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) && !(defined (_USE_DO_TIMER_MODULE_FOR_ESB)) 
				led_conrol_halSetTeachFeedbackLed(LED_COMMAND_ACTIVATE);
				#else
				esb_send_led_event(LED_LED1_PIN_NUMBER, LED_COMMAND_ACTIVATE);
				#endif 

				io_setFlashBacklight(RF_TEACH_BLINK_INTERVAL, RF_TEACH_BLINK_TIME << 1); 
			}
			else if(delay > 0)
				delay--;
			else
			{
				teachStatus = TEACH_STATE_IDLE;
				_useDefaultAddress = false; 

				_communicationPartnerAwake = true; 
			}
		} break;

		case TEACH_STATE_RESET:
		{
			if(timeout != 0)
			{
			#if !(defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) && !(defined (_USE_DO_TIMER_MODULE_FOR_ESB)) 
				led_conrol_halSetTeachFeedbackLed(LED_COMMAND_DEACTIVATE);
//				nrf_gpio_pin_write(FCT_LED,OFF);
			#else
				esb_send_led_event(LED_LED1_PIN_NUMBER, LED_COMMAND_DEACTIVATE);
			#endif 
			}
			led_control_halSetLedActiveBit(LED_BACKLIGHT_PIN_NUMBER);
			esb_init();
			teachStatus = TEACH_STATE_IDLE;
			timeout = TEACH_TIMEOUT; 
		}
		break;
		
		default:
			break;
	}
	return teachStatus == TEACH_STATE_IDLE ? false : true;
}


/***************************************************************************//**
 *
 * @fn void esb_init(void)
 *
 * @brief  inits the esb module 
 *
 * @param  none 
 * 
 * @return none     
 * 
 ******************************************************************************/ 
void esb_init(void)
{
	// Start clock
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
	
	#ifdef PROJECT_TOKEN_BUS_MODULE_ENABLED
	PERSTORE_REGISTER_DATA_FOR_STORAGE(ESB_SALT1, _rfCommunicationData);
	#endif 
	
	nrf_drv_rng_config_t defaultConfiguration = NRF_DRV_RNG_DEFAULT_CONFIG;

	uint32_t errCode = nrf_drv_rng_init(&defaultConfiguration);
	#ifdef NRF52810_XXAA 
	#ifdef _TEACH_RESET
	if(errCode == NRF_ERROR_MODULE_ALREADY_INITIALIZED)
		hal_systemReset(); 
	#else
	if(errCode != NRF_ERROR_MODULE_ALREADY_INITIALIZED)
	#endif 
	#endif 
		APP_ERROR_CHECK(errCode);
	
	#ifndef PROJECT_TOKEN_BUS_MODULE_ENABLED
	loadAddressConfiguration(&_rfCommunicationData);
	#endif 
	if(_rfCommunicationData.channel > RF_MAX_CHANNEL_NUMBER)
	{
		_rfCommunicationData.channel                 = ESB_DEFAULT_ADDR_CHANNEL;
		_rfCommunicationData.address.base_address[3] = (uint8_t)(ESB_DEFAULT_ADDR >> 24);
		_rfCommunicationData.address.base_address[2] = (uint8_t)(ESB_DEFAULT_ADDR >> 16);
		_rfCommunicationData.address.base_address[1] = (uint8_t)(ESB_DEFAULT_ADDR >> 8);
		_rfCommunicationData.address.base_address[0] = (uint8_t)(ESB_DEFAULT_ADDR >> 0);
		_rfCommunicationData.address.pre_address     = ESB_DEFAULT_ADDR_PREFIX;
		_rfCommunicationData.numberOfDevices         = 1;
		
		_useDefaultAddress = true; 
	}
	else
	{
		_useDefaultAddress = false;
	}
	
	#if !(defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) && !(defined (_USE_DO_TIMER_MODULE_FOR_ESB))
	if (_ledTimerIndex == TIMER_INVALID_TIMER)
	{
		_ledTimerIndex = timer_createTimer();
	}
	
	if (_esbWakeupTimerIndex == TIMER_INVALID_TIMER)
	{
		_esbWakeupTimerIndex = timer_createTimer();
	}
	
	if (_esbContiniousTimerIndex == TIMER_INVALID_TIMER)
	{
		_esbContiniousTimerIndex = timer_createTimer();
	}
	#endif 

	// Initialize ESB
	errCode = nrf_esb_init(&_esbDefaultTxConfiguration);
	APP_ERROR_CHECK(errCode);
	
	errCode = nrf_esb_set_base_address_0(_rfCommunicationData.address.base_address);
	APP_ERROR_CHECK(errCode);
	
	errCode = nrf_esb_set_base_address_1(_rfCommunicationData.address.base_address);
	APP_ERROR_CHECK(errCode);
	
	for (uint32_t pipe = 0; pipe  < RF_NUMBER_OF_USED_PIPES; pipe++)
	{
		errCode = nrf_esb_update_prefix(pipe, _rfCommunicationData.address.pre_address + pipe);
		APP_ERROR_CHECK(errCode);
	}

	// check if NumberOfDevices is correct
#ifdef FEATURE_TEACH_TWO_DEVICES
	if (_rfCommunicationData.numberOfDevices > RF_MAX_NUMBER_OF_TEACHABLE_DEVICES)
	{
		_rfCommunicationData.numberOfDevices = RF_MAX_NUMBER_OF_TEACHABLE_DEVICES;
	}
	
	errCode = nrf_esb_enable_pipes((1 << RF_KEYCODE_TRANSMIT_PIPE) | (1 << RF_KEYCODE_ALTERNATE_TRANSMIT_PIPE));
	APP_ERROR_CHECK(errCode);
#else
	errCode = nrf_esb_enable_pipes(1 << RF_KEYCODE_TRANSMIT_PIPE);
	APP_ERROR_CHECK(errCode);
#endif
	
	errCode = nrf_esb_set_rf_channel((uint32_t)_rfCommunicationData.channel);
	APP_ERROR_CHECK(errCode);

	errCode = nrf_esb_flush_tx();
	APP_ERROR_CHECK(errCode);
	
	errCode = nrf_esb_flush_rx();
	APP_ERROR_CHECK(errCode);
}


/***************************************************************************//**
 *
 * @fn void esb_init(void)
 *
 * @brief  deinits the esb module 
 *
 * @param  none 
 * 
 * @return none     
 * 			
 ******************************************************************************/ 
void esb_deinit(void)
{
	nrf_esb_disable();
}


/***************************************************************************//**
 *
 * @fn bool esb_transmitKeycode(uint32_t keycode)
 *
 * @brief  transmit a keycode  
 *
 * @param  uint32_t keycode -> current keycode  
 * 
 * @return true when done 
 * 
 ******************************************************************************/ 
bool esb_transmitKeycode(uint32_t keycode)
{
	
	if(keycode == 0x0020000)
	{
//		__nop(); 
	}
	
	bool tranmissionPossible = false;
	rf_frame_t frame;
	frame.frame.command = RF_CMD_BUTTON;
	frame.frame.length = 4;
	frame.frame.payload[0] = ESB_GET_BYTE(keycode, 3);
	frame.frame.payload[1] = ESB_GET_BYTE(keycode, 2);
	frame.frame.payload[2] = ESB_GET_BYTE(keycode, 1);
	frame.frame.payload[3] = ESB_GET_BYTE(keycode, 0);
	if ((_communicationPartnerAwake == true)
		|| (_useDefaultAddress == true))
	{
		_keycodeTransmissionPending = false;
#ifdef FEATURE_TEACH_TWO_DEVICES
		if ((_rfCommunicationData.numberOfDevices == 1)
		||  (_useDefaultAddress == true))
		{
#endif
			tranmissionPossible = sendRfFrame(RF_KEYCODE_TRANSMIT_PIPE, &frame, 1, true);
#ifdef FEATURE_TEACH_TWO_DEVICES
		}
		else
		{
			_ledMemoryActive = false;
			tranmissionPossible = sendRfFrame(RF_KEYCODE_TRANSMIT_PIPE, &frame, 1, true);
			tranmissionPossible = sendRfFrame(RF_KEYCODE_ALTERNATE_TRANSMIT_PIPE, &frame, 1, false);
		}
#endif
	}
	else
	{
		_keycodeTransmissionPending = true;
	}
	_esbWakeupTimerIndex = RF_PARTNER_AWAKE_TIEMOUT;

	return tranmissionPossible;
}


/***************************************************************************//**
 *
 * @fn void esb_wakeUpControls(void)
 *
 * @brief  esb wake up message / handling 
 *
 * @param  none 
 * 
 * @return none 
 * 
 ******************************************************************************/ 
void esb_wakeUpControls(void)
{
	
	rf_frame_t frame;
	frame.frame.command = RF_CMD_BUTTON;
	frame.frame.length = 4;
	frame.frame.payload[0] = 0;
	frame.frame.payload[1] = 0;
	frame.frame.payload[2] = 0;
	frame.frame.payload[3] = 0;

	// TIMEOUT
	if (_esbWakeupTimerIndex == 0)
	{
		_communicationPartnerAwake = false;
		_keycodeTransmissionPending = false;
		_wakeupSuccessCounter = 0;
	}
	else if ((_communicationPartnerAwake == false) &&
		        (_keycodeTransmissionPending == true))
	{
		// No Errors while transmitting,
#ifndef FEATURE_TEACH_TWO_DEVICES
		if (_wakeupSuccessCounter == 1)
#else
		if (_wakeupSuccessCounter == _rfCommunicationData.numberOfDevices)
#endif
		{
			_communicationPartnerAwake = true;
		}
		else
		{
#ifdef FEATURE_TEACH_TWO_DEVICES
			if (_rfCommunicationData.numberOfDevices == 1)
			{
#endif
				sendRfFrame(RF_KEYCODE_TRANSMIT_PIPE, &frame, 1, true);
#ifdef FEATURE_TEACH_TWO_DEVICES
			}
			else
			{
				sendRfFrame(RF_KEYCODE_TRANSMIT_PIPE, &frame, 1, true);
				sendRfFrame(RF_KEYCODE_ALTERNATE_TRANSMIT_PIPE, &frame, 1, false);
			}
			_wakeupSuccessCounter = 0;
#endif
		}
	}
	else
	{
		// do nothing, all devices are awake
	}
}


/***************************************************************************//**
 *
 * @fn bool esb_checkContiniousMode(void)
 *
 * @brief  esb check continuos mode 
 *
 * @param  none 
 * 
 * @return true when done  
 * 
 ******************************************************************************/ 
bool esb_checkContiniousMode(void)
{
	bool keepAwake = false;
	rf_frame_t frame;
	frame.frame.command = RF_CMD_BUTTON;
	frame.frame.length = 4;
	frame.frame.payload[0] = 0;
	frame.frame.payload[1] = 0;
	frame.frame.payload[2] = 0;
	frame.frame.payload[3] = 0;

	if ((_ledMemoryActive == true) && ( _esbContiniousTimerIndex != 0)) // (timer_timeoutReached(_esbContiniousTimerIndex) == false))
	{
#ifdef FEATURE_TEACH_TWO_DEVICES
		if (_rfCommunicationData.numberOfDevices != 0)
		{
#endif
			// even with two Paired Controls, just talk to control one for continious mode
			sendRfFrame(RF_KEYCODE_TRANSMIT_PIPE, &frame, 1, true);
#ifdef FEATURE_TEACH_TWO_DEVICES
		}
#endif
		keepAwake = true;
	}
	// timeout, but ControlUnit has not reset LED control
	else if((_ledMemoryActive == true) && ( _esbContiniousTimerIndex == 0))//(timer_timeoutReached(_esbContiniousTimerIndex) == true))
	{
		_ledMemoryActive = false;
	}
	return keepAwake;
}


/***************************************************************************//**
 *
 * @fn void esbEventHandler(nrf_esb_evt_t const * event)
 *
 * @brief  esb event handler 
 *
 * @param  nrf_esb_evt_t const * event -> event pointer 
 * 
 * @return none   
 * 
 ******************************************************************************/ 
static void esbEventHandler(nrf_esb_evt_t const * event)
{
	switch(event->evt_id)
	{
		/**< Event triggered on TX success. */
		case NRF_ESB_EVENT_TX_SUCCESS:
		{
			_esbFlags |= ESB_FLAG_ACK;
		
			if (_keycodeTransmissionPending == true)
			{
				_wakeupSuccessCounter++;
			}
			
			if (_teachingLedActive == false)
			{
				#if !(defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) && !(defined (_USE_DO_TIMER_MODULE_FOR_ESB)) 
//				led_conrol_halSetRfFeedbackLed(LED_COMMAND_ACTIVATE);
				#else
				esb_send_led_event(LED_LED1_PIN_NUMBER, LED_COMMAND_ACTIVATE);
				#endif 
			}
		} break;
		
		/**< Event triggered on TX failure. */
		case NRF_ESB_EVENT_TX_FAILED:
		{
			_esbFlags |= ESB_FLAG_NACK;
		} break;
		
		/**< Event triggered on RX received. */
		case NRF_ESB_EVENT_RX_RECEIVED:
		{
			nrf_esb_payload_t rxPayload = {0};
			
			if (nrf_esb_read_rx_payload(&rxPayload) != NRF_SUCCESS)
			{
				break;
			}
			
			if (rxPayload.length == 0)
			{
				_esbFlags |= ESB_FLAG_ACK;
				break;
			}
			
			_esbFlags |= ESB_FLAG_DATA;
			
			for (uint8_t index = 0; index < NRF_ESB_MAX_PAYLOAD_LENGTH; index++)
			{
				_rxPayload[index] = rxPayload.data[index];
			}
			
			if (_rxPayload[2] == RF_CMD_LED_STATUS)
			{
				uint32_t ledStatusBuffer = 0;
				uint32_t ledMaskBuffer = 0;
				
				for (uint8_t index = 0; index < RF_SIZE_OF_LED_STATE; ++index)
				{
					ledStatusBuffer |= _rxPayload[index + RF_ESB_HEADER_LENGTH] << (index * LENGHT_OF_BYTE);
					ledMaskBuffer   |= _rxPayload[index + RF_ESB_HEADER_LENGTH + RF_SIZE_OF_LED_STATE] << (index * LENGHT_OF_BYTE);
				}
			
				if ((ledMaskBuffer & ledStatusBuffer & RF_CONTINIOUS_MODE) != 0)
				{
					_esbContiniousTimerIndex = RF_CONTINIOUS_TIMEOUT; 
				}

				if ((ledMaskBuffer & ledStatusBuffer & RF_SET_LED_ON_MODE) != 0)
				{
					_ledMemoryActive = true;
				}
				else
				{
				_esbContiniousTimerIndex = TIMER_INVALID_TIMER; 
					_ledMemoryActive = false;
				}
			}
			else
			{
				static uint8_t toggle = 0;
				
				if (_rxPayload[0] < 4)
				{
					break;
				}
				
				uint32_t ledCode = 0;
				ledCode = _rxPayload[6];
				ledCode <<= 8;
				ledCode |= _rxPayload[5];
				ledCode <<= 8;
				ledCode |= _rxPayload[4];
				ledCode <<= 8;
				ledCode |= _rxPayload[3];
				
				if (_rxPayload[2] == RF_CMD_LED_ON)
				{
					if ((ledCode & RF_CONTINIOUS_MODE) != 0)
					{
						_esbContiniousTimerIndex = RF_CONTINIOUS_TIMEOUT; 
					}
					
					if((ledCode & RF_SET_LED_ON_MODE) != 0)
					{
						_ledMemoryActive = true;
					}
				}
				else if(_rxPayload[2] == RF_CMD_LED_OFF)
				{
					if ((ledCode & RF_CONTINIOUS_MODE) != 0)
					{
						_ledMemoryActive = false;
						_esbContiniousTimerIndex = TIMER_INVALID_TIMER; 
					}
				
					if ((ledCode & RF_SET_LED_ON_MODE) != 0)
					{
						_ledMemoryActive = false;
						_esbContiniousTimerIndex = TIMER_INVALID_TIMER; 
					}
				}
				else if (_rxPayload[2] == RF_CMD_LED_TOGGLE)
				{
					if (toggle == 0)
					{
						_esbContiniousTimerIndex = RF_CONTINIOUS_TIMEOUT; 
						toggle = 1;
					}
					else
					{
						_esbContiniousTimerIndex = TIMER_INVALID_TIMER; 
						toggle = 0;
					}
				}
				else if (_rxPayload[2] == RF_CMD_LED_AUTO)
				{
					_esbContiniousTimerIndex = TIMER_INVALID_TIMER; 
					
					if (_rxPayload[0] != 6)
					{
						break;
					}
					
					if (ledCode == 0)
					{
						io_setFlashBacklight(_rxPayload[7], _rxPayload[8] << 1); 
					}
				}
			}
		} break;
		
		default:
			break;
	}
}

bool esb_getTeachingLedState (void)
{
	return _teachingLedActive; 
}

