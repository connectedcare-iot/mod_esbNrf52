/**
******************************************************************************
* @file    io_driver.c
* @author  Nils Grosse-Goeddinghaus / Jens Wörmann
* @version V1.0
* @date    05.01.20
* @brief   this module handles the io functioalty. 
******************************************************************************
* @attention
*
*
*
******************************************************************************
*/



#include <string.h>

#include "nrf_gpio.h"

#include "gpio_rf56_hal.h"
#include "hardware_information.h"
#include "event_dispatcher.h" 
#include "key_event.h"
#include "led_event.h"
#include "do_timer.h" 

#include "io_driver.h"
#include "esb_timer_driver.h"
#include "esb_hsTransmitter.h"

#include "hardware_information.h"
#include "keycodes.h"

#include "do_gpio.h"

#include "led_control_api.h"

#include "keys_hal_app.h"

#include "ble_lib_main.h"

#include "esb_cntrlReceiver.h"
#include "esb_eventHandler.h"

static void ioServiceTimer (void); 



#define ESB_TIMER_SPEED 1

#ifdef _USE_DO_TIMER_MODULE_FOR_ESB
#define IO_STOP_RESEND_TIMEOUT             (100/IO_DRIVER_TIMER) ///< @brief IO Timeout
#define IO_BLINK_RESEND_TIMEOUT            (100/IO_DRIVER_TIMER) ///< @brief IO Timeout

DO_TIMER_REGISTER_HANDLER(ioServiceTimer, IO_DRIVER_TIMER);
#endif 

static uint16_t _stopTimerIndex = TIMER_INVALID_TIMER;  ///< @brief the stop timer index
static uint16_t _blinkTimerIndex = TIMER_INVALID_TIMER; ///< @brief the blink timer index 


static uint32_t _currentKeycode; 

// prototypes for static calls 
static uint32_t _getButtonValue(void);
static void _activateButtonBacklight(uint32_t buttonValueMask);
static io_feature_t _checkIoFeatureTeaching(uint32_t buttonValueMask, uint8_t* teachingDeviceNumber);
static io_feature_t _checkIoFeatureFlashlight(uint32_t buttonValueMask);
static io_feature_t _checkIoFeatureKey(uint32_t buttonValueMask, uint32_t buttonToggleMask);
static io_feature_t _checkIoFeatureMemory(uint32_t buttonValueMask); 

static void keysEventHandler(const logical_id_t source,
                             const eventType_t  type,
                             const void * const eventDataPtr,
                             const uint_fast8_t eventDataLength); 

typedef struct 
{
	bool         startFlashBackligth;     ///< @brief start blink procedere
	uint8_t      flashBacklightCnt;       ///< @brief blinklight counter
	uint8_t      flashBacklightInterval;  ///< @brief the interval of blinking 
	uint8_t      flashBacklightTimeout;   ///< @brief the time of blinking in s 
}__attribute__ ((packed))  io_backlight_t; 

io_backlight_t _backlightSignal; 

// extern volatile uint32_t sleep_timer;

EVENT_DISPATCHER_REGISTER_HANDLER(keysEventHandler, EVENT_TYPE_KEYS);

static void keysEventHandler(const logical_id_t source,
                             const eventType_t  type,
                             const void * const eventDataPtr,
                             const uint_fast8_t eventDataLength)
{
	if ((eventDataPtr == NULL) ||
	    (eventDataLength == 0))
	{
		return;
	}

	keys_keycode_event_t * pEvt = (keys_keycode_event_t *)eventDataPtr;
	if (pEvt->command == KEYS_CMD_KEYCODE)
	{
		_currentKeycode = pEvt->keycode; 
		
		if (pEvt->keycode == 0)
		{
			
		}
		else
		{
		
		}
	}
}

static void ioServiceTimer (void)
{
	if((_stopTimerIndex < TIMER_INVALID_TIMER) && (_stopTimerIndex > 0))  ///< @brief the stop timer index
		_stopTimerIndex--; 
	if((_blinkTimerIndex < TIMER_INVALID_TIMER) && (_blinkTimerIndex > 0))  ///< @brief the stop timer index
		_blinkTimerIndex--;

}

static void io_send_led_event (uint8_t number, uint8_t state)
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

#ifdef _USE_DO_TIMER_MODULE_FOR_ESB
void ioCheckSleep(void)
{
		esb_deinit();
	
		#warning "Turn of power transistor here" 
		// Enter system OFF. After wakeup the chip will be reset, and code execution will run from the top  
		NRF_POWER->SYSTEMOFF = 1; 
}
#endif 

/***************************************************************************//**
 *
 * @fn void io_init(void)
 *
 * @brief  inits the io module 
 *
 * @param  none
 * 
 * @return none   
 * 			
 ******************************************************************************/  

void io_init(void)
{
	gpio_setPinFunction (LATCH_PIN_NR, GPIO_PIN_FUNCTION_OUT_PULL_DOWN); 
	gpio_setOutputLevel (LATCH_PIN_NR,GPIO_PIN_LEVEL_HIGH);  

	gpio_setPinFunction (LED_FLASHLIGHT_PIN_NUMBER, GPIO_PIN_FUNCTION_OUTPUT_PUSH_PULL); 
	gpio_setOutputLevel (LED_FLASHLIGHT_PIN_NUMBER,GPIO_PIN_LEVEL_LOW); 
	
	gpio_setPinFunction (LED_LED1_PIN_NUMBER, GPIO_PIN_FUNCTION_OUTPUT_PUSH_PULL); 
	gpio_setOutputLevel (LED_LED1_PIN_NUMBER,GPIO_PIN_LEVEL_HIGH); 
	
	gpio_setPinFunction (LED_LED2_PIN_NUMBER, GPIO_PIN_FUNCTION_OUTPUT_PUSH_PULL); 
	gpio_setOutputLevel (LED_LED2_PIN_NUMBER,GPIO_PIN_LEVEL_HIGH); 
	
	gpio_setPinFunction (LED_LED3_PIN_NUMBER, GPIO_PIN_FUNCTION_OUTPUT_PUSH_PULL); 
	gpio_setOutputLevel (LED_LED3_PIN_NUMBER,GPIO_PIN_LEVEL_HIGH);           

 	gpio_setPinFunction (LED_BACKLIGHT_PIN_NUMBER, GPIO_PIN_FUNCTION_OUTPUT_PUSH_PULL); 
	gpio_setOutputLevel (LED_BACKLIGHT_PIN_NUMBER,GPIO_PIN_LEVEL_HIGH);  
}



/***************************************************************************//**
 *
 * @fn void io_deinit(void)
 *
 * @brief  deinits the io module 
 *
 * @param  none
 * 
 * @return none   
 * 			
 ******************************************************************************/  

void io_deinit(void)
{
	gpio_setOutputLevel (LATCH_PIN_NR,GPIO_PIN_LEVEL_LOW);  
	gpio_setOutputLevel (LED_FLASHLIGHT_PIN_NUMBER,GPIO_PIN_LEVEL_LOW); 
	gpio_setOutputLevel (LED_LED1_PIN_NUMBER,GPIO_PIN_LEVEL_HIGH); 
	gpio_setOutputLevel (LED_LED2_PIN_NUMBER,GPIO_PIN_LEVEL_HIGH); 
	gpio_setOutputLevel (LED_LED3_PIN_NUMBER,GPIO_PIN_LEVEL_HIGH); 
	gpio_setOutputLevel (LED_BACKLIGHT_PIN_NUMBER,GPIO_PIN_LEVEL_LOW);
	gpio_setOutputLevel (LATCH_PIN_NR,GPIO_PIN_LEVEL_LOW);  
}


/***************************************************************************//**
 *
 * @fn static uint32_t _getButtonValue(void)
 *
 * @brief  deinits the io module 
 *
 * @param  none
 * 
 * @return vale - the button value (uint32_t)   
 * 			
 ******************************************************************************/  

static uint32_t _getButtonValue(void)
{
	return _currentKeycode;
}

/***************************************************************************//**
 *
 * @fn static void _activateButtonBacklight(uint32_t buttonValueMask)
 *
 * @brief  activate the button backlight 
 *
 * @param  uint32_t buttonValueMask -> the button value mask 
 * 
 * @return none 
 * 
 ******************************************************************************/  

#define TIMER_TEACH_CONTROL 40 

static void _activateButtonBacklight(uint32_t buttonValueMask)
{
	static uint8_t timer = TIMER_TEACH_CONTROL; 
	if(getTeachReceiverState())
	{
		timer--; 
		led_control_halClrLedActiveBit(LED_BACKLIGHT_PIN_NUMBER);
		if(timer == 0)
		{
			timer = TIMER_TEACH_CONTROL; 
			led_control_halToogleLedActiveBit(LED_LED1_PIN_NUMBER);
			led_control_halToogleLedActiveBit(LED_LED2_PIN_NUMBER);
			led_control_halToogleLedActiveBit(LED_LED3_PIN_NUMBER);
		}
	}
	else if(esb_getTeachingLedState())
	{
		led_control_halSetLedActiveBit(LED_LED1_PIN_NUMBER);
		led_control_halSetLedActiveBit(LED_LED2_PIN_NUMBER);
		led_control_halSetLedActiveBit(LED_LED3_PIN_NUMBER);
		led_control_halSetLedActiveBit(LED_BACKLIGHT_PIN_NUMBER); 
	}
	else
	{
		led_control_halClrLedActiveBit(LED_LED1_PIN_NUMBER);
		led_control_halClrLedActiveBit(LED_LED2_PIN_NUMBER);
		led_control_halClrLedActiveBit(LED_LED3_PIN_NUMBER);
		
		
		if((buttonValueMask != 0)
		&& (_backlightSignal.startFlashBackligth == false))
		{
			#if !(defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) && !(defined (_USE_DO_TIMER_MODULE_FOR_ESB)) 
			led_control_halSetLedActiveBit(LED_BACKLIGHT_PIN_NUMBER); 
			#else
			io_send_led_event(LED_BACKLIGHT_PIN_NUMBER, LED_COMMAND_ACTIVATE); 
			#endif 
		}
	}
}


/***************************************************************************//**
 *
 * @fn static io_feature_t _checkIoFeatureTeaching(uint32_t buttonValueMask, uint8_t* teachingDeviceNumber)
 *
 * @brief  results feautre when io teachmode is active 
 *
 * @param  uint32_t buttonValueMask -> the button value mask 
 * @param  uint8_t* teachingDeviceNumber -> pointer to the teaching device number 
 * 
 * @return io_feature_t when teachmode should be activated 
 * 
 ******************************************************************************/  

static io_feature_t _checkIoFeatureTeaching(uint32_t buttonValueMask, uint8_t* teachingDeviceNumber)
{
	io_feature_t resultingFeature = IO_FEATURE_NOTHING;
	uint8_t teachingIndexToCheck = 0;

uint32_t teachingKeycodeArray[] = {
		BUTTON_TEACH_MASK_NR1,
//#ifdef FEATURE_TEACH_TWO_DEVICES
		BUTTON_TEACH_MASK_NR2 
//#endif
	};
	
	for (teachingIndexToCheck = 0; teachingIndexToCheck < (sizeof(teachingKeycodeArray) / sizeof(teachingKeycodeArray[0])); ++teachingIndexToCheck)
	{
		if((buttonValueMask & teachingKeycodeArray[teachingIndexToCheck]) == teachingKeycodeArray[teachingIndexToCheck])
		{
			resultingFeature = IO_FEATURE_TEACHING;
			*teachingDeviceNumber = teachingIndexToCheck + 1;
			break;
		}
	}
	return resultingFeature;
}

/***************************************************************************//**
 *
 * @fn static io_feature_t _checkIoFeatureMemory(uint32_t buttonValueMask)
 *
 * @brief  results feautre when io memory save is active 
 *
 * @param  uint32_t buttonValueMask -> the button value mask 
 * 
 * @return io_feature_t IO_FEATURE_FLASHLIGHT when flashligt is active 
 * 
 ******************************************************************************/  

static io_feature_t _checkIoFeatureMemory(uint32_t buttonValueMask)
{
	io_feature_t resultingFeature = IO_FEATURE_NOTHING;
	if((buttonValueMask & BUTTON_MEMORY_MASK) == BUTTON_MEMORY_MASK)
	{
		resultingFeature = IO_FEATRUE_MEMORY_SAVE;
	}
	return resultingFeature;
}


/***************************************************************************//**
 *
 * @fn static io_feature_t _checkIoFeatureFlashlight(uint32_t buttonValueMask)
 *
 * @brief  results feautre when io flash light is active 
 *
 * @param  uint32_t buttonValueMask -> the button value mask 
 * 
 * @return io_feature_t IO_FEATURE_FLASHLIGHT when flashligt is active 
 * 
 ******************************************************************************/  
static io_feature_t _checkIoFeatureFlashlight(uint32_t buttonValueMask)
{
	io_feature_t resultingFeature = IO_FEATURE_NOTHING;
	if((buttonValueMask & BUTTON_TORCH_MASK) == BUTTON_TORCH_MASK)
	{
		resultingFeature = IO_FEATURE_FLASHLIGHT;
	}
	return resultingFeature;
}

#ifdef  _USE_TWO_KEYS_FOR_UBB
static io_feature_t _checkIoFeatureUBB(uint32_t buttonValueMask)
{
	io_feature_t resultingFeature = IO_FEATURE_NOTHING;
	if((buttonValueMask & BUTTOM_UBB_MASK) == BUTTOM_UBB_MASK)
	{
		resultingFeature = IO_FEATURE_UBB;
	}
	return resultingFeature;
}
#endif 

#ifdef _RESET_DRIVE
static io_feature_t _checkIoFeatureResetDrive(uint32_t buttonValueMask)
{
	io_feature_t resultingFeature = IO_FEATURE_NOTHING;
	if((buttonValueMask & BUTTOM_REF_DRIVE_MASK) == BUTTOM_REF_DRIVE_MASK)
	{
		resultingFeature = IO_FEATURE_REF_DRIVE;
	}
	return resultingFeature;
}
#endif

/***************************************************************************//**
 *
 * @fn static io_feature_t _checkIoFeatureFlashlight(uint32_t buttonValueMask)
 *
 * @brief  results feautre when io flash light is active 
 *
 * @param  uint32_t buttonValueMask -> the button value mask 
 * 
 * @return io_feature_t IO_FEATURE_FLASHLIGHT when flashligt is active 
 * 
 ******************************************************************************/  

static io_feature_t _checkIoFeatureKey(uint32_t buttonValueMask, uint32_t buttonToggleMask)
{
	io_feature_t resultingFeature = IO_FEATURE_NOTHING;
	// at least one new Button was pressed or released
	if((buttonToggleMask == 0) && (buttonValueMask != 0))
	{
		resultingFeature = IO_FEATURE_KEY_PRESSED;
	}
	else if((buttonToggleMask != 0) && (buttonValueMask != 0))
	{
		resultingFeature = IO_FEATURE_KEY_CHANGED;
	}
	// all Buttons were released
	else if ((buttonToggleMask != 0) && (buttonValueMask == 0))
	{
		resultingFeature = IO_FEATURE_KEY_RELEASED;
	}
	// no Button is pressed and that has been the case for at least one cycle
	else
	{
		// do Nothing
	}
	return resultingFeature;
}

/***************************************************************************//**
 *
 * @fn bool io_checkButtons()
 *
 * @brief  check buttons and start features 
 *
 * @param  none
 * 
 * @return true if feature is active - otherwise false 
 * 
 ******************************************************************************/ 

static uint8_t sendUbbKeycode = 10; 

bool io_checkButtons(void)
{
	bool buttonWasPressed = false;
	static uint32_t activeKeycode;


	static uint32_t oldButtonValueMask = 0;
//	uint32_t buttonValueMask = _getButtonValue();
//	uint32_t buttonValueMask = gpio_rf56_hal_get_buttonKeys(); 
	
	uint32_t buttonValueMask = keys_debounceKeys(); 
	uint32_t buttonToggleMask = buttonValueMask ^ oldButtonValueMask;

	io_feature_t activeFeature = IO_FEATURE_NOTHING;
	uint8_t teachingDeviceNumber = 0;
	uint32_t activeFeatureMask = 0;
	static uint32_t lastFeatureMask = 0;

	_activateButtonBacklight(buttonValueMask);
	
	activeFeature = _checkIoFeatureTeaching(buttonValueMask, &teachingDeviceNumber);
	activeFeatureMask |= IO_GENERATE_FEATURE_MASK(activeFeature);
	
	activeFeature = _checkIoFeatureFlashlight(buttonValueMask);
	activeFeatureMask |= IO_GENERATE_FEATURE_MASK(activeFeature);
	
	#ifdef  _USE_TWO_KEYS_FOR_UBB
	activeFeature = _checkIoFeatureUBB(buttonValueMask);
	activeFeatureMask |= IO_GENERATE_FEATURE_MASK(activeFeature);
	#endif 
	
	activeFeature = _checkIoFeatureKey(buttonValueMask, buttonToggleMask);
	activeFeatureMask |= IO_GENERATE_FEATURE_MASK(activeFeature);
	
	activeFeature = _checkIoFeatureMemory(buttonValueMask);
	activeFeatureMask |= IO_GENERATE_FEATURE_MASK(activeFeature);
	
	#ifdef _RESET_DRIVE
	activeFeature = _checkIoFeatureResetDrive(buttonValueMask);
	activeFeatureMask |= IO_GENERATE_FEATURE_MASK(activeFeature);
	#endif 

	if(_blinkTimerIndex == 0)
	{
		_blinkTimerIndex = IO_BLINK_RESEND_TIMEOUT;
		io_backlightSignal(); 
	}
	#ifdef  _USE_TWO_KEYS_FOR_UBB
	if ((IO_IS_FEATURE_ACTIVE(activeFeatureMask, IO_FEATURE_TEACHING) == false)
	#ifdef _RESET_DRIVE
	&& (IO_IS_FEATURE_ACTIVE(activeFeatureMask, IO_FEATURE_REF_DRIVE) == false)
	#endif
	&& (IO_IS_FEATURE_ACTIVE(activeFeatureMask, IO_FEATURE_UBB) == false))
	#else
	if (IO_IS_FEATURE_ACTIVE(activeFeatureMask, IO_FEATURE_TEACHING) == false)
	#endif 
	{
		if (IO_IS_FEATURE_ACTIVE(activeFeatureMask, IO_FEATURE_KEY_CHANGED) == true)
		{
			uint8_t buttonIndex = 0;
			// generate new keycode
			activeKeycode = KEY_VALUE_STOP;

			
//			for(buttonIndex = 0; buttonIndex < (sizeof(key_code_table) / sizeof(key_code_table[0])); buttonIndex++)
			for(buttonIndex = 0; buttonIndex < key_getKeyCodeSize(); buttonIndex++) 
			{
				if((buttonValueMask & (1 << buttonIndex)) != 0)
				{
					activeKeycode |= key_getKeyCode(buttonIndex); 
					// activeKeycode |= key_code_table[buttonIndex];
				}
			}
			#ifdef  BUTTOM_UBB_MASK
			if(oldButtonValueMask != BUTTOM_UBB_MASK)
			{
				esb_transmitKeycode(activeKeycode);
			}
			#endif
		}
		else if (IO_IS_FEATURE_ACTIVE(activeFeatureMask, IO_FEATURE_KEY_RELEASED) == true)
		{
			_stopTimerIndex = IO_STOP_RESEND_TIMEOUT;
			activeKeycode = KEY_VALUE_STOP;
		}
		else if ((IO_IS_FEATURE_ACTIVE(activeFeatureMask, IO_FEATURE_KEY_PRESSED) == true) ||
				 // check if stop commands still have to be send after release
				 (_stopTimerIndex > 0))
		{
			esb_transmitKeycode(activeKeycode);
		}
		else
		{
			// nothing to do
		}
	}
	
	#ifdef  _USE_TWO_KEYS_FOR_UBB
	if(IO_WAS_FEATURE_ACTIVATED(activeFeatureMask, lastFeatureMask, IO_FEATURE_UBB) == true)
	{
		// esb_transmitKeycode(key_getUbbKeycode()); 
		activeKeycode = key_getUbbKeycode(); 
		sendUbbKeycode = KEY_UBB_RELEASE_TIME; 
	}
	
	if(sendUbbKeycode > 0)
	{
		sendUbbKeycode--;
	}
	
	if(IO_IS_FEATURE_ACTIVE(activeFeatureMask, IO_FEATURE_UBB) == true)
	{
		if(sendUbbKeycode > 0)
		{
			esb_transmitKeycode(key_getUbbKeycode()); 
		}
	}
	#endif 
	
	// check if teaching was activated
	if (IO_WAS_FEATURE_ACTIVATED(activeFeatureMask, lastFeatureMask, IO_FEATURE_TEACHING) == true)
	{
		// TODO: Implement check for teaching Index
		// esb_manageRfInterface(1);
		if(teachingDeviceNumber == 1) 
		{
			esb_teachNewDevice(teachingDeviceNumber);
		}
		else if(teachingDeviceNumber == 2) 
		{
			if(oldButtonValueMask != buttonValueMask)
			{
				// Restart teach mode
				esb_deinit_transiver(); 
				uint32_t errCode; 
				
				esb_resetEsb_receiver(); 
				
				errCode = esb_init_receiver(true, 0, false);
//				APP_ERROR_CHECK(errCode);
				
				errCode = bleLib_restartTeachMode(true);
				APP_ERROR_CHECK(errCode);
			}
		}
		
		led_control_halClrLedActiveBit(LED_BACKLIGHT_PIN_NUMBER);
	}
	// check if teaching was deactivated
	else if (IO_WAS_FEATURE_DEACTIVATED(activeFeatureMask, lastFeatureMask, IO_FEATURE_TEACHING) == true)
	{
		// esb_manageRfInterface(255);
		esb_stopTeaching();
		// reset Keycode
		
		#ifdef  _USE_TWO_KEYS_FOR_UBB
		if(sendUbbKeycode > 0)
		{
			esb_transmitKeycode(key_getUbbKeycode()); 
		}
		activeKeycode = KEY_VALUE_STOP;
		
		#endif 
	}

	
	// check if flashlight was activated
	if (IO_WAS_FEATURE_ACTIVATED(activeFeatureMask, lastFeatureMask, IO_FEATURE_FLASHLIGHT) == true)
	{
		#if !(defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) && !(defined (_USE_DO_TIMER_MODULE_FOR_ESB)) 
		led_control_halSetLedActiveBit(LED_TORCH_PIN_NUM); 
		#else
		io_send_led_event(LED_TORCH_PIN_NUM, LED_COMMAND_ACTIVATE); 
		#endif 
	}
	// check if flashlight was deactivated
	else if (IO_WAS_FEATURE_DEACTIVATED(activeFeatureMask, lastFeatureMask, IO_FEATURE_FLASHLIGHT) == true)
	{
		#if !(defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) && !(defined (_USE_DO_TIMER_MODULE_FOR_ESB)) 
		led_control_halClrLedActiveBit(LED_TORCH_PIN_NUM); 
		#else
		io_send_led_event(LED_TORCH_PIN_NUM, LED_COMMAND_DEACTIVATE); 
		#endif 
	}
	
	if(IO_WAS_FEATURE_ACTIVATED(activeFeatureMask, lastFeatureMask, IO_FEATRUE_MEMORY_SAVE) == true)
	{

	}
	
	#ifdef _RESET_DRIVE
	if(IO_IS_FEATURE_ACTIVE(activeFeatureMask, IO_FEATURE_REF_DRIVE) == true)
	{
		esb_transmitKeycode(key_getRefDriveMask()); 
	}
	#endif 

	lastFeatureMask = activeFeatureMask;
	if (activeFeatureMask != 0) // || (sendUbbKeycode > 0))
	{
		buttonWasPressed = true;
	}
	
	oldButtonValueMask = buttonValueMask;
	return buttonWasPressed;
}


bool io_getWaitForUbbFlag (void)
{
	if(sendUbbKeycode > 0)
		return false;
	else
		return true; 
}

/***************************************************************************//**
 *
 * @fn void io_setFlashBacklight (uint8_t interval, uint8_t count)
 *
 * @brief  set the blink function; start and interval; 
 *
 * @param  uint8_t interval -> the blink intervall
 * @param  uint8_t count -> the count >> 1 
 * 
 * @return none   
 * 			
 ******************************************************************************/  

void io_setFlashBacklight (uint8_t interval, uint8_t count)
{
	if(key_gehtBlinkFlag())
	{
		_backlightSignal.startFlashBackligth = true; 
		_backlightSignal.flashBacklightInterval = interval;
		_backlightSignal.flashBacklightCnt = count; 
	}
}

/***************************************************************************//**
 *
 * @fn static void io_watchButtonLight (void)
 *
 * @brief  will be count dow the timeout. 
 *
 * @param  none
 * 
 * @return none   
 * 			
 ******************************************************************************/


static void io_watchButtonLight (void)
{
	
	if (_backlightSignal.flashBacklightTimeout != 0)
	{
		_backlightSignal.flashBacklightTimeout--;
	}
	
	if (_backlightSignal.flashBacklightTimeout == 0)
	{
		#if !(defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) && !(defined (_USE_DO_TIMER_MODULE_FOR_ESB)) 
		led_control_halSetLedActiveBit(LED_BACKLIGHT_PIN_NUMBER);
		#else
		io_send_led_event(LED_BACKLIGHT_PIN_NUMBER, LED_COMMAND_ACTIVATE); 
		#endif 

	}
}

/***************************************************************************//**
 *
 * @fn void io_backlightSignal (void)
 *
 * @brief  will be count dow the timeout. 
 *
 * @param  none
 * 
 * @return none   
 * 			
 ******************************************************************************/

void io_backlightSignal (void)
{
	static uint8_t backlightBlinkTimer = 0; 
	
	if(_backlightSignal.startFlashBackligth)
	{
		io_watchButtonLight(); 
		
		if(_backlightSignal.flashBacklightCnt != 0)
		{
			if(backlightBlinkTimer != 0)
			{
				backlightBlinkTimer--;
			} 
			else
			{
				backlightBlinkTimer = _backlightSignal.flashBacklightInterval;
				_backlightSignal.flashBacklightCnt--;
				
				if ((_backlightSignal.flashBacklightCnt & 1) == 0)
				{
//						TastBelEin_Active();
				} else
				{
					#if !(defined (PROJECT_TOKEN_BUS_MODULE_ENABLED)) && !(defined (_USE_DO_TIMER_MODULE_FOR_ESB)) 
					led_control_halClrLedActiveBit(LED_BACKLIGHT_PIN_NUMBER); 
					#else
					io_send_led_event(LED_BACKLIGHT_PIN_NUMBER, LED_COMMAND_DEACTIVATE); 
					#endif 
//					
					_backlightSignal.flashBacklightTimeout = _backlightSignal.flashBacklightInterval;
				};
			};
		} 
		else
		{
			_backlightSignal.startFlashBackligth = false; 
			backlightBlinkTimer = 0;
		}
	}
}

uint32_t io_getButtonValue(void)
{
	return _getButtonValue();
}
