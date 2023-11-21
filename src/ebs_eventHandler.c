#include "esb_eventHandler.h"
#include "dp2.h"

#include "hardware_information.h"
#include "led_control_api.h"

static bool _teachActive = false; 

void dataHandler_bleEsbKeycodeEventHandler(ble_lib_keycode_evt_t const * eventPtr)
{
	VERIFY_PARAM_NOT_NULL_VOID(eventPtr);
	static uint32_t esbKeycode = 0;
	
	switch (eventPtr->comm_type)
	{
		case BLE_LIB_COMM_TYPE_BLE:
		{
//			// RF-Handset has the highest priority, ignore Bluetooth keycodes
//			if (esbKeycode == 0)
//			{
//				_data.bleEsbKeycode = eventPtr->keycode;
//				keycodeHandler();
//			}
		} break;
		
		case BLE_LIB_COMM_TYPE_ESB:
		{
//			esbKeycode          = eventPtr->keycode;
//			_data.bleEsbKeycode = eventPtr->keycode;
//			keycodeHandler();
		} break;
		
		default:
		{
			// Do nothing
		} break;	
	}
}

void bleLibEventHandler(ble_lib_evt_t const event)
{
		uint32_t errCode;
	
	switch (event)
	{
		// General events
		case BLE_LIB_EVT_DEVICE_WILL_RESTART:
		{
//			NRF_LOG_INFO("BLE_LIB_EVT_DEVICE_WILL_RESTART");
//			_states.isRestartPending = true;
//			
//			errCode = led_indicationSet(LED_INDICATE_RESTART);
//			APP_ERROR_CHECK(errCode);
//			
//			errCode = app_sched_event_put(NULL,
//			                              0,
//			                              prepareRestart);
//			APP_ERROR_CHECK(errCode);
		} break;
		
		// BLE events
		case BLE_LIB_EVT_BLE_OFF_MODE:
		{
//			NRF_LOG_INFO("BLE_LIB_EVT_BLE_OFF_MODE");
//			_states.bleState = event;
//			
//			errCode = led_indicationSet(LED_INDICATE_BLE_OFF);
//			APP_ERROR_CHECK(errCode);
		} break;
		
		case BLE_LIB_EVT_BLE_IDLE_MODE:
		{
//			NRF_LOG_INFO("BLE_LIB_EVT_BLE_IDLE_MODE");
//			_states.bleState = event;
//			
//			errCode = led_indicationSet(LED_INDICATE_BLE_IDLE);
//			APP_ERROR_CHECK(errCode);
		} break;
		
		case BLE_LIB_EVT_BLE_STANDBY_MODE:
		{
//			NRF_LOG_INFO("BLE_LIB_EVT_BLE_STANDBY_MODE");
//			_states.bleState = event;
//			
//			errCode = led_indicationSet(LED_INDICATE_BLE_STANDBY);
//			APP_ERROR_CHECK(errCode);
		} break;
		
		case BLE_LIB_EVT_BLE_PAIRING_MODE:
		{
//			NRF_LOG_INFO("BLE_LIB_EVT_BLE_PAIRING_MODE");
//			_states.bleState = event;
//			
//			errCode = led_indicationSet(LED_INDICATE_BLE_PAIRING);
//			APP_ERROR_CHECK(errCode);
		} break;
		
		case BLE_LIB_EVT_BLE_NORMAL_MODE:
		{
//			NRF_LOG_INFO("BLE_LIB_EVT_BLE_NORMAL_MODE");
//			_states.bleState = event;
//			
//			errCode = led_indicationSet(LED_INDICATE_BLE_NORMAL);
//			APP_ERROR_CHECK(errCode);
		} break;
		
		case BLE_LIB_EVT_BLE_CENTRAL_CONNECTED:
		{
//			NRF_LOG_INFO("BLE_LIB_EVT_BLE_CENTRAL_CONNECTED");
//			
//			_states.bleState        = event;
//			_states.isPeerConnected = true;
//			setPower(true);
//			
//			errCode = led_indicationSet(LED_INDICATE_BLE_CONNECTED);
//			APP_ERROR_CHECK(errCode);
		} break;
		
		case BLE_LIB_EVT_BLE_CENTRAL_DISCONNECTED:
		{
//			NRF_LOG_INFO("BLE_LIB_EVT_BLE_CENTRAL_DISCONNECTED");
//			
//			_states.bleState        = event;
//			_states.isPeerConnected = false;
//			
//			errCode = led_indicationSet(LED_INDICATE_BLE_DISCONNECTED);
//			APP_ERROR_CHECK(errCode);
		} break;
		
		// ESB events
		case BLE_LIB_EVT_ESB_IDLE_MODE:
		{
			_teachActive = false; 
			led_control_halClrLedActiveBit(LED_LED1_PIN_NUMBER);
			led_control_halClrLedActiveBit(LED_LED2_PIN_NUMBER);
			led_control_halClrLedActiveBit(LED_LED3_PIN_NUMBER);
			led_control_halSetLedActiveBit(LED_BACKLIGHT_PIN_NUMBER);
//			NRF_LOG_INFO("BLE_LIB_EVT_ESB_IDLE_MODE");
//			_states.esbState = event;
//			
//			errCode = led_indicationSet(LED_INDICATE_ESB_IDLE);
//			APP_ERROR_CHECK(errCode);
		} break;
		
		case BLE_LIB_EVT_ESB_STANDBY_MODE:
		{
			_teachActive = false; 
//			NRF_LOG_INFO("BLE_LIB_EVT_ESB_STANDBY_MODE");
//			_states.esbState = event;
//			
//			errCode = led_indicationSet(LED_INDICATE_ESB_STANDBY);
//			APP_ERROR_CHECK(errCode);
		} break;
		
		case BLE_LIB_EVT_ESB_TEACH_MODE:
		{
			_teachActive = true; 
			led_control_halSetLedActiveBit(LED_LED1_PIN_NUMBER);
			led_control_halSetLedActiveBit(LED_LED2_PIN_NUMBER);
			led_control_halSetLedActiveBit(LED_LED3_PIN_NUMBER);
			led_control_halClrLedActiveBit(LED_BACKLIGHT_PIN_NUMBER);
//			NRF_LOG_INFO("BLE_LIB_EVT_ESB_TEACH_MODE");
//			
//			setPower(true);
//			_states.esbState = event;
//			
//			errCode = led_indicationSet(LED_INDICATE_ESB_TEACH);
//			APP_ERROR_CHECK(errCode);
		} break;
		
		case BLE_LIB_EVT_ESB_NORMAL_MODE:
		{
			_teachActive = false; 
//			NRF_LOG_INFO("BLE_LIB_EVT_ESB_NORMAL_MODE");
//			
//			setPower(true);
//			_states.esbState = event;
//			
//			errCode = led_indicationSet(LED_INDICATE_ESB_NORMAL);
//			APP_ERROR_CHECK(errCode);
		} break;
		
		// Succeessful events
		case BLE_LIB_EVT_BLE_PAIRING_SUCCESS:
//			setPower(true);
		case BLE_LIB_EVT_BLE_ERASE_DEVICE_NAME_SUCCESS:
		case BLE_LIB_EVT_BLE_ERASE_BONDS_SUCCESS:
		case BLE_LIB_EVT_ESB_TEACH_SUCCESS:
		case BLE_LIB_EVT_ESB_RESET_SUCCESS:
		{
			_teachActive = false; 
//			indicateStatus(true);
		} break;
		
		case BLE_LIB_EVT_ESB_TEACH_FAILED:
		case BLE_LIB_EVT_ESB_RESET_FAILED:
		{
			led_control_halClrLedActiveBit(LED_BLE_PIN_NUM);
//			indicateStatus(false);
		} break;
		
		default:
		{
			// Not handled
		} break;
	}
	
}

void feedbackEventHandler(ble_lib_feedback_evt_t const * eventPtr,
                                 uint32_t                     * ledcodePtr,
                                 uint32_t                     * maskPtr,
                                 uint8_t                      * statusPtr)
{
	VERIFY_PARAM_NOT_NULL_VOID(eventPtr);
	VERIFY_PARAM_NOT_NULL_VOID(ledcodePtr);
	VERIFY_PARAM_NOT_NULL_VOID(maskPtr);
	VERIFY_PARAM_NOT_NULL_VOID(statusPtr);
	
	uint32_t errCode; 
	
	switch (eventPtr->comm_type)
	{
		case BLE_LIB_COMM_TYPE_ESB:
//		case BLE_LIB_COMM_TYPE_BLE:
		{
			if (eventPtr->feedback_evt == BLE_LIB_EVT_FEEDBACK_REQ)  /**< Feedback request, this will be send to the handset and to connected central (BT) */
			{
				// Get UBL feedback
				uint32_t ledFeedback = 0;
				uint32_t ledMask     = 0;
				
//				uint32_t errCode = led_getFeedback(&ledFeedback,
//				                                   &ledMask);
//				APP_ERROR_CHECK(errCode);
				
				// Get lock feedbacks
				uint32_t lockFeedback = 0;
				uint32_t lockMask     = 0;
				
//				errCode = lock_getFeedback(&lockFeedback,
//				                           &lockMask);
//				APP_ERROR_CHECK(errCode);
				
				// Get sync feedbacks
				uint32_t syncFeedback = 0;
				uint32_t syncMask     = 0;
				
//				errCode = sync_getFeedback(&syncFeedback,
//				                           &syncMask);
//				APP_ERROR_CHECK(errCode);
//				
				// Get memory feedbacks
				uint32_t memoryFeedback = 0;
				uint32_t memoryMask     = 0;
////				
////				errCode = memory_getFeedback(&memoryFeedback,
////				                             &memoryMask);
//				APP_ERROR_CHECK(errCode);
//				
				// Get memory feedbacks
				uint32_t massageFeedback = 0;
				uint32_t massageMask     = 0;
				uint8_t  massageStatus   = 0xFF;
//				
//				errCode = appMassage_getFeedback(&massageFeedback,
//				                                 &massageMask,
//				                                 &massageStatus);
				APP_ERROR_CHECK(errCode);
				
				// Combine feedbacks
				*ledcodePtr = ledFeedback | lockFeedback | memoryFeedback | syncFeedback | massageFeedback;
				*maskPtr    = ledMask | lockMask | memoryMask | syncMask | massageMask;
				*statusPtr  = massageStatus;
			}
		} break;
		
		default:
		{
			// Not defined
		} break;
	}
}

void dataHandler_bleEsbDataEventHandler(ble_lib_comm_source_t const * sourcePtr,
                                        uint8_t               const * dataPtr,
                                        uint16_t              const   dataSize)
{
		VERIFY_PARAM_NOT_NULL_VOID(sourcePtr);
	VERIFY_PARAM_NOT_NULL_VOID(dataPtr);
	
	if (sourcePtr->commType == BLE_LIB_COMM_TYPE_BLE)
	{
		//TODO
	}
	else if (sourcePtr->commType == BLE_LIB_COMM_TYPE_ESB)
	{
		//TODO
	}
}

bool getTeachReceiverState (void)
{
	return _teachActive; 
}

