/**
******************************************************************************
* @file    io_driver.h
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

#include <stdbool.h>
#include <stdint.h>


#ifndef __IO_DRIVER_H__
#define __IO_DRIVER_H__

#define ON                             1  ///< @brief define for on 
#define OFF                            0  ///< @brief define for off

/////////////////////////////////////

// Timeout for which no keycode can be send after release of the Teaching Buttons
// This is not written in any specification!


#define IO_MEMORY_OFF_CNT              200 ///< @brief IO Timeout

#define KEY_VALUE_STOP         0x00000000 ///< @brief default key value for stop the system 

#define DEBOUNCE_MAX_COUNT              3 ///< @brief debaunce time counter 

#define FCT_LED                        4 ///< @brief FCT LED Pin
#define FCT_LED_2                      3 ///< @brief FCT LED Pin
#define FCT_LED_3                      2 ///< @brief FCT LED Pin
#define BUTTON_LED                     15 ///< @brief button LED Pin
#define TORCH                          20 ///< @brief torch led pin 

#define IO_POWER                       1

#define BUTTON_MEMORY_MASK     key_getMemorySaveMask()  ///< @brief define for getting memory mask 
#define BUTTON_TORCH_MASK      key_getTorchMask()    ///< @brief define for getting torch mask 
#define BUTTON_TEACH_MASK_NR1  key_getTeachMask(0)   ///< @brief define for getting teach mask 1
//#ifdef  FEATURE_TEACH_TWO_DEVICES
#define BUTTON_TEACH_MASK_NR2  key_getTeachMask(1)   ///< @brief define for getting teach mask 2
//#endif
#ifdef  _USE_TWO_KEYS_FOR_UBB
#define BUTTOM_UBB_MASK        key_getUbbMask()        ///< @brief define for getting the ubb mask 
#endif 
#ifdef _RESET_DRIVE
#define BUTTOM_REF_DRIVE_MASK  0x0101         ///< @brief define for getting the ref Drive mask 
#endif

#define IO_GENERATE_FEATURE_MASK(feature) (feature == IO_FEATURE_NOTHING ? 0 : (1 << feature))
#define IO_IS_FEATURE_ACTIVE(featureMask, feature) ((IO_GENERATE_FEATURE_MASK(feature) & featureMask) != 0 ? true : false)
#define IO_WAS_FEATURE_ACTIVATED(newfeatureMask, oldFeatureMask, feature) ((IO_IS_FEATURE_ACTIVE(newfeatureMask, feature) == true && (IO_IS_FEATURE_ACTIVE(oldFeatureMask, feature) == false)) ? true : false)
#define IO_WAS_FEATURE_DEACTIVATED(newfeatureMask, oldFeatureMask, feature) ((IO_IS_FEATURE_ACTIVE(oldFeatureMask, feature) == true && (IO_IS_FEATURE_ACTIVE(newfeatureMask, feature) == false)) ? true : false)

#define KEY_UBB_RELEASE_TIME         40

typedef enum 
{
	IO_FEATURE_TEACHING,                               ///< @brief feature state teaching
	IO_FEATURE_FLASHLIGHT,                             ///< @brief feature state flash light
	IO_FEATURE_KEY_CHANGED,                            ///< @brief feature state key change
	IO_FEATURE_KEY_PRESSED,                            ///< @brief feature state key pressed
	IO_FEATURE_KEY_RELEASED,                           ///< @brief feature state key released
	IO_FEATRUE_MEMORY_SAVE,                            ///< @brief feature memoy save key pressed
	IO_FEATURE_UBB,                                    ///< @brief feature state flash light
	IO_FEATURE_REF_DRIVE, 
	IO_FEATURE_NOTHING,                                ///< @brief nothing 
}io_feature_t;


// prototypes 

void io_init(void);
void io_deinit(void);
bool io_checkButtons(void);
void io_setFlashBacklight (uint8_t interval, uint8_t count); 
void io_backlightSignal (void); 
void ioCheckSleep(void);
uint32_t io_getButtonValue(void);
bool io_getWaitForUbbFlag (void); 
#endif // __IO_DRIVER_H__
