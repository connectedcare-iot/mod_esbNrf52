
/**
******************************************************************************
* @file    keycodes.h 
* @author  Jens Wörmann
* @version V1.0
* @date    05.02.20
* @brief   this module handles the keycode functioanlys. 
******************************************************************************
* @attention
*
*
*
******************************************************************************
*/


#ifndef __KEYCODES_H__
#define __KEYCODES_H__

#include <stdbool.h>
#include <stdint.h>

// prototypes 

uint32_t key_getTorchMask(void); 
uint32_t key_getTeachMask(uint8_t array); 
uint32_t key_getKeyCode(uint8_t array); 
uint8_t key_getKeyCodeSize(void); 
bool key_gehtBlinkFlag(void); 
uint32_t key_getMemorySaveMask(void); 

#ifdef  _USE_TWO_KEYS_FOR_UBB
uint32_t key_getUbbMask(void); 
uint32_t key_getUbbKeycode(void);
#endif 

#ifdef _RESET_DRIVE
uint32_t key_getRefDriveMask (void); 
#endif 

#endif // __KEYCODES_H__
