
/**
******************************************************************************
* @file    keycodes.c 
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

#include <stdbool.h>
#include <stdint.h>
#include "io_driver.h" 
#include "project_configuration.h"


#define MASSAGE_BUTTONS         0xE5F80C00u
#define KEY_M1UP                0x00000001u
#define KEY_M1DOWN              0x00000002u
#define KEY_M2UP                0x00000004u
#define KEY_M2DOWN              0x00000008u
#define KEY_M3UP                0x00000010u
#define KEY_M3DOWN              0x00000020u
#define KEY_M4UP                0x00000040u
#define KEY_M4DOWN              0x00000080u
#define KEY_SYNC                0x00000100u
#define KEY_SOCKET              0x00000200u
#define KEY_MASSAGE_STOP        0x00000400u
#define KEY_MASSAGE_HEAD_PLUS   0x00000800u
#define KEY_MEMORY1             0x00001000u
#define KEY_MEMORY2             0x00002000u
#define KEY_MEMORY5             0x00003000u
#define KEY_MEMORY3             0x00004000u
#define KEY_MEMORY4             0x00008000u
#define KEY_STORE_POSITION      0x00010000u
#define KEY_UBB_BUTTON          0x00020000u
#define KEY_TORCH               0x00040000u
#define KEY_MASSAGE_HEAD        0x00080000u
#define KEY_MASSAGE_FEET        0x00100000u
#define KEY_MASSAGE_ALL         0x00200000u
#define KEY_MASSAGE_FEET_PLUS   0x00400000u
#define KEY_MASSAGE_HEAD_MINUS  0x00800000u
#define KEY_MASSAGE_FEET_MINUS  0x01000000u
#define KEY_COSTOM              0x02000000u
#define KEY_MASSAGE_WAVE        0x04000000u
#define KEY_LOCK                0x08000000u
#define KEY_ALLFLAT             0x10000000u
#define KEY_MASSAGE_PROG_1      0x20000000u
#define KEY_MASSAGE_PROG_2      0x40000000u
#define KEY_MASSAGE_PROG_3      0x80000000u

#define KEY_ZERO_G              KEY_MEMORY1 




#ifdef _DEFAULT_KEYCODE                   // default keycode 
#define KEY_UBB_KEYCODE 0x00020000
#define MOVE045 0x00000500 // (0x100 | 0x400)
#define MOVE049 0x00000A00 // (0x200 | 0x800)
#define MOVE046 0x00000900 // (0x100 | 0x800)
const uint32_t button_teach_mask[2] = {KEY_M1UP, KEY_M1DOWN}; ///< @brief button teachmask for key pair 1 and pair 2
const uint32_t button_torch_mask = 0x10000;                  ///< @brief button teachmask for torch 
const bool     button_blink_flag = true;                        ///< @brief button blink flag - set when feedback blinking is active  
const uint32_t button_memory_mask = KEY_STORE_POSITION;                 ///< @brief button memroy save mask

#define  buttom_ubb_mask   0x0000003;             ///< @brief ubb on mask
const uint32_t key_code_table[PROJECT_NUMBER_OF_KEYS] = {
	KEY_M1UP,                         KEY_M1DOWN,
	                  KEY_LOCK ,
	KEY_M2UP,                         KEY_M2DOWN,
	               KEY_UBB_BUTTON, 
	KEY_M3UP,                         KEY_M3DOWN,
	                KEY_ALLFLAT,
	KEY_M4UP,                          KEY_M4DOWN, 
	                KEY_MEMORY1,
	KEY_MEMORY3, KEY_STORE_POSITION, KEY_MEMORY4,
	                KEY_MEMORY2,
	 KEY_TORCH,                      KEY_MASSAGE_PROG_2,
	               KEY_MASSAGE_STOP,
	KEY_MASSAGE_PROG_1,              KEY_MASSAGE_PROG_3,	
	               KEY_MASSAGE_ALL, 
	KEY_MASSAGE_HEAD_PLUS,         KEY_MASSAGE_FEET_PLUS
	
};                                                              ///< @brief keycode - fill indivdualy for every costumer 
#else
#define KEY_UBB_KEYCODE 0x00020000
#define MOVE045 0x00000500 // (0x100 | 0x400)
#define MOVE049 0x00000A00 // (0x200 | 0x800)
#define MOVE046 0x00000900 // (0x100 | 0x800)
const uint32_t button_teach_mask[2] = {KEY_M1UP, KEY_M1DOWN}; ///< @brief button teachmask for key pair 1 and pair 2
const uint32_t button_torch_mask = KEY_TORCH;                  ///< @brief button teachmask for torch 
const bool     button_blink_flag = true;                        ///< @brief button blink flag - set when feedback blinking is active  
const uint32_t button_memory_mask = KEY_STORE_POSITION;                 ///< @brief button memroy save mask

#define  buttom_ubb_mask   0x0000003;             ///< @brief ubb on mask
const uint32_t key_code_table[PROJECT_NUMBER_OF_KEYS] = {
	KEY_M1UP,                         KEY_M1DOWN,
	                  KEY_LOCK ,
	KEY_M2UP,                         KEY_M2DOWN,
	               KEY_UBB_BUTTON, 
	KEY_M3UP,                         KEY_M3DOWN,
	                KEY_ALLFLAT,
	KEY_M4UP,                          KEY_M4DOWN,
	                KEY_MEMORY1,
	KEY_MEMORY3, KEY_STORE_POSITION, KEY_MEMORY4,
	                KEY_MEMORY2,
	 KEY_TORCH,                      KEY_MASSAGE_PROG_2,
	               KEY_MASSAGE_STOP,
	KEY_MASSAGE_PROG_1,              KEY_MASSAGE_PROG_3,	
	               KEY_MASSAGE_ALL, 
	KEY_MASSAGE_HEAD_PLUS,         KEY_MASSAGE_FEET_PLUS
};                                                              ///< @brief keycode - fill indivdualy for every costumer  

#endif 

/***************************************************************************//**
 *
 * @fn uint32_t key_getTorchMask (void)
 *
 * @brief  returns the torch key mask 
 *
 * @param  none
 * 
 * @return torch key mask    
 * 			
 ******************************************************************************/  

uint32_t key_getTorchMask (void)
{
	return button_torch_mask; 
}

/***************************************************************************//**
 *
 * @fn uint32_t key_getTeachMask (uint8_t array)
 *
 * @brief  returns the teach key masks
 *
 * @param  uint8_t array (should be 0 or 1) 
 * 
 * @return returns button_teach_mask[0] or button_teach_mask[1] 
 * 			
 ******************************************************************************/  

uint32_t key_getTeachMask (uint8_t array)
{
	return button_teach_mask[array]; 
}

/***************************************************************************//**
 *
 * @fn uint32_t key_getTeachMask (uint8_t array)
 *
 * @brief  returns the keycode table 
 *
 * @param  uint8_t array (should be between 0 or 15) 
 * 
 * @return returns key_code_table[array]
 * 			
 ******************************************************************************/  

uint32_t key_getKeyCode (uint8_t array)
{
	return key_code_table[array]; 
}

/***************************************************************************//**
 *
 * @fn uint8_t key_getKeyCodeSize (void)
 *
 * @brief  returns the size of the keycode table  
 *
 * @param  none 
 * 
 * @return size = sizeof(key_code_table) / sizeof(key_code_table[0])
 * 			
 ******************************************************************************/  

uint8_t key_getKeyCodeSize (void)
{
	return (sizeof(key_code_table) / sizeof(key_code_table[0])); 
}

/***************************************************************************//**
 *
 * @fn bool key_gehtBlinkFlag (void)
 *
 * @brief  returns the blink flag of the application - set when activ
 *
 * @param  none 
 * 
 * @return boolean -> true when blick is active in the device 
 * 			
 ******************************************************************************/  

bool key_gehtBlinkFlag (void)
{
	return button_blink_flag; 
}

/***************************************************************************//**
 *
 * @fn uint32_t key_getMemorySaveMask (void)
 *
 * @brief  returns the meory save mask of the device
 *
 * @param  none 
 * 
 * @return uint32_t -> memory save mask 
 * 			
 ******************************************************************************/  

uint32_t key_getMemorySaveMask (void)
{
	return button_memory_mask;
}

#ifdef  _USE_TWO_KEYS_FOR_UBB
uint32_t key_getUbbMask (void)
{
	return buttom_ubb_mask; 
}

uint32_t key_getUbbKeycode (void)
{
	return KEY_UBB_KEYCODE;
}
#endif 

#ifdef _RESET_DRIVE
uint32_t key_getRefDriveMask (void)
{
	return KEY_REF_DRIVE_KEYCODE;
}
#endif
