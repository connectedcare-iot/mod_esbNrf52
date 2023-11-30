/**
******************************************************************************
* @file    esb_driver.h
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

#include <stdbool.h>
#include <stdint.h>
#include "esb_constants.h"

#ifndef __ESB_DRIVER_H__
#define __ESB_DRIVER_H__

//+++++++ Rf Commands +++++++++++++++++++++++++++++++++++++++++
#define RF_CMD_MODULID           0x01 ///< @brief Identifizierung
#define RF_CMD_DATA              0x02 ///< @brief Datenübertragung
#define RF_CMD_BUTTON            0x03 ///< @brief Tastenstatus des Handsenders
#define RF_CMD_CHANGE_ADDR       0x04 ///< @brief Adress bzw. Kanalwechsel
#define RF_CMD_SET_CURRENT_ADDR  0x05 ///< @brief current Adr Einstellen
#define	RF_CMD_LED_ON            0x06 ///< @brief led on
#define	RF_CMD_LED_OFF           0x07 ///< @brief led off
#define	RF_CMD_LED_TOGGLE        0x08 ///< @brief led toggle 
#define	RF_CMD_LED_AUTO          0x09 ///< @brief led auto 
#define RF_CMD_WAKE_UP           0x0a ///< @brief wake up 
#define RF_CMD_LED_STATUS        0x0b ///< @brief led status 

#define RF_SIZE_OF_LED_STATE        4 ///< @brief size of led status 
#define RF_LED_STATE_ON    0x00000003 ///< @brief led state on
#define RF_LED_STATE_OFF   0x00000002 ///< @brief led state off 

#define RF_CONTINIOUS_MODE 0x80000000 ///< @brief continus mode 

#define RF_SET_LED_ON_MODE 0x0000F000 ///< @brief set led mode  

#define RF_MAX_NUMBER_OF_TEACHABLE_DEVICES 2 ///< @brief number of teachable devices 

#define ESB_DEFAULT_ADDR         0x817e817e
#define ESB_DEFAULT_ADDR_PREFIX  0x80
#define ESB_DEFAULT_ADDR_CHANNEL 4
 
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


void esb_init_transiver(bool resetDevice);
void esb_deinit_transiver(void);
uint8_t esb_teachModeHandler(void);

bool esb_teachNewDevice(uint8_t newDeviceNumber);

bool esb_resetTeaching(void);
bool esb_stopTeaching(void);

bool esb_transmitKeycode(uint32_t keycode);

bool esb_checkContiniousMode(void);

void esb_wakeUpControls(void);

bool esb_getTeachingLedState (void); 

#endif // __ESB_DRIVER_H__
