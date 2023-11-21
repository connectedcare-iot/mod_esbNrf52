/*****************************************************************************************************************
 * Commands (Funkübertragung)
 *****************************************************************************************************************/
 #ifndef __ESB_CONTROL_RECEIVER_H__
 #define __ESB_CONTROL_RECEIVER_H__

#include "rf_member_manager.h" 

#define RF_MODULID          0x01  // Identifizierung
#define RF_DATA             0x02  // Datenübertragung
#define RF_BUTTON           0x03  // Tastenstatus des Handsenders
#define RF_CHANGE_ADDR      0x04  // Adress bzw. Kanalwechsel
#define RF_SET_CURRENT_ADDR 0x05  // current Adr übernehmen
#define	RF_LED_ON           0x06
#define	RF_LED_OFF          0x07
#define	RF_LED_TOGGLE       0x08
#define	RF_LED_AUTO         0x09
#define	RF_WAKE_UP          0x0A
#define	RF_STATUS           0x0B

void esb_resetEsb_receiver(void); 

uint32_t esb_init_receiver(bool start_tech_mode, uint32_t resetReason, bool bouds); 


#endif 
