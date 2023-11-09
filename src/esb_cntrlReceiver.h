/*****************************************************************************************************************
 * Commands (Funkübertragung)
 *****************************************************************************************************************/
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

enum rfMode_t
{
	rfMode_normal = 0,
	rfMode_teach_tx = 1,
	rfMode_teach_rx = 2,
	rfMode_teach_change_channel = 3,
#ifndef NO_SOCKET_SUPPORT
	rfMode_teach_pending = 4,
	rfMode_tx = 5
#else
	rfMode_teach_pending = 4
#endif
};
extern enum rfMode_t rfMode;

#define esbTeachmodeIsActive() (rfMode != rfMode_normal)

void esb_init(bool start_tech_mode);
void esb_timer_10ms(void);
void esb_handler(void);
void esb_stop(void);
void esb_sys_event_hanlder(uint32_t sys_evt);
void esb_flash_feedback(uint_fast8_t interval, uint_fast8_t count);
void esb_teachmode_stop(void);

extern uint32_t esb_keyCode;

#include "rf_member_manager.h"

extern member_type_t	esbAddress;
