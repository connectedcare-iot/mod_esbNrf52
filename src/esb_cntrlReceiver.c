#include <stdint.h>
#include <string.h>
#include <app_error.h>
#include <nrf.h>
#include <nrf_esb.h>
#include <nrf_soc.h>

#include "ble_lib_main.h"
#include "ble_lib_esb.h"

#include "esb_cntrlReceiver.h"
#include "esb_gateway_api.h"
#include "esb_constants.h" 
#include "dp2.h"
#include "app.h"
//#include "misc.h"


/*****************************************************************************************************************
 * macros and prototypes
 *****************************************************************************************************************/

 static bool _eraseEsbAdress = false; 

/*****************************************************************************************************************
 * functions
 *****************************************************************************************************************/

void esb_resetEsb_receiver(void)
{
	bleLib_esbEraseAddress(); 
}	


uint32_t esb_init_receiver(bool start_tech_mode, uint32_t resetReason, bool bouds)
{
	// Init BLE/ESB library
	ble_lib_config_t const config = {
		.eventHandler    = app_bleLibEventHandler,
		.keycodeHandler  = dataHandler_bleEsbKeycodeEventHandler,
		.feedbackHandler = app_feedbackEventHandler,
		.dataHandler     = dataHandler_bleEsbDataEventHandler,
		.eraseBonds      = bouds,
		.eraseDeviceName = bouds
	};

	uint32_t retCode = bleLib_init(&config, resetReason, start_tech_mode);
	VERIFY_SUCCESS(retCode);
	
	return retCode; 
}
