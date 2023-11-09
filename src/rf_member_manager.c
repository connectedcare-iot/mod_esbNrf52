#include "rf_member_manager.h"
#include <stdlib.h>
#include <string.h>
#include "pstorage.h"
#include "app_error.h"
#include "misc.h"


static pstorage_handle_t mp_flash_rf_member;
static uint8_t           m_rf_member_in_flash_count; 


static void rf_member_cb_handler(pstorage_handle_t * handle, uint8_t op_code, uint32_t result, uint8_t * p_data, uint32_t data_len)
{
	APP_ERROR_CHECK(result);
}

#if (PSTORAGE_NUM_OF_PAGES < 7) && defined(ESB)
#error Please adjust PSTORAGE_NUM_OF_PAGES. This lib will need at least 7 pages!
#endif

uint32_t rf_member_manager_init(void)
{
	pstorage_module_param_t param;
	uint32_t err_code;

	BUILD_BUG_ON((sizeof(member_type_t) % 4) != 0);
	BUILD_BUG_ON(sizeof(member_type_t) < PSTORAGE_MIN_BLOCK_SIZE);
	BUILD_BUG_ON(sizeof(member_type_t) > PSTORAGE_MAX_BLOCK_SIZE);
	
	param.block_size  = sizeof(member_type_t);
	param.block_count = 1;
	param.cb          = rf_member_cb_handler;
	
	err_code = pstorage_register(&param, &mp_flash_rf_member);    
	return err_code;
}


bool member_changed(member_type_t * p_member)
{
	uint32_t err_code;
	member_type_t in_flash;
	memset(&in_flash, 0, sizeof(member_type_t));

	err_code = member_info_load_from_flash(&in_flash);
	if(err_code != NRF_SUCCESS)
		return false;

	if(memcmp(&in_flash, p_member, sizeof(in_flash)) != 0)
		return false;

	return true;
}

/**@brief      Function for storing the RF-Member Information to the flash.
 *
 * @param[in]  p_member   Member information to be stored.
 *
 * @return     NRF_SUCCESS on success, an error_code otherwise.
 */
uint32_t rf_member_store(member_type_t * p_member)
{
	uint32_t err_code;
	pstorage_handle_t dest_block;

	//Save Member Information, if changed.
	if(!member_changed(p_member))
	{//Erase flash page.
		err_code = pstorage_clear(&mp_flash_rf_member, (1 * (sizeof (member_type_t))));
		if(err_code != NRF_SUCCESS)
			return err_code;
	}

	//Get block pointer from base
	err_code = pstorage_block_identifier_get(&mp_flash_rf_member, m_rf_member_in_flash_count, &dest_block);
	if(err_code != NRF_SUCCESS)
		return err_code;
		
	//Write Member Information
	err_code = pstorage_store(&dest_block, (uint8_t *)p_member, sizeof(member_type_t), 0);
	if(err_code != NRF_SUCCESS)
		return err_code;
	
	return NRF_SUCCESS;
}


/**@brief      Function for loading the Member Information from flash.
 *
 * @param[out] p_member   Loaded Member Information.
 *
 * @return     NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t member_info_load_from_flash(member_type_t * p_member)
{
	pstorage_handle_t source_block;
	uint32_t          err_code;

	//Get member pointer from base
	err_code = pstorage_block_identifier_get(&mp_flash_rf_member, m_rf_member_in_flash_count, &source_block);
	if(err_code != NRF_SUCCESS)
		return err_code;
	
	//Load member.
	err_code = pstorage_load((uint8_t *)p_member, &source_block, sizeof(member_type_t), 0);
	return err_code;
}


/**@brief      Function for erasing the flash pages that contain Bonding Information and System
 *             Attributes.
 *
 * @return     NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t erase_rf_members(void)
{
	uint32_t err_code;
	
	//Erase flash page.
	err_code = pstorage_clear(&mp_flash_rf_member, (1 * (sizeof (member_type_t))));
	return err_code;
}
