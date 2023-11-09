#ifndef RF_MEMBER_MANAGER_H__
#define RF_MEMBER_MANAGER_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

typedef __align(4) __packed struct
{
	uint32_t addr;
	uint8_t addrPrefix;
	uint8_t channel;
	uint8_t pipe;
	uint8_t dummy[9];
} member_type_t;

uint32_t rf_member_manager_init(void);
uint32_t member_info_load_from_flash(member_type_t * p_member);
uint32_t rf_member_store(member_type_t * p_member);
bool     member_changed(member_type_t * p_member);
uint32_t erase_rf_members(void);

#endif
