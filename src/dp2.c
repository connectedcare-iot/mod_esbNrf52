#include <stdint.h>
#include <string.h>
#include "dp2.h"
#include "dp2_keycodequeue.h"

#define NUM_MAX_DP2_VALUES 4

static struct
{
	uint_fast8_t index;
	uint8_t *data;
	uint_fast8_t dataSize;	///< This is initialized to zero by C standard. If dataSize equals zero this entry is empty.
} dp2_values [NUM_MAX_DP2_VALUES];

uint32_t lastDp2KeyCode = 0;

void dp2_setValue(uint_fast8_t index, uint8_t *data, uint_fast8_t dataSize)
{
	unsigned int i;
	for (i = 0; i < NUM_MAX_DP2_VALUES; i++)
	{
		// first we seatch for an existing entry with the same index...
		if (dp2_values[i].index == index)
			break;
	}
	
	if (i >= NUM_MAX_DP2_VALUES)
	{
		for (i = 0; i < NUM_MAX_DP2_VALUES; i++)
		{
			// if no entry with our index was found we search for a free entry...
			if (dp2_values[i].dataSize == 0)
				break;
		}
	}
	
	if (i < NUM_MAX_DP2_VALUES)
	{
		dp2_values[i].index = index;
		dp2_values[i].data = data;
		dp2_values[i].dataSize = dataSize;
	}
};

int_fast8_t dp2_getNextPacket(Dp2_packet_t *packet)
{
	// currently only RF_VALUE commands are supported...
	
	static unsigned int lastEntry = 0;
	unsigned int i;
	for (i = 0; i < NUM_MAX_DP2_VALUES; i++)
	{
		unsigned int entry = (i + lastEntry) % NUM_MAX_DP2_VALUES;
		
		if (dp2_values[entry].dataSize != 0)
		{
			lastEntry = i;
			
			packet->command = DP2_CMD_RF_VALUE;
			packet->length = dp2_values[entry].dataSize + 1;
			packet->data[0] = dp2_values[entry].index;
			
			if (packet->length > sizeof(packet->data))
				packet->length = sizeof(packet->data);
			
			memcpy(packet->data + 1, dp2_values[entry].data, packet->length - 1);
			
			return -1;
		}
	}
	
	// there are no packets in queue
	return 0;
}

uint_fast8_t dp2_getNumPackets(void)
{
	// currently only RF_VALUE commands are supported...
	
	uint_fast8_t numberOfPackets = 0;
	unsigned int i;
	for (i = 0; i < NUM_MAX_DP2_VALUES; i++)
	{
		if (dp2_values[i].dataSize != 0)
			numberOfPackets++;
	}
	
	return numberOfPackets;
}

int dp2_packetReceived(Dp2_packet_t *packet, Dp2_packet_t *answer)
{
	int retValue;
	
	if (answer->length > sizeof(answer->data))
		answer->length = sizeof(answer->data);
	
	switch(packet->command)
	{
		case DP2_CMD_RF_KEYQUEUE:
			retValue = dp2_packetReceived_keycodeQueue(packet, answer);
			break;
	}
	
	return retValue;
}

void dp2_100msTimer(void)
{
	dp2_100msTimer_keycodeQueue();
}

uintmax_t dp2_convertDp2ToLocal(uint8_t * data, size_t length)
{
	uintmax_t value = 0;
	
	for (size_t i = 0; i < length; i++)
	{
		value <<= 8;
		value += data[i];
	}
	
	return value;
}

void dp2_convertLocalToDp2(uintmax_t value, uint8_t * data, size_t length)
{
	while(length--)
	{
		data[length] = value;
		value >>= 8;
	};
}
