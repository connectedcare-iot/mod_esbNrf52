#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "dp2.h"
#include "dp2_keycodequeue.h"
#include "misc.h"

#include "bt_nRF51822Rev3_lib.h"
#include "ble_events.h"
#include "esb.h"

typedef struct
{
	uint32_t delay;
	uint32_t keycode;
	uint8_t duration;
	uint8_t index;
	uint8_t step;
} keycodeQueueElement_t;

static keycodeQueueElement_t keycodeQueue[40];

static signed int searchElement(const uint8_t index, const uint8_t step)
{
	for (int entry = 0; entry < ARRAY_LENGTH_OF(keycodeQueue); entry++)
	{
		if ((keycodeQueue[entry].index == index)
		 && (keycodeQueue[entry].step == step)
		 && ((keycodeQueue[entry].delay != 0)
		  || (keycodeQueue[entry].keycode != 0)
		  || (keycodeQueue[entry].duration != 0)))
		{
			// We have found an element.
			return entry;
		}
	}
	
	// There is no element.
	return -1;
}


static signed int searchFreeElement(void)
{
	for (int entry = 0; entry < ARRAY_LENGTH_OF(keycodeQueue); entry++)
	{
		if ((keycodeQueue[entry].delay == 0)
		 && (keycodeQueue[entry].keycode == 0)
		 && (keycodeQueue[entry].duration == 0))
		{
			// We have found an element.
			return entry;
		}
	}
	
	// There is no element.
	return -1;
}

static void deleteElements(const uint8_t index, const uint8_t step)
{
	for (int entry = 0; entry < ARRAY_LENGTH_OF(keycodeQueue); entry++)
	{
		if ((keycodeQueue[entry].index == index)
		 && (keycodeQueue[entry].step >= step))
		{
			keycodeQueue[entry].delay = 0;
			keycodeQueue[entry].keycode = 0;
			keycodeQueue[entry].duration = 0;
		}
	}
}

int dp2_packetReceived_keycodeQueue(Dp2_packet_t *packet, Dp2_packet_t *answer)
{
	unsigned int dataIndex;
	signed int nextEntry;

	answer->command = DP2_CMD_RF_KEYQUEUE;
	
	if (packet->length < 2)
	{
		// This packet is too small.
		answer->data[0] = DP2_INVALID_PACKET;
		answer->length = 1;
		return DP2_SUCCESS; // Even though this packet is too small, we have created an answer. Thereby we return DP2_SUCCESS because we have succesfully handled the error...
	}
	
	const uint8_t queueIndex = packet->data[0];
	uint8_t step = packet->data[1];
	
	// let's prepare the answer packet - this part is the same for read and write commands...
	answer->data[1] = queueIndex;
	answer->data[2] = step;
	
	if (packet->length == 2)
	{
		// This is a request to read out a queue.
		
		dataIndex = 3;
		
#pragma push
#pragma diag_suppress 1293	//	do not warn on expressions in condition

		while ((nextEntry = searchElement(queueIndex, step) >= 0)   // First we search for the next element
		    && (dataIndex < (answer->length - 9)))                  //  then we make sure there is enough space left in the packet. We need at least 9 bytes.
		{

#pragma pop

			dp2_convertLocalToDp2(keycodeQueue[nextEntry].delay, answer->data + dataIndex, 4);
			dataIndex += 4;

			dp2_convertLocalToDp2(keycodeQueue[nextEntry].keycode, answer->data + dataIndex, 4);
			dataIndex += 4;

			answer->data[dataIndex++] = keycodeQueue[nextEntry].duration;
			
			step++;
		}
		
		answer->length = dataIndex;
		answer->data[0] = DP2_SUCCESS;
		
		return DP2_SUCCESS;
	}
	
	if (packet->length < 11)
	{
		// This packet is not a request to read a queue, but it is too small for a request to add something to a queue...
		// We will ignore this packet.
		answer->length = 3;
		answer->data[0] = DP2_INVALID_PACKET;
		return DP2_SUCCESS; // Even though this packet is invalid, we have created an answer. Thereby we return DP2_SUCCESS because we have succesfully handled the error...
	}
	
	// This is a request to write a queue.
	
	dataIndex = 2;
	
	while ((sizeof(packet->data) - dataIndex >= 9)
		&& (dataIndex <= (packet->length - 9)))
	{
		uint32_t delay = dp2_convertDp2ToLocal(packet->data + dataIndex, 4);
		dataIndex += 4;
		
		uint32_t keycode = dp2_convertDp2ToLocal(packet->data + dataIndex, 4);
		dataIndex += 4;
		
		uint8_t duration = packet->data[dataIndex++];
		
		if ((delay == 0)
		 && (keycode == 0)
		 && (duration == 0))
		{
			deleteElements(queueIndex, step);
		} else
		{
			nextEntry = searchElement(queueIndex, step);    //	1st search for an exisiting entry to override
			
			if (nextEntry < 0)
				nextEntry = searchFreeElement();            //	if nothing exists search for a free entry
			
			if (nextEntry < 0)
			{
				answer->data[0] = DP2_BUFFER_FULL;	// There is no space left...
				answer->data[2] = step;             // we confirm the last written step
				answer->length = 3;
				
				return DP2_SUCCESS;
			}
			
			keycodeQueue[nextEntry].index = queueIndex;
			keycodeQueue[nextEntry].step = step++;
			keycodeQueue[nextEntry].delay = delay;
			keycodeQueue[nextEntry].keycode = keycode;
			keycodeQueue[nextEntry].duration = duration;
		}
	};
	
	answer->data[0] = DP2_SUCCESS;
	answer->data[2] = step; // we confirm the last written step
	answer->length = 3;
	
	return DP2_SUCCESS;
}

static inline int keycodeQueueIsNotEmpty(unsigned int i)
{
	return ((keycodeQueue[i].delay != 0)
	     || (keycodeQueue[i].keycode != 0)
	     || (keycodeQueue[i].duration != 0));
}

void dp2_100msTimer_keycodeQueue(void)
{
	unsigned int i;
	uint32_t nextKeycode = 0;
	
	for (i = 0; i < ARRAY_LENGTH_OF(keycodeQueue); i++)
	{
		if (keycodeQueueIsNotEmpty(i))
		{
			int x = ARRAY_LENGTH_OF(keycodeQueue);
			
			// First we search for another element with the same index, but a lower step number.
			while ((x--)
			    && (!keycodeQueueIsNotEmpty(x)                        // ignore element x if it is empty
			     || (keycodeQueue[i].index != keycodeQueue[x].index)  // ignore element x if its index is different
			     || (keycodeQueue[i].step <= keycodeQueue[x].step))); // ignore element x if its step number is bigger or equal
			
			if (x == -1)
			{
				// This is the lowest/first/next step for this index number.
				
				if (keycodeQueue[i].delay == 0)
				{
					nextKeycode |= keycodeQueue[i].keycode;
					if (keycodeQueue[i].duration == 0)
					{
						// This step has finished, we need to delete this step.
						memset(keycodeQueue + i, 0, sizeof(*keycodeQueue));
					} else
					{
						keycodeQueue[i].duration--;
					};
				} else
				{
					keycodeQueue[i].delay--;
				}
			}
		}
	}
	
	if (lastDp2KeyCode != nextKeycode)
	{
		lastDp2KeyCode = nextKeycode;
		ble_keyCode_received_event(ble_keyCode | esb_keyCode | lastDp2KeyCode);
	}
}
