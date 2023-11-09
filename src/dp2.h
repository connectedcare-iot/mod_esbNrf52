typedef struct
{
	uint8_t command;
	uint8_t length;
	uint8_t data[30];
} Dp2_packet_t;

#define DP2_CMD_RF_VALUE    0x0C
#define DP2_CMD_RF_KEYQUEUE 0x0D

#define DP2_SUCCESS         0x00
#define DP2_INVALID_PACKET  0x01
#define DP2_BUFFER_FULL     0x02

#define dp2GetKeycode()         lastDp2KeyCode
#define dp2_removeValue(index)  dp2_setValue(index, NULL, 0)

void dp2_setValue(uint_fast8_t index, uint8_t *data, uint_fast8_t dataSize);
int_fast8_t dp2_getNextPacket(Dp2_packet_t *packet);
uint_fast8_t dp2_getNumPackets(void);
int dp2_packetReceived(Dp2_packet_t *packet, Dp2_packet_t *answer);
void dp2_100msTimer(void);

uintmax_t dp2_convertDp2ToLocal(uint8_t * data, size_t length);
void dp2_convertLocalToDp2(uintmax_t value, uint8_t * data, size_t length);

extern uint32_t lastDp2KeyCode;
