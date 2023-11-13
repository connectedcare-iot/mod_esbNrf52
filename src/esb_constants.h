
#ifndef __ESB_CONSTANTS_H__
#define __ESB_CONSTANTS_H__

#define RF_LENGTH_OF_ADDRESS               4       ///< @brief  RF Adress 
#define RF_LENGHT_OF_PRE_ADDRESS           1       ///< @brief  RF Pre Adress 
#define RF_PREADDRESS_MASK                 0xF0    ///< @brief  RF Pre Adress Mask  
#define RF_PIPE_MASK                       0x0F    ///< @brief  RF Pipe Mask

#define RF_MAX_CHANNEL_NUMBER              84      ///< @brief  RF Adress max Channel Number
#define RF_MIN_CHANNEL_NUMBER              0       ///< @brief  RF Adress min Channel Number

#define ESB_FLAG_NONE                      0x00    ///< @brief  ESB Flag not set
#define ESB_FLAG_ACK                       0x01    ///< @brief  ESB Flag for TX success/ACK
#define ESB_FLAG_NACK                      0x02    ///< @brief  ESB Flag for TX failed/nACK
#define ESB_FLAG_DATA                      0x04    ///< @brief  ESB Flag for received data

#define RF_ESB_CMD                         2       ///< @brief  RF ESB cmd byte

#define RF_ESB_HEADER_LENGTH               3       ///< @brief  RF ESB Header length
#define RF_NUMBER_OF_USED_PIPES            6       ///< @brief  RF Number of used pipes 
#define RF_KEYCODE_TRANSMIT_PIPE           4       ///< @brief  RF Keycode for tramited pipe
#define RF_KEYCODE_ALTERNATE_TRANSMIT_PIPE 1       ///< @brief  RF Keycode for alternate pipe 

#define RF_TEACH_BLINK_INTERVAL            2       ///< @brief  blink interval 
#define RF_TEACH_BLINK_TIME                2       ///< @brief  blink time

#define LENGHT_OF_BYTE                     8       ///< @brief  esb byte length
#define ESB_GET_BYTE(rawData, byteNumber) ((rawData >> (LENGHT_OF_BYTE * byteNumber)) & 0xFF)

#ifdef SOFTDEVICE_NOT_PRESENT
#define NRF_ESB_SWI_IRQn SWI1_IRQn              ///< Software interrupt # used for callback functions.
#define NRF_ESB_SWI_IRQ_HANDLER SWI1_IRQHandler ///< Software interrupt handler used for callback functions.

#define NRF_ESB_TIMER NRF_TIMER0                               ///< Timer to be used as flywheel timer.
#define NRF_ESB_TIMER_PERPOWER_Msk POWER_PERPOWER_TIMER0_Msk   ///< PERPOWER mask for the timer.
#define NRF_ESB_TIMER_IRQn TIMER0_IRQn                         ///< Interrupt # for the timer.
#define NRF_ESB_TIMER_IRQ_HANDLER TIMER0_IRQHandler            ///< Interrupt handler for the timer.    
#endif 

#endif // __ESB_CONSTANTS_H__
