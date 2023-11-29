#ifndef __ESB_GATEWAY_API
#define __ESB_GATEWAY_API

#include <stdbool.h>
#include <stdint.h>

#include "ble_lib_main.h"

/** @brief Function to receive ble/esb events.
 *
 * @param[out] eventPtr   BLE/ESB keycode events.
 *
 * @retval nrf_error  Error code containing information about what went wrong
 */

void dataHandler_bleEsbKeycodeEventHandler(ble_lib_keycode_evt_t const * eventPtr);

/** @brief Function for handle library BLE/ESB events.
 *
 * @param[in] eventPtr BLE/ESB lib event
 */

void bleLibEventHandler(ble_lib_evt_t const event); 

/** @brief Function to handle feedback events. 
 *
 * @param[in]  eventPtr    Feedback event 
 * @param[out] ledcodePtr  Ledcode
 * @param[out] maskPtr     Feedback mask
 * @param[out] statusPtr   LED status 
 */

void feedbackEventHandler(ble_lib_feedback_evt_t const * eventPtr,
                                 uint32_t                     * ledcodePtr,
                                 uint32_t                     * maskPtr,
                                 uint8_t                      * statusPtr); 

 /** @brief Function to receive ble/esb data events.
 *
 * @param[in] sourcePtr  Source.
 * @param[in] sourcePtr  Pointer to data.
 * @param[in] dataSize   Data size.
 *
 * @retval nrf_error  Error code containing information about what went wrong
 */

void dataHandler_bleEsbDataEventHandler(ble_lib_comm_source_t const * sourcePtr,
                                        uint8_t               const * dataPtr,
                                        uint16_t              const   dataSize);

bool getTeachHandsetState (void); 



#endif 