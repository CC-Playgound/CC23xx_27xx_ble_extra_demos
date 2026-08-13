/******************************************************************************

@file  app_ranging_client_api.h

@brief This file contains the ranging client APIs and structures.


Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2025-2026, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

******************************************************************************


*****************************************************************************/

#ifndef APP_RANGING_CLIENT_API_H
#define APP_RANGING_CLIENT_API_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef RANGING_CLIENT
/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "ti/ble/profiles/ranging/ranging_profile_client.h"
#include "app_extctrl_common.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * ENUMS
 */

// RREQ event types
typedef enum
{
  APP_RREQ_EVENT_DATA_READY = 0x01,
  APP_RREQ_EVENT_STATUS     = 0x02
} AppRREQEventType_e;

/*********************************************************************
 * TYPEDEFS
 */

// Once this function returns the pSegmentsReader is owned by the callback
// handler. pSegmentsReader contains valid data only when status is SUCCESS.
typedef void (*AppRREQ_dataEventHandler_t)(uint16_t connHandle, uint16_t rangingCount, uint8_t status, RangingDBClient_procedureSegmentsReader_t pSegmentsReader);

// Event handler for RREQ events
typedef void (*AppRREQ_eventHandler_t)(AppRREQEventType_e eventType, BLEAppUtil_msgHdr_t *pMsgData);

typedef struct
{
  uint16_t connHandle;    // Connection handle
  uint16_t rangingCount;  // CS procedure counter
} AppRREQDataReady_t;

typedef struct
{
  uint16_t connHandle;    // Connection handle
  uint8_t  statusCode;    // Error code indicating the type of error
  uint8_t  statusDataLen; // Length of statusData
  uint8_t *statusData;    // Additional data related to the error, if any
} AppRREQStatus_t;

/*********************************************************************
 * Functions
 */

/*********************************************************************
 * @fn      AppRREQ_start
 *
 * @brief   This function is called to start the RREQ module.
 *
 * input parameters
 *
 * @param   none
 *
 * output parameters
 *
 * @param   None
 *
 * @return  SUCCESS or an error status indicating the failure reason.
 */
uint8_t AppRREQ_start(void);

/*********************************************************************
 * @fn      AppRREQ_enable
 *
 * @brief Enables the RREQ process.
 *        This function start the RREQ process by discovering
 *        the RAS (Ranging Service) service on the specified
 *        connection handle
 *
 * input parameters
 *
 * @param connHandle - The connection handle for the RAS service
 * @param enableMode - The mode to enable
 *
 * output parameters
 *
 * @param   None
 *
 * @return  SUCCESS - if the RREQ process was successfully enabled.
 *          blePending - if the operation is pending and will be completed later.
 *          bleMemAllocError - if memory allocation failed.
 *          INVALIDPARAMETER - if the connection handle is invalid or the enable mode is not supported.
 */
uint8_t AppRREQ_enable(uint16_t connHandle, RREQEnableModeType_e enableMode);

/*********************************************************************
 * @fn      AppRREQ_disable
 *
 * @brief Disable the RREQ process.
 *        This function start the RREQ process by discovering
 *        the RAS (Ranging Service) service on the specified
 *        connection handle
 *
 * input parameters
 *
 * @param connHandle - Connection handle.
 *
 * output parameters
 *
 * @param   None
 *
 * @return SUCCESS - if the RREQ process was successfully disabled.
 *         INVALIDPARAMETER - if the connection handle is invalid.
 */
uint8_t AppRREQ_disable(uint16_t connHandle);

/*********************************************************************
 * @fn      AppRREQ_abort
 *
 * @brief abort the RREQ process.
 *        This function abort the RREQ process.
 *
 * input parameters
 *
 * @param connHandle - Connection handle.
 *
 * output parameters
 *
 * @param   None
 *
 * @return SUCCESS - if the RREQ process was successfully aborted.
 *         INVALIDPARAMETER - if the connection handle is invalid.
 */
uint8_t AppRREQ_abort(uint16_t connHandle);

/*********************************************************************
 * @fn      AppRREQ_GetRangingData
 *
 * @brief   Starts the process of reading data for a ranging request.
 *          This function initiates the data reading process for a specified
 *          connection handle and ranging count.
 *
 * input parameters
 *
 * @param   connHandle - Connection handle.
 * @param   RangingCount - CS procedure counter.
 *
 * output parameters
 *
 * @param   None
 *
 * @return return SUCCESS or an error status indicating the failure reason.
 */
uint8_t AppRREQ_GetRangingData(uint16_t connHandle, uint16_t rangingCount);

/*********************************************************************
 * @fn      AppRREQ_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * input parameters
 *
 * @param   fEventHandler - pointer to the event handler function.
 *
 * output parameters
 *
 * @param   None
 *
 * @return  None
 */
void AppRREQ_registerEvtHandler(AppRREQ_eventHandler_t fEventHandler);

/*********************************************************************
 * @fn      AppRREQ_registerDataCompleteEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function for data complete events.
 *
 * input parameters
 *
 * @param   pDataEventHandler - pointer to the event handler function.
 *
 * output parameters
 *
 * @param   None
 *
 * @return  None
 */
void AppRREQ_registerDataEvtHandler(AppRREQ_dataEventHandler_t pDataEventHandler);

/*********************************************************************
 * @fn      AppRREQ_configureCharRegistration
 *
 * @brief   Configure characteristic registration for the RREQ client.
 *
 * @param   connHandle - Connection handle.
 * @param   charUUID   - Characteristic UUID to configure.
 * @param   subscribeMode - Subscription mode for the characteristic.
 *
 * @return  SUCCESS or error code.
 */
uint8_t AppRREQ_configureCharRegistration(uint16_t connHandle, uint16_t charUUID, uint8_t  subscribeMode);

/*********************************************************************
 * @fn      AppRREQ_procedureStarted
 *
 * @brief Notifies the RREQ profile that a Channel Sounding procedure has started.
 *
 * @param connHandle - Connection handle.
 * @param procedureCounter - CS procedure counter.
 *
 * @return SUCCESS or error code.
 */
uint8_t AppRREQ_procedureStarted(uint16_t connHandle, uint16_t procedureCounter);

/*********************************************************************
 * RREQ Data Transffer API
 *
 * This module provides APIs to support RREQ (Ranging Request) data transffer.
 *
 * Usage Flow:
 * -----------
 * 1. On the providing node:
 *    - Call RREQ_getConnInfoSize(connHandle) to get the required
 *      buffer size for the data.
 *    - Allocate a pBuffer with the returned size.
 *    - Call RREQ_getConnInfoData(connHandle, pData) to populate the
 *      buffer with the RREQ connection data.
 *    - Transfer the buffer to the Candidate Node via external control
 *      (e.g., proprietary protocol / ble-agent).
 *
 * 2. On the receiving node:
 *    - Receive the data buffer from external control.
 *    - Save the RREQ data from the buffer.
 *    - After the BLE link is established, call RREQ_populateConnInfoData(connHandle, pData, length)
 *      to apply the new data to the actual RREQ table.
 *
 * 3. Finalization on Providing Node:
 *    - After the link is successfully established on the Receiving Node,
 *      call AppRREQ_localDisable(connHandle) on the Providing Node to
 *      disable the RREQ process without notifying the peer, so the peer
 *      would not be aware of the connection transition.
 *
 *********************************************************************/
/*********************************************************************
 * @fn      AppRREQ_localDisable
 *
 * @brief Disable the RREQ process on the providing node side, without notifying the peer.
 *
 * @param connHandle - Connection handle.
 *
 * @return SUCCESS or error code.
 */
uint8_t AppRREQ_localDisable(uint16_t connHandle);

/*********************************************************************
 * @fn      AppRREQ_getConnInfoSize
 *
 * @brief   Returns the size of the RREQ data required from the providing node.
 *          This function calculates the size needed to transfer the RREQ
 *          connection information during a transfer process.
 *
 * @param   connHandle - Connection handle.
 *
 * @return  The size of the RREQ transfer data in bytes if connection is valid.
 * @return  0 if the connection handle is invalid or RREQ is not enabled for this connection.
 */
uint32_t AppRREQ_getConnInfoSize(uint16_t connHandle);

/*********************************************************************
 * @fn      AppRREQ_getConnInfoData
 *
 * @brief   Populates the RREQ data for the providing node into
 *          the provided buffer. This function copies the RREQ connection
 *          information needed.
 *
 * @param   connHandle      - Connection handle.
 * @param   pData   - Pointer to the buffer where the SN data will be stored.
 *                          The buffer must be at least RREQ_getConnInfoSize() bytes.
 *
 * @return  SUCCESS.
 * @return  INVALIDPARAMETER.
 */
uint8_t AppRREQ_getConnInfoData(uint16_t connHandle, uint8_t *pData);

/*********************************************************************
 * @fn      AppRREQ_isAvailableSlot
 *
 * @brief   Check if there is available slots for new connection.
 *
 * input parameters
 * @param   None
 *
 * @return  SUCCESS.
 * @return  FAILURE.
 */
uint8_t AppRREQ_isAvailableSlot(void);

/*********************************************************************
 * @fn      AppRREQ_populateConnInfoData
 *
 * @brief   Apply the provided data to the actual RREQ table.
 *
 * @param   connHandle      - Connection handle.
 * @param   pData           - Pointer to the buffer where the CN RREQ data is stored.
 * @param   length          - Length of the pData.
 *
 * @return  SUCCESS.
 * @return  FAILURE.
 */
uint8_t AppRREQ_populateConnInfoData(uint16_t connHandle, uint8_t* pData, uint32_t length);

#endif // RANGING_CLIENT

#ifdef __cplusplus
}
#endif

#endif /* APP_RANGING_CLIENT_API_H */
