/******************************************************************************

@file  app_ca_client.c

@brief This file contains the Car Access service client application
       functionality

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

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include "ti/ble/app_util/common/util.h"

#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "ti/ble/app_util/framework/bleapputil_timers.h"
#include <app_main.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

// Car Access Service UUID
#define CA_CLIENT_APP_SERVICE_UUID         0xA0A0
// Indication Characteristic UUID
#define CA_CLIENT_APP_INDICATION_CHAR_UUID 0xA0A1
// Data In Characteristic UUID
#define CA_CLIENT_APP_DATA_IN_CHAR_UUID    0xA0A2
// Pairing Characteristic UUID
#define CA_CLIENT_APP_PAIRING_CHAR_UUID    0xA0A3

// 16-bit high and low index in the 128-bit custom UUID
#define CA_CLIENT_APP_HIGH_UUID_INDEX      0x0C
#define CA_CLIENT_APP_LOW_UUID_INDEX       0x0D

// Indication CCCD handle offset
#define CA_CLIENT_APP_INDICATION_CCCD_OFFSET    0x01
// Indication CCCD value length
#define CA_CLIENT_APP_INDICATION_CCCD_VALUE_LEN 0x02

// The index of characteristic value handle
#define CA_CLIENT_APP_CHAR_VALUE_HANDLE_INDEX 0x03
// The index of characteristic UUID handle
#define CA_CLIENT_APP_CHAR_UUID_HANDLE_INDEX  0x05

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

// Retryable function type definition
// This type of function can be invoked with a context pointer that can be casted to a specific type.
typedef bStatus_t (*RetryableFunction)(void *);

// @ref CAClient_invokeWithTimer function parameters
typedef struct
{
    uint32_t timeInMS;      //!< time in milliseconds to wait before invoking the function when re-scheduled.
    RetryableFunction func; //!< function to invoke.
    void *funcContext;      //!< function parameters. Casted to a specific type inside the function.
} invokeWithTimerParams_t;

// @ref CAClient_discoverPrimServ function parameters
typedef struct
{
    uint16_t connHandle;    //!< Connection handle to use for the command
} discPrimServParams_t;

// @ref CAClient_readPairingCharValue function parameters
typedef struct
{
    uint16_t connHandle;    //!< Connection handle to use for the command
} readPairingCharValueParams_t;

//*****************************************************************************
//! Globals
//Characteristic indication handle
static uint16_t gIndCharHandle = 0;

// Characteristic pairing handle
static uint16_t gPairingCharHandle = 0;

//*****************************************************************************
//*****************************************************************************
//!LOCAL FUNCTIONS
//*****************************************************************************
static bStatus_t CAClient_discoverAllChars(uint16_t connHandle, uint16_t startHandle, uint16_t endHandle);
static uint8_t CAClient_discoverPrimServ(void* pParams);
static bStatus_t CAClient_enabledIndication(uint16_t connHandle, uint8_t enable);
static bStatus_t CAClient_readPairingCharValue(void* pParams);
static void CAClient_readPairingChar(uint16_t connHandle);
static bStatus_t CAClient_invokeWithTimer(invokeWithTimerParams_t *pParams);
static void CAClient_timerCB(BLEAppUtil_timerHandle timerHandle, BLEAppUtil_timerTermReason_e reason, void *pData);

//*****************************************************************************
//!LOCAL VARIABLES
//*****************************************************************************

// Handles of the Car Access characteristics
static uint16_t gCaStartHandle = 0;
static uint16_t gCaEndHandle = 0;

//*****************************************************************************
//!APPLICATION CALLBACK
//*****************************************************************************
static void CAClient_GATTEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static void CAClient_PairStateEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

// Events handlers struct, contains the handlers and event masks
// of the application data module
BLEAppUtil_EventHandler_t caClientGATTHandler =
{
    .handlerType    = BLEAPPUTIL_GATT_TYPE,
    .pEventHandler  = CAClient_GATTEventHandler,
    .eventMask      = BLEAPPUTIL_ATT_MTU_UPDATED_EVENT        |
                      BLEAPPUTIL_ATT_FIND_BY_TYPE_VALUE_RSP   |
                      BLEAPPUTIL_ATT_READ_BY_TYPE_RSP         |
                      BLEAPPUTIL_ATT_HANDLE_VALUE_IND         |
                      BLEAPPUTIL_ATT_READ_RSP                 |
                      BLEAPPUTIL_ATT_ERROR_RSP
};

BLEAppUtil_EventHandler_t caClientPairStateHandler =
{
    .handlerType    = BLEAPPUTIL_PAIR_STATE_TYPE,
    .pEventHandler  = CAClient_PairStateEventHandler,
                      BLEAPPUTIL_PAIRING_STATE_ENCRYPTED |
                      BLEAPPUTIL_PAIRING_STATE_BOND_SAVED
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      CAClient_GATTEventHandler
 *
 * @brief   The purpose of this function is to handle GATT events
 *          that rise from the GATT and were registered in
 *          @ref BLEAppUtil_RegisterGAPEvent
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
static void CAClient_GATTEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  gattMsgEvent_t *gattMsg = ( gattMsgEvent_t * )pMsgData;

  switch ( event )
  {
    case BLEAPPUTIL_ATT_MTU_UPDATED_EVENT:
    {
        // Build timer callback parameters
        invokeWithTimerParams_t pParams;
        
        // Get the connection information
        linkDBInfo_t connInfo = {0};
        uint8_t status = linkDB_GetInfo(gattMsg->connHandle, &connInfo);

        // Check if the connection is valid
        if ( (status != bleTimeout) && (status != bleNotConnected))
        {
            // Set the retryable function and timer
            pParams.timeInMS = connInfo.connInterval;
            pParams.func = (RetryableFunction)CAClient_discoverPrimServ;

            // Allocate the invoked function context. Will be freed by @ref CAClient_invokeWithTimer
            pParams.funcContext = (void *)BLEAppUtil_malloc(sizeof(discPrimServParams_t));
            if (pParams.funcContext != NULL)
            {
                // Set connection handle
                ((discPrimServParams_t*)pParams.funcContext)->connHandle = gattMsg->connHandle;

                // Call the function with a timer
                CAClient_invokeWithTimer(&pParams);
            }
        }
    }
    break;

    case BLEAPPUTIL_ATT_FIND_BY_TYPE_VALUE_RSP:
    {
        if( gattMsg->hdr.status == SUCCESS )
        {
            gCaStartHandle = ATT_ATTR_HANDLE(gattMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
            gCaEndHandle = ATT_GRP_END_HANDLE(gattMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        }

        else if( gattMsg->hdr.status == bleProcedureComplete )
        {
            // Find all characteristics within the Car Access service
            CAClient_discoverAllChars(gattMsg->connHandle, gCaStartHandle, gCaEndHandle);
        }
    }
    break;

    case BLEAPPUTIL_ATT_READ_BY_TYPE_RSP:
    {
        attReadByTypeRsp_t att = gattMsg->msg.readByTypeRsp;

        if ( ( gattMsg->hdr.status == SUCCESS ) && ( att.numPairs > 0 ) )
        {
            for (uint8_t i = 0; i < att.numPairs; i++)
            {
                // The format of the data in att.pDataList for each characteristic is:
                // Properties handle: 2 bytes
                // Properties value: 1 byte
                // Char value handle: 2 bytes
                // Char value UUID: 16 bytes
                // Since the handle and value of the properties are not used in this case,
                // they are skipped for each characteristic
                uint16_t charUUID = 0;
                uint8_t customUUID[ATT_UUID_SIZE];
                uint8_t index = i * att.len;

                // ATT handle
                uint16_t currAttHandle = BUILD_UINT16(att.pDataList[index + CA_CLIENT_APP_CHAR_VALUE_HANDLE_INDEX],
                                                      att.pDataList[index + CA_CLIENT_APP_CHAR_VALUE_HANDLE_INDEX + 1]);

                // Copy the full 128-bit custom UUID
                memcpy(customUUID, &att.pDataList[index + CA_CLIENT_APP_CHAR_UUID_HANDLE_INDEX], ATT_UUID_SIZE);

                // Build the 16-bit UUID of the characteristic
                charUUID = BUILD_UINT16(customUUID[CA_CLIENT_APP_HIGH_UUID_INDEX],
                                        customUUID[CA_CLIENT_APP_LOW_UUID_INDEX]);

                // Check if this is the indication UUID
                if ( charUUID == CA_CLIENT_APP_INDICATION_CHAR_UUID )
                {
                    // Save the handle of the indication characteristic
                    gIndCharHandle = currAttHandle;
                }

                // Check if this is the indication UUID
                if ( charUUID == CA_CLIENT_APP_PAIRING_CHAR_UUID )
                {
                    // Save the handle of the pairing characteristic
                    gPairingCharHandle = currAttHandle;
                }
            }
        }
        else if( gattMsg->hdr.status == bleProcedureComplete )
        {
            uint8_t status = SUCCESS;
            linkDBInfo_t linkInfo = { 0 };

            // Get the link information
            status = linkDB_GetInfo(gattMsg->connHandle, &linkInfo);

            // If the link is not encrypted read the pairing characteristic value
            // to trigger pairing
            if ( ( status == SUCCESS ) && !( linkInfo.stateFlags & LINK_ENCRYPTED ) )
            {
                // Read the pairing characteristic value to trigger pairing
                CAClient_readPairingChar(gattMsg->connHandle);
            }
        }

        break;
    }

    case BLEAPPUTIL_ATT_HANDLE_VALUE_IND:
    {
        // If the handle is the Car Access Indication characteristic handle
        if( gattMsg->msg.handleValueInd.handle == gIndCharHandle )
        {
            // Send an indication confirmation
            ATT_HandleValueCfm(gattMsg->connHandle);
        }

        break;
    }

    case BLEAPPUTIL_ATT_READ_RSP:
    {
        // Enable indications
        CAClient_enabledIndication(gattMsg->connHandle, true);
        break;
    }

    case BLEAPPUTIL_ATT_ERROR_RSP:
    {
        if( gattMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ )
        {
            // If the handle is the Car Access Pairing characteristic handle
            if ( ( gattMsg->msg.errorRsp.handle == gPairingCharHandle ) &&
                 ( ( gattMsg->msg.errorRsp.errCode == ATT_ERR_INSUFFICIENT_AUTHEN ) ||
                   ( gattMsg->msg.errorRsp.errCode == ATT_ERR_INSUFFICIENT_ENCRYPT ) ) )
            {
                uint8_t status = SUCCESS;
                linkDBInfo_t linkInfo = { 0 };
                // Get the link information
                status = linkDB_GetInfo(gattMsg->connHandle, &linkInfo);

                if ( status == SUCCESS )
                {
                    if ( linkInfo.connRole == GAP_PROFILE_CENTRAL )
                    {
                        // For Central role - start encryption,
                        // it will start if it is bonded
                        status = GapBondMgr_StartEnc(gattMsg->connHandle);
                    }

                    // If the role is Peripheral or if the role is Central and
                    // is not bonded (the GapBondMgr_StartEnc() return value != SUCCESS)
                    if ( ( linkInfo.connRole == GAP_PROFILE_PERIPHERAL ) ||
                         ( ( linkInfo.connRole == GAP_PROFILE_CENTRAL ) && ( status != SUCCESS ) ) )
                    {
                        // Start pairing
                        GAPBondMgr_Pair(gattMsg->connHandle);
                    }
                }
            }
        }

        break;
    }

    default:
      break;
  }
}

/*********************************************************************
 * @fn      CAClient_PairStateEventHandler
 *
 * @brief   The purpose of this function is to handle pairing state
 *          events that rise from the GAPBondMgr and were registered
 *          in @ref BLEAppUtil_RegisterGAPEvent
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
static void CAClient_PairStateEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch (event)
    {
        case BLEAPPUTIL_PAIRING_STATE_ENCRYPTED:
        case BLEAPPUTIL_PAIRING_STATE_BOND_SAVED:
        {
            // Read the pairing characteristic value after pairing/encryption
            CAClient_readPairingChar(((BLEAppUtil_PairStateData_t *)pMsgData)->connHandle);

            break;
        }

        default:
        {
            break;
        }
    }
}

/*********************************************************************
 * @fn      CAClient_discoverPrimServ
 *
 * @brief   Car Access client discover primary service by UUID
 *
 * @param   pParams - pointer to function parameters. Casted to @ref discPrimServParams_t.
 *
 * @return  SUCCESS or stack call status
 */
static uint8_t CAClient_discoverPrimServ(void* pParams)
{
    uint8_t status = INVALIDPARAMETER;

    if (pParams != NULL)
    {
        discPrimServParams_t* pDiscPrimServParams = (discPrimServParams_t*) pParams;

        // Discovery Car Access service
        uint8_t caUUID[ATT_UUID_SIZE] = {TI_BASE_UUID_128(CA_CLIENT_APP_SERVICE_UUID)};
        status = GATT_DiscPrimaryServiceByUUID(pDiscPrimServParams->connHandle, caUUID,
                                               ATT_UUID_SIZE, BLEAppUtil_getSelfEntity());
    }

    return status;
}

/*********************************************************************
 * @fn      CAClient_discoverAllChars
 *
 * @brief   Car Access client discover all service characteristics
 *
 * @param   connHandle - connection message was received on
 *
 * @return  SUCCESS or stack call status
 */
static bStatus_t CAClient_discoverAllChars(uint16_t connHandle, uint16_t startHandle, uint16_t endHandle)
{
    bStatus_t status;

    // Discovery simple service
    status = GATT_DiscAllChars(connHandle, startHandle, endHandle, BLEAppUtil_getSelfEntity());

    return status;
}

/*********************************************************************
 * @fn      CAClient_enabledIndication
 *
 * @brief   Car Access client enabled indication item
 *
 * @param   connHandle - connection message was received on
 * @param   enable - enable (true) or disable (false) indications
 *
 * @return  SUCCESS or stack call status
 */
static bStatus_t CAClient_enabledIndication(uint16_t connHandle, uint8_t enable)
{
    bStatus_t status;
    attWriteReq_t req;
    // Set the default value to disable indications
    uint16_t dataValue = 0;

    // Set the value to enable indications
    if (enable)
    {
        dataValue = GATT_CLIENT_CFG_INDICATE;
    }

    if ( gIndCharHandle != 0 )
    {
        // Allocate buffer for the write request
        req.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, CA_CLIENT_APP_INDICATION_CCCD_VALUE_LEN, NULL);

        // Send the write request for indications enable/diable
        if (req.pValue != NULL)
        {
            req.handle = gIndCharHandle + CA_CLIENT_APP_INDICATION_CCCD_OFFSET;
            req.len = CA_CLIENT_APP_INDICATION_CCCD_VALUE_LEN;
            memcpy(req.pValue, &dataValue, CA_CLIENT_APP_INDICATION_CCCD_VALUE_LEN);
            req.cmd = TRUE;
            req.sig = FALSE;
            status = GATT_WriteNoRsp(connHandle, &req);
            // If the write request failed, free the buffer
            if ( status != SUCCESS )
            {
                GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
            }
        }
        else
        {
            status = bleMemAllocError;
        }
    }
    else
    {
        status = bleIncorrectMode;
    }

    return status;
}

/*********************************************************************
 * @fn      CAClient_readPairingCharValue
 *
 * @brief   Car Access client read pairing characteristic value,
 *          this function should be called after the characteristic
 *          handle was discovered
 *
 * @param   pParams - pointer to function parameters. Casted to @ref readPairingCharValueParams_t.
 *
 * @return  SUCCESS or stack call status
 */
static bStatus_t CAClient_readPairingCharValue(void* pParams)
{
    bStatus_t status = INVALIDPARAMETER;
    attReadReq_t req = {0};
    readPairingCharValueParams_t* pReadParams;

    if (pParams != NULL)
    {
        status = SUCCESS;
        pReadParams = (readPairingCharValueParams_t*) pParams;

        if ( gPairingCharHandle != 0 )
        {
            // If the pairing characteristic handle is not discovered, return error
            req.handle = gPairingCharHandle;

            // Read the pairing characteristic value to trigger pairing
            status = GATT_ReadCharValue(pReadParams->connHandle, &req, BLEAppUtil_getSelfEntity());
        }
        else
        {
            // If the pairing characteristic handle is not discovered, return error
            status = bleIncorrectMode;
        }
    }

    return status;
}

/*********************************************************************
 * @fn      CAClient_readPairingChar
 *
 * @brief   Car Access client read pairing characteristic, this
 *          function calls @ref CAClient_invokeWithTimer
 *          in order to call @ref CAClient_readPairingCharValue.
 *          Note that this function should be called after the
 *          characteristic handle was discovered
 *
 * @param   pData - the address of the connection handle
 *
 * @return  SUCCESS or stack call status
 */
static void CAClient_readPairingChar(uint16_t connHandle)
{
    // Timer parameters
    invokeWithTimerParams_t pParams;
    
    // Get the connection information
    linkDBInfo_t connInfo = {0};
    uint8_t status = linkDB_GetInfo(connHandle, &connInfo);

    // Check if the connection is valid
    if ( (status != bleTimeout) && (status != bleNotConnected))
    {
        // Set the retryable function and timer
        pParams.timeInMS = connInfo.connInterval;
        pParams.func = (RetryableFunction)CAClient_readPairingCharValue;

        // Allocate the invoked function context. Will be freed by @ref CAClient_invokeWithTimer
        pParams.funcContext = (void *)BLEAppUtil_malloc(sizeof(readPairingCharValueParams_t));
        if (pParams.funcContext != NULL)
        {
            // Set connection handle
            ((readPairingCharValueParams_t*)pParams.funcContext)->connHandle = connHandle;

            // Call the function with a timer
            CAClient_invokeWithTimer(&pParams);
        }
    }
}

/*********************************************************************
 * @fn      CAClient_invokeWithTimer
 *
 * @brief   The purpose of this function is to invoke specific function
 *          and re-invoke it with a timer if it returns @ref blePending.
 *
 * @param   pParams - Pointer to function parameters.
 *                    This function won't free this pointer.
 *
 * @return  INVALIDPARAMETER - In case of NULL pointer
 * @return  bleMemAllocError - In case of memory allocation failure
 * @return  status of the invoked function otherwise
 */
static bStatus_t CAClient_invokeWithTimer(invokeWithTimerParams_t *pParams)
{
    bStatus_t status = INVALIDPARAMETER; // status to be returned

    if (pParams != NULL)
    {
        // Invoke the function with the given context
        status = pParams->func(pParams->funcContext);

        // Re-schedule the function if the status is blePending
        if (status == blePending)
        {
            // Allocate memory for the re-schedule parameters
            // Note: This memory will be freed by @ref CAClient_timerCB
            invokeWithTimerParams_t *pReScheduleParams = (invokeWithTimerParams_t*) BLEAppUtil_malloc(sizeof(invokeWithTimerParams_t));

            if (pReScheduleParams == NULL)
            {
                status = bleMemAllocError;
            }
            else
            {
                // Set the time to invoke the function again
                pReScheduleParams->timeInMS = pParams->timeInMS;

                // Set the function to invoke and its context.
                // Note: The context will be freed here once the invoked function will return non blePending status
                pReScheduleParams->func = pParams->func;
                pReScheduleParams->funcContext = pParams->funcContext;

                BLEAppUtil_startTimer(CAClient_timerCB, pReScheduleParams->timeInMS, false, (void*) pReScheduleParams);
            }
        }
        else if (pParams->funcContext != NULL)
        {
            // Free the function context
            // Note: This is done only if the function was invoked successfully
            // and not re-scheduled.
            // If it was re-scheduled, the context will be freed on the next successfull run.
            BLEAppUtil_free(pParams->funcContext);
        }
    }

    return status;
}

/*********************************************************************
 * @fn      CAClient_timerCB
 *
 * @brief   General timer callback function of type @ref BLEAppUtil_timerCB_t.
 *          This function is called when the associated timer expires.
 *          It calls @ref CAClient_invokeWithTimer function which will
 *          invoke the function that was passed to it.
 *
 * input parameters
 *
 * @param   timerHandle - The handle of the timer that expired.
 * @param   reason - The reason for the timer expiration.
 * @param   pData - Pointer to the data associated with the timer.
 *                  Casted to @ref invokeWithTimerParams_t.
 *                  Freed by this function after the function is invoked.
 *
 * output parameters
 *
 * @param   None
 *
 * @return None
 */
static void CAClient_timerCB(BLEAppUtil_timerHandle timerHandle, BLEAppUtil_timerTermReason_e reason, void *pData)
{
    // Cast the data to the expected type
    invokeWithTimerParams_t* pParams = (invokeWithTimerParams_t*) pData;

    if (pParams != NULL)
    {
        if (reason == BLEAPPUTIL_TIMER_TIMEOUT)
        {
            // If the timer expired, invoke the function with the given parameters again,
            // while giving the option to re-schedule it with a timer
            CAClient_invokeWithTimer(pParams);
        }
        else if (pParams->funcContext != NULL)
        {
            // Free the context data if the timer was aborted
            BLEAppUtil_free(pParams->funcContext);
        }

        // Free the given data
        BLEAppUtil_free(pParams);
    }
}

/*********************************************************************
 * @fn      CAClient_Start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to discover all the UUIDs of
 *          Car Access service.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t CAClient_Start( void )
{
    bStatus_t status = SUCCESS;

    // Register the GATT event handler
    status = BLEAppUtil_registerEventHandler( &caClientGATTHandler );
    if ( status == SUCCESS )
    {
        // Register the Pairing state event handler
        status = BLEAppUtil_registerEventHandler( &caClientPairStateHandler );
    }

    return status;
}
