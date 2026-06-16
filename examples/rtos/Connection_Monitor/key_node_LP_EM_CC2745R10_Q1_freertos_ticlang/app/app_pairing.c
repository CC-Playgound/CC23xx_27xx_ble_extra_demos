/******************************************************************************

@file  app_pairing.c

@brief This file contains the application pairing role functionality

  The example provides also APIs to be called from upper layer and support sending events
  to an external control module.

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2026, Texas Instruments Incorporated
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

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include <app_main.h>
#include "app_pairing_api.h"

#if defined(RANGING_CLIENT) && !defined(RANGING_CLIENT_EXTCTRL_APP)
#include "app_ranging_client_api.h"
#endif

//*****************************************************************************
//! Prototypes
//*****************************************************************************

static void Pairing_passcodeHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static void Pairing_pairStateHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
static void Pairing_extEvtHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t*);

//*****************************************************************************
//! Globals
//*****************************************************************************

//! The external event handler
// *****************************************************************//
// ADD YOUR EVENT HANDLER BY CALLING @ref Pairing_registerEvtHandler
//*****************************************************************//
static ExtCtrl_eventHandler_t gExtEvtHandler = NULL;

BLEAppUtil_EventHandler_t pairingPasscodeHandler =
{
  .handlerType    = BLEAPPUTIL_PASSCODE_TYPE,
  .pEventHandler  = Pairing_passcodeHandler
};

BLEAppUtil_EventHandler_t PairingPairStateHandler =
{
  .handlerType    = BLEAPPUTIL_PAIR_STATE_TYPE,
  .pEventHandler  = Pairing_pairStateHandler,
  .eventMask      = BLEAPPUTIL_PAIRING_STATE_STARTED |
                    BLEAPPUTIL_PAIRING_STATE_COMPLETE |
                    BLEAPPUTIL_PAIRING_STATE_ENCRYPTED |
                    BLEAPPUTIL_PAIRING_STATE_BOND_SAVED,
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Pairing_passcodeHandler
 *
 * @brief   The purpose of this function is to handle passcode data
 *          that rise from the GAPBondMgr and were registered
 *          in @ref BLEAppUtil_RegisterGAPEvent
 *
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
static void Pairing_passcodeHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  BLEAppUtil_PasscodeData_t *pData = (BLEAppUtil_PasscodeData_t *)pMsgData;

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pData->connHandle, SUCCESS, B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      Pairing_pairStateHandler
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
static void Pairing_pairStateHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  switch (event)
  {
    case BLEAPPUTIL_PAIRING_STATE_STARTED:
    {

      break;
    }
    case BLEAPPUTIL_PAIRING_STATE_COMPLETE:
    {

      // The pairing is completed, so update the entry in connection list
      // to the ID address instead of the RP address
      linkDBInfo_t linkInfo;
      // Get the list of connected devices
      App_connInfo* connList = Connection_getConnList();
      if (linkDB_GetInfo(((BLEAppUtil_PairStateData_t *)pMsgData)->connHandle, &linkInfo) == SUCCESS)
      {
        // If the peer was using private address, update with ID address
        if ((linkInfo.addrType == ADDRTYPE_PUBLIC_ID ||
             linkInfo.addrType == ADDRTYPE_RANDOM_ID) &&
             !BLEAppUtil_isbufset(linkInfo.addrPriv, 0, B_ADDR_LEN))
        {

          // Get the index of connection list by connHandle
          uint8_t connIdx = Connection_getConnIndex(((BLEAppUtil_PairStateData_t *)pMsgData)->connHandle);

          // Verify that there is a match of connection handle
          if (connIdx != LL_INACTIVE_CONNECTIONS)
          {
            // Update the connection list with the ID address
            memcpy(connList[connIdx].peerAddress, linkInfo.addr, B_ADDR_LEN);
          }
        }
      }
      break;
    }

    case BLEAPPUTIL_PAIRING_STATE_ENCRYPTED:
    case BLEAPPUTIL_PAIRING_STATE_BOND_SAVED:
    {
#if defined(RANGING_CLIENT) && defined(RANGING_CLIENT_MODE) && !defined(RANGING_CLIENT_EXTCTRL_APP)
      // Cast the data
      BLEAppUtil_PairStateData_t *pPairData = (BLEAppUtil_PairStateData_t *)pMsgData;

      // Enable the RREQ process
      AppRREQ_enable(pPairData->connHandle, (RREQEnableModeType_e) RANGING_CLIENT_MODE);
#endif
      break;
    }

    default:
    {
      break;
    }
  }

  // Send the event to the upper layer
  Pairing_extEvtHandler(BLEAPPUTIL_PAIR_STATE_TYPE, event, pMsgData);
}

/*********************************************************************
 * @fn      Pairing_readBond
 *
 * @brief   The purpose of this function is to read a bond information.
 *
 * @param  pReadData - The data that indicates which bond to read
 * @output pBondRecordInfo - The pointer to return the bond record information into
 *
 * @return  SUCCESS if all bond information was read successfully
 *          bleGAPNotFound if there is no bond record
 *          bleInvalidRange if the bond index is out of range or the mode is invalid
 *          bleGAPBondItemNotFound if an item is not found in nv
 */
uint8_t Pairing_readBond(AppPairing_readBond_t *pReadData, AppPairing_BondNvRecord_t *pBondRecordInfo)
{
  uint8_t status = SUCCESS;

  // Fill the bond record structure
  gapBondNvRecord_t bondRecord = {0};
  bondRecord.pBondRec = &pBondRecordInfo->pBondRec;
  bondRecord.pLocalLtk = &pBondRecordInfo->localLtk;
  bondRecord.pDevLtk = &pBondRecordInfo->devLtk;
  bondRecord.pIRK = pBondRecordInfo->pIRK;
  bondRecord.pSRK = pBondRecordInfo->pSRK;
  bondRecord.pSignCount = &pBondRecordInfo->signCount;
  bondRecord.pCharCfg = pBondRecordInfo->charCfg;

  // Read the bond record
  status = GapBondMgr_readBondFromNV(pReadData->mode,
                                     pReadData->pIdentifier,
                                     pReadData->addrType,
                                     &bondRecord);

  return status;
}

/*********************************************************************
 * @fn      Pairing_writeBond
 *
 * @brief   The purpose of this function is to write a bond information
 *
 * @param pWriteBond - The data information to write as a bond record
 *
 * @return SUCCESS if bond was writen successfully or updated if exists
 *         bleNoResources if there are no empty slots
 */
uint8_t Pairing_writeBond(AppPairing_writeBond_t *pWriteBond)
{
  uint8_t status = SUCCESS;

  // Write the bond record
  status = GapBondMgr_writeBondToNv(&pWriteBond->bondRec,
                                    pWriteBond->pLocalLtk,
                                    pWriteBond->pDevLtk,
                                    pWriteBond->pIRK,
                                    pWriteBond->pSRK,
                                    pWriteBond->signCount,
                                    pWriteBond->charCfg);

  return status;
}

/*********************************************************************
 * @fn      Pairing_extEvtHandler
 *
 * @brief   The purpose of this function is to forward the event to the external
 *          event handler that registered to handle the events.
 *
 * @param   eventType - the event type of the event @ref BLEAppUtil_eventHandlerType_e
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  none
 */
static void Pairing_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  // Send the event to the upper layer if its handle exists
  if (gExtEvtHandler != NULL)
  {
    gExtEvtHandler(eventType, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      Pairing_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void Pairing_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler)
{
  gExtEvtHandler = fEventHandler;
}

/*********************************************************************
 * @fn      Pairing_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the pairing
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Pairing_start()
{
  bStatus_t status = SUCCESS;

  // Register the handlers
  status = BLEAppUtil_registerEventHandler(&pairingPasscodeHandler);
  if (status != SUCCESS)
  {
      return(status);
  }

  status = BLEAppUtil_registerEventHandler(&PairingPairStateHandler);
  if (status != SUCCESS)
  {
      // Return status value
      return(status);
  }

  // Return status value
  return(status);
}

#endif // ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )
