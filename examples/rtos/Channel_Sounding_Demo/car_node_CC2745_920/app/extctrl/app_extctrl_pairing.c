/******************************************************************************

@file  app_extctrl_pairing.c

@brief This file parse and process the messages comes form the external control
 dispatcher module, and build the events from the app_pairing.c application and
 send it to the external control dispatcher module back.

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2024-2026, Texas Instruments Incorporated
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


/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_extctrl_dispatcher.h"
#include "app_extctrl_pairing.h"
#include <string.h>

/*********************************************************************
 * PROTOTYPES
 */
static void PairingExtCtrl_sendErrEvt(uint8_t, uint8_t);
static void PairingExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t*);
static void PairingExtCtrl_commandParser(uint8_t *pData);
static void PairingExtCtrl_extHostEvtHandler(uint8_t*, uint16_t);
static uint8_t PairingExtCtrl_readBondCmdParser(AppExtCtrlPairingReadBondCmd_t *pData);
static uint8_t PairingExtCtrl_writeBondCmdParser(AppExtCtrlPairingWriteBondCmd_t *pData);
static uint8_t PairingExtCtrl_setOobEnable(AppExtCtrlPairingSetOobEnableCmd_t *pData);
static uint8_t PairingExtCtrl_getLocalOobData(void);
static uint8_t PairingExtCtrl_generateEccKeys(void);
static uint8_t PairingExtCtrl_setRemoteOobData(AppExtCtrlPairingSetRemoteOobDataCmd_t *pData);
static void PairingExtCtrl_maxNumCharCfgEvt(uint8_t maxNumCharCfg);
static void PairingExtCtrl_readBondEvt(gapBondRec_t *pBondRec,
                                       gapBondLTK_t *pLocalLtk,
                                       gapBondLTK_t *pDevLtk,
                                       uint8_t *pIRK,
                                       uint8_t *pSRK,
                                       uint32_t signCount,
                                       gapBondCharCfg_t *charCfg);

/*********************************************************************
 * CONSTANTS
 */
#define PAIRING_APP_GET_MAX_NUM_CHAR_CFG   0x00
#define PAIRING_APP_READ_BOND              0x01
#define PAIRING_APP_WRITE_BOND             0x02
#define PAIRING_APP_SET_OOB_ENABLE         0x03
#define PAIRING_APP_SET_REMOTE_OOB_DATA    0x04
#define PAIRING_APP_GET_LOCAL_OOB_DATA     0x05
#define PAIRING_APP_GENERATE_ECC_KEYS      0x06

/*********************************************************************
 * GLOBALS
 */
// The external control host event handler
static ExtCtrlHost_eventHandler_t gExtHostEvtHandler = NULL;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      PairingExtCtrl_commandParser
 *
 * @brief   This function parse the received host message, and call the
 *          the relevant API with the data.
 *
 * @param   pData - pointer to the data message.
 *
 * @return  None
 */
static void PairingExtCtrl_commandParser(uint8_t *pData)
{
  if (pData != NULL)
  {
    appMsg_t* appMsg = (appMsg_t*)pData;
    bStatus_t status = SUCCESS;

    switch (appMsg->cmdOp)
    {
      case PAIRING_APP_GET_MAX_NUM_CHAR_CFG:
      {
        // Send the event
        PairingExtCtrl_maxNumCharCfgEvt(GAP_CHAR_CFG_MAX);
        break;
      }

      case PAIRING_APP_READ_BOND:
      {
        // Read the bond record
        status = PairingExtCtrl_readBondCmdParser((AppExtCtrlPairingReadBondCmd_t *)appMsg->pData);
        break;
      }

      case PAIRING_APP_WRITE_BOND:
      {
        // Read the command data
        status = PairingExtCtrl_writeBondCmdParser((AppExtCtrlPairingWriteBondCmd_t *)appMsg->pData);
        break;
      }

      case PAIRING_APP_SET_OOB_ENABLE:
      {
        // Set the OOB enable
        status = PairingExtCtrl_setOobEnable((AppExtCtrlPairingSetOobEnableCmd_t *)appMsg->pData);
        break;
      }

      case PAIRING_APP_SET_REMOTE_OOB_DATA:
      {
        // Set the remote OOB data
        status = PairingExtCtrl_setRemoteOobData((AppExtCtrlPairingSetRemoteOobDataCmd_t *)appMsg->pData);
        break;
      }

      case PAIRING_APP_GET_LOCAL_OOB_DATA:
      {
        // Get the local OOB data
        status = PairingExtCtrl_getLocalOobData();
        break;
      }

      case PAIRING_APP_GENERATE_ECC_KEYS:
      {
        // Generate ECC keys
        status = PairingExtCtrl_generateEccKeys();
        break;
      }

      default:
      {
        status = FAILURE;
        break;
      }
    }

    if (status != SUCCESS)
    {
      // Send error message
        PairingExtCtrl_sendErrEvt(status, appMsg->cmdOp);
    }
  }
}

/*********************************************************************
 * @fn      PairingExtCtrl_sendErrEvt
 *
 * @brief   This function sends the error event to the external control.
 *
 * @param   errCode - the error code
 * @param   cmdOp   - the command op-code that raised the error
 *
 * @return  None
 */
static void PairingExtCtrl_sendErrEvt(uint8_t errCode, uint8_t cmdOp)
{
  // Create error event structure
  AppExtCtrlErrorEvent_t errEvt;
  errEvt.event        = APP_EXTCTRL_FAILURE;
  errEvt.cmdOp        = cmdOp;
  errEvt.appSpecifier = APP_SPECIFIER_PAIRING;
  errEvt.errCode      = errCode;

  // Send error event
  PairingExtCtrl_extHostEvtHandler((uint8_t *)&errEvt, sizeof(AppExtCtrlErrorEvent_t));
}

/*********************************************************************
 * @fn      PairingExtCtrl_readBondCmdParser
 *
 * @brief Parses the command to read a bond information.
 *
 * This function processes the command parameters to read a bond
 * and calls the read bond information API
 *
 * @param pData - The data information for the read bond command.
 *
 * @return  None
 */
uint8_t PairingExtCtrl_readBondCmdParser(AppExtCtrlPairingReadBondCmd_t *pData)
{
  uint8_t status = SUCCESS;
  AppPairing_readBond_t readData;

  // Allocate memory for the bond record
  AppPairing_BondNvRecord_t *pPairingAppBondRecord = (AppPairing_BondNvRecord_t *)ICall_malloc(sizeof(AppPairing_BondNvRecord_t));
  gapBondCharCfg_t *pCharCfg = (gapBondCharCfg_t *)ICall_malloc(sizeof(gapBondCharCfg_t) * GAP_CHAR_CFG_MAX);;

  // If the memory allocation succeed read the bond record
  if (pPairingAppBondRecord != NULL)
  {
    // Fill the bond read data information
    readData.mode = pData->mode;
    memcpy(readData.pIdentifier, pData->pIdentifier, B_ADDR_LEN);
    readData.addrType = pData->addrType;

    // Clear the memory
    memset(pPairingAppBondRecord, 0, sizeof(AppPairing_BondNvRecord_t));
    memset(pCharCfg, 0, sizeof(gapBondCharCfg_t)*GAP_CHAR_CFG_MAX);
    // Point to the characteristic configuration pointer
    pPairingAppBondRecord->charCfg = pCharCfg;

    // Read the bond record
    status = Pairing_readBond(&readData, pPairingAppBondRecord);

    // If the bond read succeed send the bond record to the external control
    if ((status == SUCCESS) || (status == bleGAPBondItemNotFound))
    {
      PairingExtCtrl_readBondEvt(&pPairingAppBondRecord->pBondRec,
                                &pPairingAppBondRecord->localLtk,
                                &pPairingAppBondRecord->devLtk,
                                pPairingAppBondRecord->pIRK,
                                pPairingAppBondRecord->pSRK,
                                pPairingAppBondRecord->signCount,
                                pPairingAppBondRecord->charCfg);
    }

    ICall_free(pPairingAppBondRecord);
    pPairingAppBondRecord = NULL;

  }
  else
  {
    status = bleMemAllocError;
  }

  return status;
}

/*********************************************************************
 * @fn      PairingExtCtrl_writeBondCmdParser
 *
 * @brief Parses the command to write a bond information.
 *
 * This function processes the command parameters to write a bond
 * and calls the write bond information API
 *
 * @param pData - The data information for the write bond command.
 *
 * @return  None
 */
static uint8_t PairingExtCtrl_writeBondCmdParser(AppExtCtrlPairingWriteBondCmd_t *pData)
{
  uint8_t status = SUCCESS;
  gapBondLTK_t *pLocalLtk = NULL;
  gapBondLTK_t *pDevLtk = NULL;
  uint8_t *pSRK = NULL;
  AppPairing_writeBond_t writeData;

  // Check if the local LTK structure is empty
  if ((BLEAppUtil_isbufset((uint8_t*)pData->bondInfo.localLTK, 0, KEYLEN) == FALSE) ||
      (BLEAppUtil_isbufset((uint8_t*)pData->bondInfo.localRand, 0, KEYLEN) == FALSE) ||
      (pData->bondInfo.localDiv != 0) ||
      (pData->bondInfo.localKeySize != 0))
  {
    // Allocate memory for the local LTK since it is not empty
    pLocalLtk = (gapBondLTK_t *)ICall_malloc(sizeof(gapBondLTK_t));
    if (pLocalLtk != NULL)
    {
      // Copy the local LTK
      memcpy(pLocalLtk->LTK, pData->bondInfo.localLTK, KEYLEN);
      pLocalLtk->div = pData->bondInfo.localDiv;
      memcpy(pLocalLtk->rand, pData->bondInfo.localRand, B_RANDOM_NUM_SIZE);
      pLocalLtk->keySize = pData->bondInfo.localKeySize;
    }
    else
    {
      // Memory allocation failed
      status = bleMemAllocError;
    }
  }

  // Check if the memory allocation succeed or the local LTK is empty
  if (status == SUCCESS)
  {
    // Check if the peer LTK is empty
  if ((BLEAppUtil_isbufset((uint8_t*)pData->bondInfo.peerLTK, 0, KEYLEN) == FALSE) ||
      (BLEAppUtil_isbufset((uint8_t*)pData->bondInfo.peerRand, 0, KEYLEN) == FALSE) ||
      (pData->bondInfo.peerDiv != 0) ||
      (pData->bondInfo.peerKeySize != 0))
    {
      // Allocate memory for the peer LTK since it is not empty
      pDevLtk = (gapBondLTK_t *)ICall_malloc(sizeof(gapBondLTK_t));
      if (pDevLtk != NULL)
      {
        // Copy the peer LTK
        memcpy(pDevLtk->LTK, pData->bondInfo.peerLTK, KEYLEN);
        pDevLtk->div = pData->bondInfo.peerDiv;
        memcpy(pDevLtk->rand, pData->bondInfo.peerRand, B_RANDOM_NUM_SIZE);
        pDevLtk->keySize = pData->bondInfo.peerKeySize;
      }
      else
      {
        // Memory allocation failed
        status = bleMemAllocError;

        // Free the local LTK memory
        if (pLocalLtk != NULL)
        {
          ICall_free(pLocalLtk);
          pLocalLtk = NULL;
        }
      }
    }

    // Check if the memory allocation succeed or the peer LTK is empty
    if (status == SUCCESS)
    {
      // Check if the SRK is empty
      if (BLEAppUtil_isbufset((uint8_t*)pData->bondInfo.pSRK, 0, KEYLEN) == FALSE)
      {
        // SRK is not empty, point to the SRK data
        pSRK = pData->bondInfo.pSRK;
      }

      // Copy the bond record data
      memcpy(writeData.bondRec.addr, pData->bondInfo.peerAddr, B_ADDR_LEN);
      writeData.bondRec.addrType = pData->bondInfo.peerAddrType;
      writeData.bondRec.stateFlags = pData->bondInfo.stateFlags;
      writeData.pLocalLtk = pLocalLtk;
      writeData.pDevLtk = pDevLtk;
      writeData.pIRK = pData->bondInfo.pIRK;
      writeData.pSRK = pSRK;
      writeData.signCount = pData->bondInfo.signCount;

      // Copy the characteristic configuration data
      for (uint8_t i = 0; i < GAP_CHAR_CFG_MAX; i++)
      {
        writeData.charCfg[i].attrHandle = pData->bondInfo.charCfg[i].attrHandle;
        writeData.charCfg[i].value = pData->bondInfo.charCfg[i].value;
      }

      // Write the bond record
      status = Pairing_writeBond(&writeData);

      // Free the local LTK memory
      if (pLocalLtk != NULL)
      {
        ICall_free(pLocalLtk);
        pLocalLtk = NULL;
      }

      // Free the peer LTK memory
      if (pDevLtk != NULL)
      {
        ICall_free(pDevLtk);
        pDevLtk = NULL;
      }
    }
  }

  return status;
}

/*********************************************************************
 * @fn      PairingExtCtrl_setOobEnable
 *
 * @brief  This function sets the OOB enable/disable using the GAPBondMgr API.
 *
 * @param pData - The data information for the set OOB enable command
 *                (TRUE - use, FALSE - do not use)
 *
 * @return  None
 */
static uint8_t PairingExtCtrl_setOobEnable(AppExtCtrlPairingSetOobEnableCmd_t *pData)
{
  uint8_t status = SUCCESS;
  uint8_t OOBEnable = pData->enable;
  // Set the OOB enable
  status = GAPBondMgr_SetParameter(GAPBOND_OOB_ENABLED,
                                   sizeof(uint8_t),
                                   (void*)(&OOBEnable));

  return status;
}

/*********************************************************************
 * @fn      PairingExtCtrl_setRemoteOobData
 *
 * @brief  This function sets the remote device OOB data
 *         using the GAPBondMgr API.
 *
 * @param pData - The data information for the set OOB data command.
 *
 * @return  None
 */
static uint8_t PairingExtCtrl_setRemoteOobData(AppExtCtrlPairingSetRemoteOobDataCmd_t *pData)
{
  uint8_t status = SUCCESS;
  gapBondOOBData_t oobData;

  // Copy the OOB data
  memcpy(oobData.confirm, pData->oobConfirm, KEYLEN);
  memcpy(oobData.rand, pData->oobRand, KEYLEN);

  // Set Remote OOB data
  status = GAPBondMgr_SCSetRemoteOOBParameters(&oobData, TRUE);
  return status;
}

/*********************************************************************
 * @fn      PairingExtCtrl_getLocalOobData
 *
 * @brief  This function gets the local OOB data using the GAPBondMgr API.
 *
 * @param None
 *
 * @return  None
 */
static uint8_t PairingExtCtrl_getLocalOobData(void)
{
  uint8_t status = SUCCESS;
  gapBondOOBData_t retBuf;
  AppExtCtrlPairingGetLocalOobDataEvent_t localOobDataEvt;

  // Get local OOB data
  status = GAPBondMgr_SCGetLocalOOBParameters(&retBuf);
  if (status == SUCCESS)
  {
    // Send the local OOB data to the external control
    localOobDataEvt.event = APP_EXTCTRL_PAIRING_GET_LOCAL_OOB_DATA;
    memcpy(localOobDataEvt.confirm, retBuf.confirm, KEYLEN);
    memcpy(localOobDataEvt.rand, retBuf.rand, KEYLEN);
    PairingExtCtrl_extHostEvtHandler((uint8_t *)&localOobDataEvt, sizeof(AppExtCtrlPairingGetLocalOobDataEvent_t));
  }
  return status;
}

/*********************************************************************
 * @fn      PairingExtCtrl_generateEccKeys
 *
 * @brief   This function generates ECC keys using the GAPBondMgr API.
 *
 * @return  Status of the key generation operation.
 */
static uint8_t PairingExtCtrl_generateEccKeys(void)
{
  uint8_t status = SUCCESS;
  status = GAPBondMgr_GenerateEccKeys();
  return status;
}

/*********************************************************************
 * @fn      PairingExtCtrl_maxNumCharCfgEvt
 *
 * @brief   This function sends the max num char cfg value as event to
 *          the external control.
 *
 * @param maxNumCharCfg - The max number of char cfg entries value
 *
 * @return  None
 */
static void PairingExtCtrl_maxNumCharCfgEvt(uint8_t maxNumCharCfg)
{
  AppExtCtrlPairingMaxNumCharCfgEvent_t maxNumCharCfgEvt;

  // Fill the max num char cfg event structure
  maxNumCharCfgEvt.event = APP_EXTCTRL_PAIRING_MAX_NUM_CHAR_CFG;
  maxNumCharCfgEvt.maxNumCharCfg = maxNumCharCfg;

  PairingExtCtrl_extHostEvtHandler((uint8_t *)&maxNumCharCfgEvt, sizeof(AppExtCtrlPairingMaxNumCharCfgEvent_t));
}

/*********************************************************************
 * @fn      PairingExtCtrl_readBondEvt
 *
 * @brief   This function sends the bond information received from
 *          the gapBondMgr read bond cmd as event to the external control.
 *
 * @param   pBondRec - pointer to the bond record
 * @param   pLocalLtk - pointer to the local LTK
 * @param   pDevLtk - pointer to the peer LTK
 * @param   pIRK - pointer to the IRK
 * @param   pSRK - pointer to the SRK
 * @param   signCount - the sign counter value
 * @param   charCfg - pointer to the characteristic configuration
 *
 * @return  None
 */
static void PairingExtCtrl_readBondEvt(gapBondRec_t *pBondRec,
                                      gapBondLTK_t *pLocalLtk,
                                      gapBondLTK_t *pDevLtk,
                                      uint8_t *pIRK,
                                      uint8_t *pSRK,
                                      uint32_t signCount,
                                      gapBondCharCfg_t *charCfg)
{
  // Create the bond read event structure
  AppExtCtrlPairingReadBondEvent_t readBondEvt;

  // Fill the bond read event structure
  readBondEvt.event = APP_EXTCTRL_PAIRING_READ_BOND;

  // Copy the bond record data
  memcpy(readBondEvt.bondInfo.peerAddr, pBondRec->addr, B_ADDR_LEN);
  readBondEvt.bondInfo.peerAddrType = pBondRec->addrType;
  readBondEvt.bondInfo.stateFlags = pBondRec->stateFlags;

  // Copy the local LTK data
  memcpy(readBondEvt.bondInfo.localLTK, pLocalLtk->LTK, KEYLEN);
  readBondEvt.bondInfo.localDiv = pLocalLtk->div;
  memcpy(readBondEvt.bondInfo.localRand, pLocalLtk->rand, B_RANDOM_NUM_SIZE);
  readBondEvt.bondInfo.localKeySize = pLocalLtk->keySize;

  // Copy the peer LTK data
  memcpy(readBondEvt.bondInfo.peerLTK, pDevLtk->LTK, KEYLEN);
  readBondEvt.bondInfo.peerDiv = pDevLtk->div;
  memcpy(readBondEvt.bondInfo.peerRand, pDevLtk->rand, B_RANDOM_NUM_SIZE);
  readBondEvt.bondInfo.peerKeySize = pDevLtk->keySize;

  // Copy the IRK and SRK data
  memcpy(readBondEvt.bondInfo.pIRK, pIRK, KEYLEN);
  memcpy(readBondEvt.bondInfo.pSRK, pSRK, KEYLEN);

  // Copy the sign counter value
  readBondEvt.bondInfo.signCount = signCount;

  // Copy the characteristic configuration data
  for (uint8_t i = 0; i < GAP_CHAR_CFG_MAX; i++)
  {
    readBondEvt.bondInfo.charCfg[i].attrHandle = charCfg[i].attrHandle;
    readBondEvt.bondInfo.charCfg[i].value = charCfg[i].value;
  }

  PairingExtCtrl_extHostEvtHandler((uint8_t *)&readBondEvt, sizeof(AppExtCtrlPairingReadBondEvent_t));
}

/*********************************************************************
 * @fn      PairingExtCtrl_pairingStateEvtHandler
 *
 * @brief   This function handles the Pairing State event.
 *
 * @param   pStateMsg    - pointer to the pairing state message
 * @param   extCtrlEvent - the event that should be sent to the external control
 *
 * @return  None
 */
static void PairingExtCtrl_pairingStateEvtHandler(BLEAppUtil_PairStateData_t *pStateMsg, AppExtCtrlEvent_e extCtrlEvent)
{
  if (pStateMsg != NULL)
  {
    AppExtCtrlPairingStateEvent_t pairingStateEvt;

    pairingStateEvt.event      = extCtrlEvent;
    pairingStateEvt.connHandle = pStateMsg->connHandle;
    pairingStateEvt.state      = pStateMsg->state;
    pairingStateEvt.status     = pStateMsg->status;

    PairingExtCtrl_extHostEvtHandler((uint8_t *)&pairingStateEvt, sizeof(AppExtCtrlPairingStateEvent_t));
  }
}

/*********************************************************************
 * @fn      PairingExtCtrl_PairStateTypeEvtHandler
 *
 * @brief   This function handles events from type @ref BLEAPPUTIL_PAIR_STATE_TYPE
 *
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void PairingExtCtrl_pairingStateTypeEvtHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    switch (event)
    {
      case BLEAPPUTIL_PAIRING_STATE_STARTED:
      {
        PairingExtCtrl_pairingStateEvtHandler((BLEAppUtil_PairStateData_t *)pMsgData,
                                              APP_EXTCTRL_PAIRING_STATE_STARTED);
        break;
      }
      case BLEAPPUTIL_PAIRING_STATE_COMPLETE:
      {
        PairingExtCtrl_pairingStateEvtHandler((BLEAppUtil_PairStateData_t *)pMsgData,
                                              APP_EXTCTRL_PAIRING_STATE_COMPLETE);
        break;
      }

      case BLEAPPUTIL_PAIRING_STATE_ENCRYPTED:
      {
        PairingExtCtrl_pairingStateEvtHandler((BLEAppUtil_PairStateData_t *)pMsgData,
                                              APP_EXTCTRL_PAIRING_STATE_ENCRYPTED);
        break;
      }

      case BLEAPPUTIL_PAIRING_STATE_BOND_SAVED:
      {
        PairingExtCtrl_pairingStateEvtHandler((BLEAppUtil_PairStateData_t *)pMsgData,
                                              APP_EXTCTRL_PAIRING_STATE_BOND_SAVED);
        break;
      }


      default:
      {
        break;
      }

    }
  }
}
/*********************************************************************
 * @fn      PairingExtCtrl_eventHandler
 *
 * @brief   This function handles the events raised from the application that
 * this module registered to.
 *
 * @param   eventType - the type of the events @ref BLEAppUtil_eventHandlerType_e.
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void PairingExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    switch (eventType)
    {
      case BLEAPPUTIL_PAIR_STATE_TYPE:
      {
        PairingExtCtrl_pairingStateTypeEvtHandler(event, pMsgData);
      }

      default :
      {
        break;
      }
    }
  }
}

/*********************************************************************
 * @fn      PairingExtCtrl_extHostEvtHandler
 *
 * @brief   The purpose of this function is to forward the event to the external
 *          control host event handler that the external control returned once
 *          this module registered to it.
 *
 * @param   pData    - pointer to the event data
 * @param   dataLen  - data length.
 *
 * @return  None
 */
static void PairingExtCtrl_extHostEvtHandler(uint8_t *pData, uint16_t dataLen)
{
  if (gExtHostEvtHandler != NULL && pData != NULL)
  {
    gExtHostEvtHandler(pData, dataLen);
  }
}

/*********************************************************************
 * @fn      PairingExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the pairing module
 *          to the external control dispatcher, and register the call back
 *          event handler function to the pairing application.
 *
 * @return  SUCCESS/FAILURE
 */
bStatus_t PairingExtCtrl_start(void)
{
  bStatus_t status = SUCCESS;

  // Register to the Dispatcher module
  gExtHostEvtHandler = Dispatcher_registerMsgHandler(APP_SPECIFIER_PAIRING,
                                                     &PairingExtCtrl_commandParser,
                                                     APP_CAP_PAIRING);

  // If the registration succeed, register event handler call back to the pairing application
  if (gExtHostEvtHandler != NULL)
  {
    Pairing_registerEvtHandler(&PairingExtCtrl_eventHandler);
  }
  return status;
}
