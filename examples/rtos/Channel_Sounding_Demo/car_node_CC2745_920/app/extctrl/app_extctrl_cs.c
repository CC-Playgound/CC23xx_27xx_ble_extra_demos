/******************************************************************************

@file  app_extctrl_cs.c

@brief This file parses and processes the messages coming from the external control
 dispatcher module, and build the events from the app_cs.c application and
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
#ifdef CHANNEL_SOUNDING

/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_extctrl_dispatcher.h"
#include <app_extctrl_cs.h>
#include "app_extctrl_common.h"
#include "app_cs_api.h"
#include "string.h"

/*********************************************************************
 * PROTOTYPES
 */
static void CSExtCtrl_sendErrEvt(uint8_t, uint8_t);
bStatus_t readRemoteCapabCmdParser(AppExtCtrl_csReadRemoteCapCmdParams_t *pData);
bStatus_t createConfigCmdParser(AppExtCtrl_csCreateConfigCmdParams_t *pData);
bStatus_t securityEnableCmdParser(AppExtCtrl_csSecurityEnableCmdParams_t *pData);
bStatus_t setDefaultSettingsCmdParser(AppExtCtrl_csSetDefaultSettingsCmdParams_t *pData);
bStatus_t readFAETableCmdParser(AppExtCtrl_csReadRemoteFAETableCmdParams_t *pData);
bStatus_t writeFAETableCmdParser(AppExtCtrl_csWriteRemoteFAETableCmdParams_t *pData);
bStatus_t removeConfigCmdParser(AppExtCtrl_csRemoveConfigCmdParams_t *pData);
bStatus_t setChannelClassCmdParser(AppExtCtrl_csSetChannelClassificationCmdParams_t *pData);
bStatus_t setProcedureParamsCmdParser(AppExtCtrl_csSetProcedureParamsCmdParams_t *pData);
bStatus_t procedureEnableCmdParser(AppExtCtrl_csSetProcedureEnableCmdParams_t *pData);
bStatus_t setDefaultAntennParser(AppExtCtrl_csSetDefaultAntennaCmdParams_t *pData);
static void CSExtCtrl_commandParser(uint8_t*);
static void CsExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t*);
static void CSExtCtrl_extHostEvtHandler(uint8_t*, uint16_t);
static void CsExtCtrl_csEvtHandler(csEvtHdr_t *);
static void CsExtCtrl_csTypeEvtHandler(uint32_t, csEvtHdr_t *);
static void CsExtCtrl_readRemoteCapabEvtHandler(ChannelSounding_readRemoteCapabEvent_t *);
static void CsExtCtrl_configCompleteEvtHandler(ChannelSounding_configComplete_t *);
static void CsExtCtrl_readRemFAETableEvtHandler(ChannelSounding_readRemFAEComplete_t *);
static void CsExtCtrl_securityEnableCompleteEvtHandler(ChannelSounding_securityEnableComplete_t *);
static void CsExtCtrl_procEnableCompleteEvtHandler(ChannelSounding_procEnableComplete_t *);
static void CsExtCtrl_subEventResultsEvtHandler(ChannelSounding_subeventResults_t *);
static void CsExtCtrl_subEventResultsContinueEvtHandler(ChannelSounding_subeventResultsContinue_t *);


/*********************************************************************
 * CONSTANTS
 */
#define CS_CMD_READ_LOCAL_CAP        0x00
#define CS_CMD_READ_REMOTE_CAP       0x01
#define CS_CMD_CREATE_CONFIG         0x02
#define CS_CMD_SECURITY_ENABLE       0x03
#define CS_CMD_SET_DEFAULT_SETTINGS  0x04
#define CS_CMD_READ_FAE_TABLE        0x05
#define CS_CMD_WRITE_FAE_TABLE       0x06
#define CS_CMD_REMOVE_CONFIG         0x07
#define CS_CMD_SET_CHNL_CLASS        0x08
#define CS_CMD_SET_PROCEDURE_PARAMS  0x09
#define CS_CMD_PROCEDURE_ENABLE      0x0A
#define CS_CMD_SET_DEFAULT_ANT       0x0B

/*********************************************************************
 * GLOBALS
 */
// Global variable to store the external control host event handler
static ExtCtrlHost_eventHandler_t gExtHostEvtHandler = NULL;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @brief Parses the command to read remote capabilities.
 *
 * This function processes the command parameters to read the remote capabilities.
 *
 * @param pData Pointer to the structure containing the command parameters.
 *
 * @return Status of the command parsing.
 */
bStatus_t readRemoteCapabCmdParser(AppExtCtrl_csReadRemoteCapCmdParams_t *pData)
{
  ChannelSounding_readRemoteCapCmdParams_t pParams;

  pParams.connHandle = pData->connHandle;

  return ChannelSounding_readRemoteSupportedCapabilities(&pParams);
}

/*********************************************************************
 * @brief Parses the command to create a configuration.
 *
 * This function processes the command parameters to create a configuration.
 *
 * @param pData Pointer to the structure containing the command parameters.
 *
 * @return Status of the command parsing.
 */
bStatus_t createConfigCmdParser(AppExtCtrl_csCreateConfigCmdParams_t *pData)
{
  ChannelSounding_createConfigCmdParams_t pParams;

  pParams.connHandle = pData->connHandle;
  pParams.configID = pData->configID;
  pParams.createContext = pData->createContext;
  pParams.mainMode = pData->mainMode;
  pParams.subMode = pData->subMode;
  pParams.mainModeMinSteps = pData->mainModeMinSteps;
  pParams.mainModeMaxSteps = pData->mainModeMaxSteps;
  pParams.mainModeRepetition = pData->mainModeRepetition;
  pParams.modeZeroSteps = pData->modeZeroSteps;
  pParams.role = pData->role;
  pParams.rttType = pData->rttType;
  pParams.csSyncPhy = pData->csSyncPhy;
  memcpy(&pParams.channelMap, &pData->channelMap, CS_CHM_SIZE);
  pParams.chMRepetition = pData->chMRepetition;
  pParams.chSel = pData->chSel;
  pParams.ch3cShape = pData->ch3cShape;
  pParams.ch3CJump = pData->ch3CJump;

  return ChannelSounding_createConfig(&pParams);
}

/*********************************************************************
 * @brief Parses the security enable command parameters.
 *
 * This function processes the security enable command parameters.
 *
 * @param pData Pointer to the structure containing the security enable
 *              command parameters.
 *
 * @return bStatus_t Status of the command parsing and execution.
 */
bStatus_t securityEnableCmdParser(AppExtCtrl_csSecurityEnableCmdParams_t *pData)
{
  ChannelSounding_securityEnableCmdParams_t pParams;

  pParams.connHandle = pData->connHandle;

  return ChannelSounding_securityEnable(&pParams);
}

/*********************************************************************
 * @brief Parses the command to set default settings.
 *
 * This function processes the command parameters to set the default settings.
 *
 * @param pData Pointer to the structure containing the command parameters.
 *
 * @return Status of the command parsing.
 */
bStatus_t setDefaultSettingsCmdParser(AppExtCtrl_csSetDefaultSettingsCmdParams_t *pData)
{
  ChannelSounding_setDefaultSettingsCmdParams_t pParams;

  pParams.connHandle = pData->connHandle;
  pParams.roleEnable = pData->roleEnable;
  pParams.csSyncAntennaSelection = pData->csSyncAntennaSelection;
  pParams.maxTxPower = pData->maxTxPower;

  return ChannelSounding_setDefaultSettings(&pParams);
}

/*********************************************************************
 * @brief Parses the command to read the remote FAE table.
 *
 * This function processes the command parameters to read the remote FAE table.
 *
 * @param pData Pointer to the structure containing the command parameters.
 *
 * @return Status of the command parsing.
 */
bStatus_t readFAETableCmdParser(AppExtCtrl_csReadRemoteFAETableCmdParams_t *pData)
{
  ChannelSounding_readRemoteFAETableCmdParams_t pParams;

  pParams.connHandle = pData->connHandle;

  return ChannelSounding_readRemoteFAETable(&pParams);
}

/*********************************************************************
 * @brief Parses the command to write the remote FAE table.
 *
 * This function processes the command parameters to write the remote FAE table.
 *
 * @param pData Pointer to the structure containing the command parameters.
 *
 * @return Status of the command parsing.
 */
bStatus_t writeFAETableCmdParser(AppExtCtrl_csWriteRemoteFAETableCmdParams_t *pData)
{
  ChannelSounding_writeCachedRemoteFAETableCmdParams_t pParams;

  pParams.connHandle = pData->connHandle;
  memcpy(&pParams.remoteFaeTable, &pData->remoteFaeTable, CS_FAE_TBL_LEN);

  return ChannelSounding_writeCachedRemoteFAETable(&pParams);
}

/*********************************************************************
 * @brief Parses the command to remove a configuration.
 *
 * This function processes the command parameters to remove a configuration.
 *
 * @param pData Pointer to the structure containing the command parameters.
 *
 * @return Status of the command parsing.
 */
bStatus_t removeConfigCmdParser(AppExtCtrl_csRemoveConfigCmdParams_t *pData)
{
  ChannelSounding_removeConfigCmdParams_t pParams;

  pParams.connHandle = pData->connHandle;
  pParams.configID = pData->configID;

  return ChannelSounding_removeConfig(&pParams);
}

/*********************************************************************
 * @brief Parses the command to set the channel classification.
 *
 * This function processes the command parameters to set the channel classification.
 *
 * @param pData Pointer to the structure containing the command parameters.
 *
 * @return Status of the command parsing.
 */
bStatus_t setChannelClassCmdParser(AppExtCtrl_csSetChannelClassificationCmdParams_t *pData)
{
  ChannelSounding_SetChannelClassificationCmdParams_t pParams;

  memcpy(&pParams.channelClassification, &pData->channelClassification, CS_CHM_SIZE);

  return ChannelSounding_setChannelClassification(&pParams);
}

/*********************************************************************
 * @brief Parses the command to set procedure parameters.
 *
 * This function processes the command parameters to set the procedure parameters.
 *
 * @param pData Pointer to the structure containing the command parameters.
 *
 * @return Status of the command parsing.
 */
bStatus_t setProcedureParamsCmdParser(AppExtCtrl_csSetProcedureParamsCmdParams_t *pData)
{
  ChannelSounding_setProcedureParamsCmdParams_t pParams = {0};

  pParams.connHandle = pData->connHandle;
  pParams.configID = pData->configID;
  pParams.maxProcedureDur = pData->maxProcedureDur;
  pParams.minProcedureInterval = pData->minProcedureInterval;
  pParams.maxProcedureInterval = pData->maxProcedureInterval;
  pParams.maxProcedureCount = pData->maxProcedureCount;
  pParams.minSubEventLen = pData->minSubEventLen;
  pParams.maxSubEventLen = pData->maxSubEventLen;
  pParams.aci = pData->aci;
  pParams.phy = pData->phy;
  pParams.txPwrDelta = pData->txPwrDelta;
  pParams.preferredPeerAntenna = pData->preferredPeerAntenna;
  pParams.snrCtrlI = pData->snrCtrlI;
  pParams.snrCtrlR = pData->snrCtrlR;
  pParams.enable = pData->enable;

  return ChannelSounding_setProcedureParameters(&pParams);
}

/*********************************************************************
 * @brief Parses the command to enable a procedure.
 *
 * This function processes the command parameters to enable a procedure.
 *
 * @param pData Pointer to the structure containing the command parameters.
 *
 * @return Status of the command parsing.
 */
bStatus_t procedureEnableCmdParser(AppExtCtrl_csSetProcedureEnableCmdParams_t *pData)
{
  ChannelSounding_setProcedureEnableCmdParams_t pParams;

  pParams.connHandle = pData->connHandle;
  pParams.configID = pData->configID;
  pParams.enable = pData->enable;

  return ChannelSounding_procedureEnable(&pParams);
}

/*********************************************************************
 * @brief Parses the command to enable a procedure.
 *
 * This function processes the command parameters to set default antenna
 * to be used for common BLE communications.
 *
 * @param pData Pointer to the structure containing the command parameters.
 *
 * @return Status of the command parsing.
 */
bStatus_t setDefaultAntennaParser(AppExtCtrl_csSetDefaultAntennaCmdParams_t *pData)
{
  ChannelSounding_setDefaultAntennaCmdParams_t pParams;

  pParams.defaultAntennaIndex = pData->defaultAntennaIndex;

  return ChannelSounding_setDefaultAntenna(&pParams);
}

/*********************************************************************
 * @fn      CSExtCtrl_commandParser
 *
 * @brief   This function parse the received host message, and call the
 *          the relevant API with the data.
 *
 * @param   pData - pointer to the data message.
 *
 * @return  None
 */
static void CSExtCtrl_commandParser(uint8_t *pData)
{
  if (pData != NULL)
  {
    bStatus_t status = SUCCESS;
    appMsg_t* appMsg = (appMsg_t*)pData;

    switch (appMsg->cmdOp)
    {
      case CS_CMD_READ_LOCAL_CAP:
      {
        status = ChannelSounding_readLocalSupportedCapabilities();
        break;
      }

      case CS_CMD_READ_REMOTE_CAP:
      {
        status = readRemoteCapabCmdParser((AppExtCtrl_csReadRemoteCapCmdParams_t *)appMsg->pData);
        break;
      }

      case CS_CMD_CREATE_CONFIG:
      {
        status = createConfigCmdParser((AppExtCtrl_csCreateConfigCmdParams_t *)appMsg->pData);
        break;
      }

      case CS_CMD_SECURITY_ENABLE:
      {
        status = securityEnableCmdParser((AppExtCtrl_csSecurityEnableCmdParams_t *)appMsg->pData);
        break;
      }

      case CS_CMD_SET_DEFAULT_SETTINGS:
      {
        status = setDefaultSettingsCmdParser((AppExtCtrl_csSetDefaultSettingsCmdParams_t *)appMsg->pData);
        break;
      }

      case CS_CMD_READ_FAE_TABLE:
      {
        status = readFAETableCmdParser((AppExtCtrl_csReadRemoteFAETableCmdParams_t *)appMsg->pData);
        break;
      }

      case CS_CMD_WRITE_FAE_TABLE:
      {
        status = writeFAETableCmdParser((AppExtCtrl_csWriteRemoteFAETableCmdParams_t *)appMsg->pData);
        break;
      }

      case CS_CMD_REMOVE_CONFIG:
      {
        status = removeConfigCmdParser((AppExtCtrl_csRemoveConfigCmdParams_t *)appMsg->pData);
        break;
      }

      case CS_CMD_SET_CHNL_CLASS:
      {
        status = setChannelClassCmdParser((AppExtCtrl_csSetChannelClassificationCmdParams_t *)appMsg->pData);
        break;
      }

      case CS_CMD_SET_PROCEDURE_PARAMS:
      {
        status = setProcedureParamsCmdParser((AppExtCtrl_csSetProcedureParamsCmdParams_t *)appMsg->pData);
        break;
      }

      case CS_CMD_PROCEDURE_ENABLE:
      {
        status = procedureEnableCmdParser((AppExtCtrl_csSetProcedureEnableCmdParams_t *)appMsg->pData);
        break;
      }

      case CS_CMD_SET_DEFAULT_ANT:
      {
          status = setDefaultAntennaParser((AppExtCtrl_csSetDefaultAntennaCmdParams_t *)appMsg->pData);
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
      CSExtCtrl_sendErrEvt(status, appMsg->cmdOp);
    }
  }
}

/*********************************************************************
 * @fn      CsExtCtrl_readRemoteCapabEvtHandler
 *
 * @brief   This function handles the read remote capabilities event.
 *
 * @param   pCsEvt - pointer to the CS notification message
 *
 * @return  None
 */
static void CsExtCtrl_readRemoteCapabEvtHandler(ChannelSounding_readRemoteCapabEvent_t *pCsEvt)
{
  if (pCsEvt != NULL)
  {
    AppExtCtrlCsReadRemoteCapabEvent_t eventCbReport;

    eventCbReport.event             = APP_EXTCTRL_CS_READ_REMOTE_CAPS;
    eventCbReport.connHandle        = pCsEvt->connHandle;
    eventCbReport.csStatus          = pCsEvt->csStatus;
    eventCbReport.numConfig         = pCsEvt->numConfig;
    eventCbReport.maxProcedures     = pCsEvt->maxProcedures;
    eventCbReport.numAntennas       = pCsEvt->numAntennas;
    eventCbReport.maxAntPath        = pCsEvt->maxAntPath;
    eventCbReport.role              = pCsEvt->role;
    eventCbReport.optionalModes     = pCsEvt->optionalModes;
    eventCbReport.rttCap            = pCsEvt->rttCap;
    eventCbReport.rttAAOnlyN        = pCsEvt->rttAAOnlyN;
    eventCbReport.rttSoundingN      = pCsEvt->rttSoundingN;
    eventCbReport.rttRandomPayloadN = pCsEvt->rttRandomPayloadN;
    eventCbReport.nadmSounding      = pCsEvt->nadmSounding;
    eventCbReport.nadmRandomSeq     = pCsEvt->nadmRandomSeq;
    eventCbReport.optionalCsSyncPhy = pCsEvt->optionalCsSyncPhy;
    eventCbReport.noFAE             = pCsEvt->noFAE;
    eventCbReport.chSel3c           = pCsEvt->chSel3c;
    eventCbReport.csBasedRanging    = pCsEvt->csBasedRanging;
    eventCbReport.tIp1Cap           = pCsEvt->tIp1Cap;
    eventCbReport.tIp2Cap           = pCsEvt->tIp2Cap;
    eventCbReport.tFcsCap           = pCsEvt->tFcsCap;
    eventCbReport.tPmCsap           = pCsEvt->tPmCsap;
    eventCbReport.tSwCap            = pCsEvt->tSwCap;
    eventCbReport.snrTxCap          = pCsEvt->snrTxCap;

    // Send the event forward
    CSExtCtrl_extHostEvtHandler((uint8_t *)&eventCbReport,
                                sizeof(AppExtCtrlCsReadRemoteCapabEvent_t));
  }
}

/*********************************************************************
 * @fn      CsExtCtrl_configCompleteEvtHandler
 *
 * @brief   This function handles the config complete event.
 *
 * @param   pCsEvt - pointer to the CS notification message
 *
 * @return  None
 */
static void CsExtCtrl_configCompleteEvtHandler(ChannelSounding_configComplete_t *pCsEvt)
{
  if (pCsEvt != NULL)
  {
    AppExtCtrlCsConfigComplete_t eventCbReport;

    eventCbReport.event              = APP_EXTCTRL_CS_CONFIG_COMPLETE;
    eventCbReport.connHandle         = pCsEvt->connHandle;
    eventCbReport.csStatus           = pCsEvt->csStatus;
    eventCbReport.configId           = pCsEvt->configId;
    eventCbReport.state              = pCsEvt->state;
    eventCbReport.mainMode           = pCsEvt->mainMode;
    eventCbReport.subMode            = pCsEvt->subMode;
    eventCbReport.mainModeMinSteps   = pCsEvt->mainModeMinSteps;
    eventCbReport.mainModeMaxSteps   = pCsEvt->mainModeMaxSteps;
    eventCbReport.mainModeRepetition = pCsEvt->mainModeRepetition;
    eventCbReport.modeZeroSteps      = pCsEvt->modeZeroSteps;
    eventCbReport.role               = pCsEvt->role;
    eventCbReport.rttType            = pCsEvt->rttType;
    eventCbReport.csSyncPhy          = pCsEvt->csSyncPhy;
    eventCbReport.chMRepetition      = pCsEvt->chMRepetition;
    eventCbReport.chSel              = pCsEvt->chSel;
    eventCbReport.ch3cShape          = pCsEvt->ch3cShape;
    eventCbReport.ch3CJump           = pCsEvt->ch3CJump;
    eventCbReport.rfu0               = pCsEvt->rfu0;
    eventCbReport.tIP1               = pCsEvt->tIP1;
    eventCbReport.tIP2               = pCsEvt->tIP2;
    eventCbReport.tFCs               = pCsEvt->tFCs;
    eventCbReport.tPM                = pCsEvt->tPM;
    eventCbReport.rfu                = pCsEvt->rfu;
    memcpy(&eventCbReport.channelMap, &pCsEvt->channelMap, CS_CHM_SIZE);

    // Send the event forward
    CSExtCtrl_extHostEvtHandler((uint8_t *)&eventCbReport,
                                sizeof(AppExtCtrlCsConfigComplete_t));
  }
}

/*********************************************************************
 * @fn      CsExtCtrl_readRemFAETableEvtHandler
 *
 * @brief   This function handles the read remote FAE table complete event.
 *
 * @param   pCsEvt - pointer to the CS notification message
 *
 * @return  None
 */
static void CsExtCtrl_readRemFAETableEvtHandler(ChannelSounding_readRemFAEComplete_t *pCsEvt)
{
  if (pCsEvt != NULL)
  {
    AppExtCtrlCsReadRemFAEComplete_t eventCbReport;

    eventCbReport.event              = APP_EXTCTRL_CS_READ_REMOTE_FAE_TABLE;
    eventCbReport.connHandle         = pCsEvt->connHandle;
    eventCbReport.csStatus           = pCsEvt->csStatus;
    memcpy(&eventCbReport.faeTable, &pCsEvt->faeTable, CS_FAE_TBL_LEN);

    // Send the event forward
    CSExtCtrl_extHostEvtHandler((uint8_t *)&eventCbReport,
                                sizeof(AppExtCtrlCsReadRemFAEComplete_t));
  }
}

/*********************************************************************
 * @fn      CsExtCtrl_securityEnableCompleteEvtHandler
 *
 * @brief   This function handles the security enable complete event.
 *
 * @param   pCsEvt - pointer to the CS notification message
 *
 * @return  None
 */
static void CsExtCtrl_securityEnableCompleteEvtHandler(ChannelSounding_securityEnableComplete_t *pCsEvt)
{
  if (pCsEvt != NULL)
  {
    AppExtCtrlCsSecurityEnableComplete_t eventCbReport;

    eventCbReport.event              = APP_EXTCTRL_CS_SECURITY_ENABLE_COMPLETE;
    eventCbReport.connHandle         = pCsEvt->connHandle;
    eventCbReport.csStatus           = pCsEvt->csStatus;

    // Send the event forward
    CSExtCtrl_extHostEvtHandler((uint8_t *)&eventCbReport,
                                sizeof(AppExtCtrlCsSecurityEnableComplete_t));
  }
}

/*********************************************************************
 * @fn      CsExtCtrl_procEnableCompleteEvtHandler
 *
 * @brief   This function handles the config complete event.
 *
 * @param   pCsEvt - pointer to the CS notification message
 *
 * @return  None
 */
static void CsExtCtrl_procEnableCompleteEvtHandler(ChannelSounding_procEnableComplete_t *pCsEvt)
{
  if (pCsEvt != NULL)
  {
    AppExtCtrlCsProcEnableComplete_t eventCbReport;

    eventCbReport.event                = APP_EXTCTRL_CS_PROCEDURE_ENABLE_COMPLETE;
    eventCbReport.connHandle           = pCsEvt->connHandle;
    eventCbReport.csStatus             = pCsEvt->csStatus;
    eventCbReport.configId             = pCsEvt->configId;
    eventCbReport.enable               = pCsEvt->enable;
    eventCbReport.ACI                  = pCsEvt->ACI;
    eventCbReport.selectedTxPower      = pCsEvt->selectedTxPower;
    eventCbReport.subEventLen          = pCsEvt->subEventLen;
    eventCbReport.subEventsPerEvent    = pCsEvt->subEventsPerEvent;
    eventCbReport.subEventInterval     = pCsEvt->subEventInterval;
    eventCbReport.eventInterval        = pCsEvt->eventInterval;
    eventCbReport.procedureInterval    = pCsEvt->procedureInterval;
    eventCbReport.procedureCount       = pCsEvt->procedureCount;


    // Send the event forward
    CSExtCtrl_extHostEvtHandler((uint8_t *)&eventCbReport,
                                sizeof(AppExtCtrlCsProcEnableComplete_t));
  }
}

/*********************************************************************
 * @fn      CsExtCtrl_subEventResultsEvtHandler
 *
 * @brief   This function handles the subevent results event.
 *
 * @param   pCsEvt - pointer to the CS notification message
 *
 * @return  None
 */
static void CsExtCtrl_subEventResultsEvtHandler(ChannelSounding_subeventResults_t *pCsEvt)
{
  if (pCsEvt != NULL)
  {
    AppExtCtrlCsSubeventResults_t* eventCbReport;

    eventCbReport = (AppExtCtrlCsSubeventResults_t *)
            ICall_malloc(sizeof(AppExtCtrlCsSubeventResults_t) + pCsEvt->dataLen);

    if ( eventCbReport != NULL )
    {
      eventCbReport->event                   = APP_EXTCTRL_CS_SUBEVENT_RESULTS;
      eventCbReport->connHandle              = pCsEvt->connHandle;
      eventCbReport->configID                = pCsEvt->configID;
      eventCbReport->startAclConnectionEvent = pCsEvt->startAclConnectionEvent;
      eventCbReport->procedureCounter        = pCsEvt->procedureCounter;
      eventCbReport->frequencyCompensation   = pCsEvt->frequencyCompensation;
      eventCbReport->referencePowerLevel     = pCsEvt->referencePowerLevel;
      eventCbReport->procedureDoneStatus     = pCsEvt->procedureDoneStatus;
      eventCbReport->subeventDoneStatus      = pCsEvt->subeventDoneStatus;
      eventCbReport->abortReason             = pCsEvt->abortReason;
      eventCbReport->numAntennaPath          = pCsEvt->numAntennaPath;
      eventCbReport->numStepsReported        = pCsEvt->numStepsReported;
      eventCbReport->dataLen                 = pCsEvt->dataLen;

      memcpy(eventCbReport->data, pCsEvt->data, pCsEvt->dataLen);

      // Send the event forward
      CSExtCtrl_extHostEvtHandler((uint8_t *) eventCbReport,
                                  sizeof(AppExtCtrlCsSubeventResults_t) + pCsEvt->dataLen);
      ICall_free(eventCbReport);
    }
  }
}

/*********************************************************************
 * @fn      CsExtCtrl_subEventResultsContinueEvtHandler
 *
 * @brief   This function handles the subevent results continue event.
 *
 * @param   pCsEvt - pointer to the CS notification message
 *
 * @return  None
 */
static void CsExtCtrl_subEventResultsContinueEvtHandler(ChannelSounding_subeventResultsContinue_t *pCsEvt)
{
  if ( pCsEvt != NULL )
  {
    AppExtCtrlCsSubeventResultsContinue_t* eventCbReport;

    eventCbReport = (AppExtCtrlCsSubeventResultsContinue_t *)
            ICall_malloc(sizeof(AppExtCtrlCsSubeventResultsContinue_t) + pCsEvt->dataLen);

    if ( eventCbReport != NULL )
    {
      eventCbReport->event                   = APP_EXTCTRL_CS_SUBEVENT_RESULTS_CONTINUE;
      eventCbReport->connHandle              = pCsEvt->connHandle;
      eventCbReport->configID                = pCsEvt->configID;
      eventCbReport->procedureDoneStatus     = pCsEvt->procedureDoneStatus;
      eventCbReport->subeventDoneStatus      = pCsEvt->subeventDoneStatus;
      eventCbReport->abortReason             = pCsEvt->abortReason;
      eventCbReport->numAntennaPath          = pCsEvt->numAntennaPath;
      eventCbReport->numStepsReported        = pCsEvt->numStepsReported;
      eventCbReport->dataLen                 = pCsEvt->dataLen;

      memcpy(eventCbReport->data, pCsEvt->data, pCsEvt->dataLen);

      // Send the event forward
      CSExtCtrl_extHostEvtHandler((uint8_t *) eventCbReport,
                                  sizeof(AppExtCtrlCsSubeventResultsContinue_t) + pCsEvt->dataLen);
      ICall_free(eventCbReport);
    }
  }
}

/*********************************************************************
 * @fn      CSExtCtrl_sendErrEvt
 *
 * @brief   This function sends the error event to the external control.
 *
 * @param   errCode - the error code
 * @param   cmdOp   - the command op-code that raised the error
 *
 * @return  None
 */
static void CSExtCtrl_sendErrEvt(uint8_t errCode, uint8_t cmdOp)
{
  // Create error event structure
  AppExtCtrlErrorEvent_t errEvt;
  errEvt.event        = APP_EXTCTRL_FAILURE;
  errEvt.cmdOp        = cmdOp;
  errEvt.appSpecifier = APP_SPECIFIER_CS;
  errEvt.errCode      = errCode;

  // Send error event
  CSExtCtrl_extHostEvtHandler((uint8_t *)&errEvt, sizeof(AppExtCtrlErrorEvent_t));
}

/*********************************************************************
 * @fn      CsExtCtrl_csEvtHandler
 *
 * @brief   This function handles @ref BLEAPPUTIL_CS_EVENT_CODE
 *
 * @param   pMsgData - pointer message data
 *
 * @return  None
 */
static void CsExtCtrl_csEvtHandler(csEvtHdr_t *pMsgData)
{
  uint8_t opcode = pMsgData->opcode;

  switch( opcode )
  {
    case CS_READ_REMOTE_SUPPORTED_CAPABILITIES_COMPLETE_EVENT:
    {
      CsExtCtrl_readRemoteCapabEvtHandler((ChannelSounding_readRemoteCapabEvent_t *) pMsgData);
      break;
    }

    case CS_CONFIG_COMPLETE_EVENT:
    {
      CsExtCtrl_configCompleteEvtHandler((ChannelSounding_configComplete_t *) pMsgData);
      break;
    }

    case CS_READ_REMOTE_FAE_TABLE_COMPLETE_EVENT:
    {
      CsExtCtrl_readRemFAETableEvtHandler((ChannelSounding_readRemFAEComplete_t *) pMsgData);
      break;
    }

    case CS_SECURITY_ENABLE_COMPLETE_EVENT:
    {
      CsExtCtrl_securityEnableCompleteEvtHandler((ChannelSounding_securityEnableComplete_t *) pMsgData);
      break;
    }

    case CS_PROCEDURE_ENABLE_COMPLETE_EVENT:
    {
      CsExtCtrl_procEnableCompleteEvtHandler((ChannelSounding_procEnableComplete_t *) pMsgData);
      break;
    }

    case CS_SUBEVENT_RESULT:
    {
      CsExtCtrl_subEventResultsEvtHandler((ChannelSounding_subeventResults_t *) pMsgData);
      break;
    }

    case CS_SUBEVENT_CONTINUE_RESULT:
    {
      CsExtCtrl_subEventResultsContinueEvtHandler((ChannelSounding_subeventResultsContinue_t *) pMsgData);
      break;
    }

    default:
    {
      break;
    }
  }
}

/*********************************************************************
 * @fn      CsExtCtrl_csAppDistanceResultsEvtHandler
 *
 * @brief   This function handles the distance results event.
 *
 * @param   pCsEvt - pointer to the CS notification message
 *
 * @return  None
 */
static void CsExtCtrl_csAppDistanceResultsEvtHandler(ChannelSounding_appDistanceResultsEvent_t *pCsEvt)
{
  if (pCsEvt != NULL)
  {
    AppExtCtrlCsAppDistanceResultsEvent_t eventCbReport;

    eventCbReport.event             = APP_EXTCTRL_CS_APP_DISTANCE_RESULTS;
    eventCbReport.status            = pCsEvt->status;
    eventCbReport.connHandle        = pCsEvt->connHandle;
    eventCbReport.distance          = pCsEvt->distance;
    eventCbReport.quality           = pCsEvt->quality;
    eventCbReport.confidence        = pCsEvt->confidence;

    // Send the event forward
    CSExtCtrl_extHostEvtHandler((uint8_t *)&eventCbReport,
                                sizeof(AppExtCtrlCsAppDistanceResultsEvent_t));
  }
}

/*********************************************************************
 * @fn      CsExtCtrl_csAppDistanceExtendedResultsEvtHandler
 *
 * @brief   This function handles the distance debug results event.
 *
 * @param   pCsEvt - pointer to the CS notification message
 *
 * @return  None
 */
static void CsExtCtrl_csAppDistanceExtendedResultsEvtHandler(ChannelSounding_appExtendedResultsEvent_t *pCsEvt)
{
    if (pCsEvt != NULL)
    {
        AppExtCtrlCsAppExtendedResultsEvent_t* eventCbReport =
                (AppExtCtrlCsAppExtendedResultsEvent_t*) ICall_malloc(sizeof(AppExtCtrlCsAppExtendedResultsEvent_t));

        if (NULL != eventCbReport)
        {
            eventCbReport->event      = APP_EXTCTRL_CS_APP_DISTANCE_EXTENDED_RESULTS;
            eventCbReport->status     = pCsEvt->status;
            eventCbReport->connHandle = pCsEvt->connHandle;
            eventCbReport->distance   = pCsEvt->distance;
            eventCbReport->quality    = pCsEvt->quality;
            eventCbReport->confidence = pCsEvt->confidence;
            eventCbReport->numMpc     = pCsEvt->numMpc;

            memcpy(eventCbReport->distanceMusic,          pCsEvt->distanceMusic,          sizeof(pCsEvt->distanceMusic));
            memcpy(eventCbReport->distanceNN,             pCsEvt->distanceNN,             sizeof(pCsEvt->distanceNN));
            memcpy(eventCbReport->numMpcPaths,            pCsEvt->numMpcPaths,            sizeof(pCsEvt->numMpcPaths));
            memcpy(eventCbReport->qualityPaths,           pCsEvt->qualityPaths,           sizeof(pCsEvt->qualityPaths));
            memcpy(eventCbReport->confidencePaths,        pCsEvt->confidencePaths,        sizeof(pCsEvt->confidencePaths));
            memcpy(eventCbReport->localRpl,               pCsEvt->localRpl,               sizeof(pCsEvt->localRpl));
            memcpy(eventCbReport->remoteRpl,              pCsEvt->remoteRpl,              sizeof(pCsEvt->remoteRpl));
            memcpy(eventCbReport->modeZeroStepsInit,      pCsEvt->modeZeroStepsInit,      sizeof(pCsEvt->modeZeroStepsInit));
            memcpy(eventCbReport->modeZeroStepsRef,       pCsEvt->modeZeroStepsRef,       sizeof(pCsEvt->modeZeroStepsRef));
            memcpy(eventCbReport->permutationIndexLocal,  pCsEvt->permutationIndexLocal,  sizeof(pCsEvt->permutationIndexLocal));
            memcpy(eventCbReport->stepsDataLocal,         pCsEvt->stepsDataLocal,         sizeof(pCsEvt->stepsDataLocal));
            memcpy(eventCbReport->permutationIndexRemote, pCsEvt->permutationIndexRemote, sizeof(pCsEvt->permutationIndexRemote));
            memcpy(eventCbReport->stepsDataRemote,        pCsEvt->stepsDataRemote,        sizeof(pCsEvt->stepsDataRemote));

            // Send the event forward
            CSExtCtrl_extHostEvtHandler((uint8_t *)eventCbReport, sizeof(AppExtCtrlCsAppExtendedResultsEvent_t));

            ICall_free(eventCbReport);
        }
    }
}

/*********************************************************************
 * @fn      CsExtCtrl_csAppEvtHandler
 *
 * @brief   This function handles @ref BLEAPPUTIL_CS_APP_EVENT_CODE
 *
 * @param   pMsgData - pointer message data
 *
 * @return  None
 */
static void CsExtCtrl_csAppEvtHandler(csEvtHdr_t *pMsgData)
{
  uint8_t opcode = pMsgData->opcode;

  switch( opcode )
  {
    case CS_APP_DISTANCE_RESULTS_EVENT:
    {
      CsExtCtrl_csAppDistanceResultsEvtHandler((ChannelSounding_appDistanceResultsEvent_t *) pMsgData);
      break;
    }
    case CS_APP_DISTANCE_EXTENDED_RESULTS_EVENT:
    {
      CsExtCtrl_csAppDistanceExtendedResultsEvtHandler((ChannelSounding_appExtendedResultsEvent_t *) pMsgData);
      break;
    }
    default:
    {
      break;
    }
  }
}

/*********************************************************************
 * @fn      CSExtCtrl_hciTypeEvtHandler
 *
 * @brief   This function handles HCI Le and Command Complete events.
 *
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void CsExtCtrl_csTypeEvtHandler(uint32_t event, csEvtHdr_t *pMsgData)
{
  switch ( event )
  {
    case BLEAPPUTIL_CS_EVENT_CODE:
    {
      CsExtCtrl_csEvtHandler(pMsgData);
      break;
    }
    case BLEAPPUTIL_CS_APP_EVENT_CODE:
    {
      CsExtCtrl_csAppEvtHandler(pMsgData);
      break;
    }

    default:
      break;
  }
}

/*********************************************************************
 * @fn      CsExtCtrl_eventHandler
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
static void CsExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if ( pMsgData != NULL )
  {
    switch ( eventType )
    {
      case BLEAPPUTIL_CS_TYPE:
      {
         CsExtCtrl_csTypeEvtHandler(event, (csEvtHdr_t *)pMsgData);
         break;
      }

      default :
        break;
    }
  }
}

/*********************************************************************
 * @fn      CSExtCtrl_extHostEvtHandler
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
static void CSExtCtrl_extHostEvtHandler(uint8_t *pData, uint16_t dataLen)
{
  if ( gExtHostEvtHandler != NULL && pData != NULL )
  {
    gExtHostEvtHandler(pData, dataLen);
  }
}

/*********************************************************************
 * @fn      CSExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the cs module
 *          to the external control dispatcher, and register the call back
 *          event handler function using the given registration function.
 *
 * @param   fRegisterFun - A registration function to be used by extCtrl in order
 *                         to register its CS event handler function.
 *
 * @return  SUCCESS/FAILURE
 */
bStatus_t CSExtCtrl_start(ExtCtrl_eventHandlerRegister_t fRegisterFun)
{
  bStatus_t status = SUCCESS;

  // Register to the Dispatcher module and get the event handler
  gExtHostEvtHandler = Dispatcher_registerMsgHandler(APP_SPECIFIER_CS,
                                                     &CSExtCtrl_commandParser,
                                                     APP_CAP_CS);

  // If the registration succeed, register event handler call back to the cs application
  if ( gExtHostEvtHandler != NULL )
  {
      fRegisterFun(&CsExtCtrl_eventHandler);
  }

  return status;
}

#endif // CHANNEL_SOUNDING
