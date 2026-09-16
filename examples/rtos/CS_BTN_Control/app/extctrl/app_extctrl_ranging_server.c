/******************************************************************************

@file  app_extctrl_ranging_server.c

@brief This file implements external control commands for the ranging server.

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

#if defined(RANGING_SERVER_EXTCTRL_APP) && defined(RANGING_SERVER)

/*********************************************************************
 * INCLUDES
 */
#include "app_extctrl_ranging_server.h"
#include "app_extctrl_common.h"
#include "app_ranging_server_api.h"
#include "ti/ble/app_util/framework/bleapputil_extctrl_dispatcher.h"
#include "app_cs_api.h"
#include <string.h>

/*********************************************************************
 * COMMAND OPCODES
 */
#define APP_RRSP_CMD_SEND_CS_ENABLE_EVENT       0x01
#define APP_RRSP_CMD_SEND_CS_EVENT              0x02
#define APP_RRSP_CMD_SEND_CS_EVENT_CONT         0x03
/*********************************************************************
 * GLOBALS
 */
// The external control host event handler
static ExtCtrlHost_eventHandler_t gExtHostEvtHandler = NULL;

/*********************************************************************
 * INTERNAL FUNCTIONS
 */
static void AppRRSPExtCtrl_sendEvt(uint8_t status, uint8_t cmdOp, AppExtCtrlEvent_e eventOp);
static void AppRRSPExtCtrl_sendErrEvt(uint8_t errCode, uint8_t cmdOp);
static void AppRRSPExtCtrl_extHostEvtHandler(uint8_t *pData, uint16_t dataLen);
static void AppRRSPExtCtrl_commandParser(uint8_t *pData);
static uint8_t AppRRSPExtCtrl_SendCsProcedureEnable(uint16_t connHandle);
static uint8_t AppRRSPExtCtrl_SendCsSubEvent(const AppExtCtrlRRSPSendCsEvtCmd_t *pData);
static uint8_t AppRRSPExtCtrl_SendCsSubEventCont(const AppExtCtrlRRSPSendCsEvtContCmd_t *pData);

/*********************************************************************
 * @fn      AppRRSPExtCtrl_commandParser
 *
 * @brief   Parse and process incoming external control commands.
 *
 * @param   pData - pointer to the command data.
 *
 * @return  None
 */
static void AppRRSPExtCtrl_commandParser(uint8_t *pData)
{
  if (pData != NULL)
  {
    appMsg_t *appMsg = (appMsg_t*)pData;
    bStatus_t status = FAILURE;

    switch (appMsg->cmdOp)
    {
        case APP_RRSP_CMD_SEND_CS_ENABLE_EVENT:
        {
            AppExtCtrlRRSPEnableCmd_t *cmd = (AppExtCtrlRRSPEnableCmd_t *)(appMsg->pData);
            status = AppRRSPExtCtrl_SendCsProcedureEnable(cmd->connHandle);
            // Send event back to the external control
            AppRRSPExtCtrl_sendEvt(status, appMsg->cmdOp, APP_EXTCTRL_RRSP_SEND_CS_ENABLE_EVENT);
            break;
        }
        case APP_RRSP_CMD_SEND_CS_EVENT:
        {
            AppExtCtrlRRSPSendCsEvtCmd_t *cmd = (AppExtCtrlRRSPSendCsEvtCmd_t *)(appMsg->pData);
            status = AppRRSPExtCtrl_SendCsSubEvent(cmd);
            // Send event back to the external control
            AppRRSPExtCtrl_sendEvt(status, appMsg->cmdOp, APP_EXTCTRL_RRSP_SEND_CS_EVENT);
            break;
        }
        case APP_RRSP_CMD_SEND_CS_EVENT_CONT:
        {
            AppExtCtrlRRSPSendCsEvtContCmd_t *cmd = (AppExtCtrlRRSPSendCsEvtContCmd_t *)(appMsg->pData);
            status = AppRRSPExtCtrl_SendCsSubEventCont(cmd);
            // Send event back to the external control
            AppRRSPExtCtrl_sendEvt(status, appMsg->cmdOp, APP_EXTCTRL_RRSP_SEND_CS_EVENT_CONT);
            break;
        }
        default:
            // Send error message
            AppRRSPExtCtrl_sendErrEvt(status, appMsg->cmdOp);
            break;
    }
  }
}

/*********************************************************************
 * @fn      AppRRSPExtCtrl_sendEvt
 *
 * @brief   This function sends the event to the external control.
 *
 * @param   status - the status code
 * @param   cmdOp   - the command op-code that raised the event
 * @param   eventOp - the event op-code to be sent
 *
 * @return  None
 */
static void AppRRSPExtCtrl_sendEvt(uint8_t status, uint8_t cmdOp, AppExtCtrlEvent_e eventOp)
{
  // Create event structure
  AppExtCtrlRRSPStatus_t evt;
  evt.event    = eventOp;
  evt.cmdOp    = cmdOp;
  evt.status   = status;

  // Send event
  AppRRSPExtCtrl_extHostEvtHandler((uint8_t *)&evt, sizeof(AppExtCtrlRRSPStatus_t));
}

/*********************************************************************
 * @fn      AppRREQExtCtrl_sendErrEvt
 *
 * @brief   This function sends the error event to the external control.
 *
 * @param   errCode - the error code
 * @param   cmdOp   - the command op-code that raised the error
 *
 * @return  None
 */
static void AppRRSPExtCtrl_sendErrEvt(uint8_t errCode, uint8_t cmdOp)
{
  // Create error event structure
  AppExtCtrlErrorEvent_t errEvt;
  errEvt.event        = APP_EXTCTRL_FAILURE;
  errEvt.cmdOp        = cmdOp;
  errEvt.appSpecifier = APP_SPECIFIER_RRSP;
  errEvt.errCode      = errCode;

  // Send error event
  AppRRSPExtCtrl_extHostEvtHandler((uint8_t *)&errEvt, sizeof(AppExtCtrlErrorEvent_t));
}

/*********************************************************************
 * @fn      AppRRSPExtCtrl_extHostEvtHandler
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
static void AppRRSPExtCtrl_extHostEvtHandler(uint8_t *pData, uint16_t dataLen)
{
  if ((gExtHostEvtHandler != NULL) && (pData != NULL))
  {
    gExtHostEvtHandler(pData, dataLen);
  }
}

/*********************************************************************
 * @fn      AppRRSPExtCtrl_SendCsProcedureEnable
 *
 * @brief   Handle the CS enable event.
 *
 * @param   connHandle - connection handle.
 *
 * @return  status - SUCCESS or FAILURE
 */
static uint8_t AppRRSPExtCtrl_SendCsProcedureEnable(uint16_t connHandle)
{
    ChannelSounding_procEnableComplete_t enableCompleteEvent = {0};
    uint8_t status;

    // Set the event parameters
    enableCompleteEvent.csEvtOpcode = APP_CS_PROCEDURE_ENABLE_COMPLETE_EVENT;
    enableCompleteEvent.csStatus = CS_STATUS_SUCCESS;
    enableCompleteEvent.connHandle = connHandle;
    enableCompleteEvent.ACI = ACI_A1_B1;

    // Send the procedure enable complete event to the RAS profile
    status = AppRRSP_CsProcedureEnable(&enableCompleteEvent);

    return status;
}

/*********************************************************************
 * @fn      AppRRSPExtCtrl_SendCsSubEvent
 *
 * @brief   Handle the CS event.
 *
 * @param   pData    - pointer to the event data.
 *
 * @return  status - SUCCESS or FAILURE
 */
static uint8_t AppRRSPExtCtrl_SendCsSubEvent(const AppExtCtrlRRSPSendCsEvtCmd_t *pData)
{
    ChannelSounding_subeventResults_t *pSubeventResults;
    uint8_t status = FAILURE;
    pSubeventResults = (ChannelSounding_subeventResults_t *)ICall_malloc(sizeof(ChannelSounding_subeventResults_t) + pData->dataLen);

    if (pSubeventResults != NULL)
    {
        pSubeventResults->csEvtOpcode             = pData->csEvtOpcode;
        pSubeventResults->connHandle              = pData->connHandle;
        pSubeventResults->configID                = pData->configID;
        pSubeventResults->startAclConnectionEvent = pData->startAclConnectionEvent;
        pSubeventResults->procedureCounter        = pData->procedureCounter;
        pSubeventResults->frequencyCompensation   = pData->frequencyCompensation;
        pSubeventResults->referencePowerLevel     = pData->referencePowerLevel;
        pSubeventResults->procedureDoneStatus     = pData->procedureDoneStatus;
        pSubeventResults->subeventDoneStatus      = pData->subeventDoneStatus;
        pSubeventResults->abortReason             = pData->abortReason;
        pSubeventResults->numAntennaPath          = pData->numAntennaPath;
        pSubeventResults->numStepsReported        = pData->numStepsReported;
        pSubeventResults->dataLen                 = pData->dataLen;
        memcpy(pSubeventResults->data, pData->data, pSubeventResults->dataLen);

        // Send the subevent results to the RAS profile
        status = AppRRSP_CsSubEvent(pSubeventResults);

        // Free pSubeventResults
        ICall_free(pSubeventResults);
    }

    return status;
}

/*********************************************************************
 * @fn      AppRRSPExtCtrl_SendCsSubEventCont
 *
 * @brief   Handle the CS event continuation.
 *
 * @param   pData    - pointer to the event data.
 *
 * @return status - SUCCESS or FAILURE
 */
static uint8_t AppRRSPExtCtrl_SendCsSubEventCont(const AppExtCtrlRRSPSendCsEvtContCmd_t *pData)
{
    ChannelSounding_subeventResultsContinue_t *pSubeventResultsCont;
    uint8_t status = FAILURE;

    pSubeventResultsCont = (ChannelSounding_subeventResultsContinue_t *)ICall_malloc(sizeof(ChannelSounding_subeventResultsContinue_t) + pData->dataLen);

    if (pSubeventResultsCont != NULL)
    {
        pSubeventResultsCont->csEvtOpcode         = pData->csEvtOpcode;
        pSubeventResultsCont->connHandle          = pData->connHandle;
        pSubeventResultsCont->configID            = pData->configID;
        pSubeventResultsCont->procedureDoneStatus = pData->procedureDoneStatus;
        pSubeventResultsCont->subeventDoneStatus  = pData->subeventDoneStatus;
        pSubeventResultsCont->abortReason         = pData->abortReason;
        pSubeventResultsCont->numAntennaPath      = pData->numAntennaPath;
        pSubeventResultsCont->numStepsReported    = pData->numStepsReported;
        pSubeventResultsCont->dataLen             = pData->dataLen;
        memcpy(pSubeventResultsCont->data, pData->data, pSubeventResultsCont->dataLen);

        // Send the subevent results continuation to the RAS profile
        status = AppRRSP_CsSubContEvent(pSubeventResultsCont);

        // Free pSubeventResultsCont
        ICall_free(pSubeventResultsCont);
    }

    return status;
}

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * @fn      AppRRSPExtCtrl_start
 *
 * @brief   Register the ranging server external control handler.
 *
 * @return  SUCCESS/FAILURE
 */
uint8_t AppRRSPExtCtrl_start(void)
{
    gExtHostEvtHandler = Dispatcher_registerMsgHandler(APP_SPECIFIER_RRSP,
                                                      &AppRRSPExtCtrl_commandParser,
                                                      APP_CAP_RRSP);
    return (gExtHostEvtHandler != NULL) ? SUCCESS : FAILURE;
}

#endif // defined(RANGING_SERVER_EXTCTRL_APP) && defined(RANGING_SERVER)
