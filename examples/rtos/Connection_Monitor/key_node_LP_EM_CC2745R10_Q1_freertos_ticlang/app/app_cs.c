/******************************************************************************

@file  app_cs.c

 @brief This source file demonstrates how to activate the Channel Sounding (CS) role
 using BLEAppUtil APIs. It includes the registration, initialization, and activation
 processes utilizing the BLEAppUtil API functions and the structures defined within
 the file.

 The ChannelSounding_start() function at the bottom of the file handles the registration,
 initialization, and activation using the BLEAppUtil API functions.

 Additionally, this example provides APIs that can be called from the upper layer
 and supports sending events to an external control module. These APIs cover various
 functionalities such as reading local and remote capabilities, creating configurations,
 enabling security, setting default settings, managing channel classification, and
 handling CS procedures.

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

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "app_cs_api.h"
#include "ti_ble_config.h"
#include "app_main.h"
#include "string.h"

//*****************************************************************************
//! Prototypes
//*****************************************************************************
static void ChannelSounding_EventHandler(uint32, BLEAppUtil_msgHdr_t *);
static void ChannelSounding_extEvtHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t *);
static void ChannelSounding_csTypeEventHandler(csEvtHdr_t *pCsEvt);

//*****************************************************************************
//! Globals
//*****************************************************************************

//! The external event handler
// *****************************************************************//
// ADD YOUR EVENT HANDLER BY CALLING @ref ChannelSounding_registerEvtHandler
//*****************************************************************//
static ExtCtrl_eventHandler_t gExtEvtHandler = NULL;

// Events handlers struct, contains the handlers and event masks
// of the application cs role module
BLEAppUtil_EventHandler_t csCmdCompleteEvtHandler =
{
  .handlerType    = BLEAPPUTIL_CS_TYPE,
  .pEventHandler  = ChannelSounding_EventHandler,
  .eventMask      = BLEAPPUTIL_CS_EVENT_CODE
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*******************************************************************************
 * Public function defined in app_cs_api.h
 */
bStatus_t ChannelSounding_readLocalSupportedCapabilities(void)
{
  bStatus_t status = SUCCESS;
  csCapabilities_t localCsCapabilities;

  status = CS_ReadLocalSupportedCapabilities(&localCsCapabilities);

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_api.h
 */
bStatus_t ChannelSounding_readRemoteSupportedCapabilities(ChannelSounding_readRemoteCapCmdParams_t* pReadRemoteCapParams)
{
  bStatus_t status = FAILURE;

  if ( pReadRemoteCapParams != NULL )
  {
    status = CS_ReadRemoteSupportedCapabilities(
            (CS_readRemoteCapCmdParams_t *) pReadRemoteCapParams);
  }

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_api.h
 */
bStatus_t ChannelSounding_writeCachedRemoteSupportedCapabilities(ChannelSounding_writeCachedRemoteCapCmdParams_t* pWriteCRSCParams)
{
  bStatus_t status = FAILURE;

  if ( pWriteCRSCParams != NULL )
  {
    status = CS_WriteCachedRemoteSupportedCapabilities(
            (CS_writeCachedRemoteCapCmdParams_t *) pWriteCRSCParams);
  }

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_api.h
 */
bStatus_t ChannelSounding_createConfig(ChannelSounding_createConfigCmdParams_t* pCreateConfigParams)
{
  bStatus_t status = FAILURE;

  if ( pCreateConfigParams != NULL )
  {
    status = CS_CreateConfig((CS_createConfigCmdParams_t *) pCreateConfigParams);
  }

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_api.h
 */
bStatus_t ChannelSounding_securityEnable(ChannelSounding_securityEnableCmdParams_t* pCSsecurityEnableParams)
{
  bStatus_t status = FAILURE;

  if ( pCSsecurityEnableParams != NULL )
  {
    status = CS_SecurityEnable((CS_securityEnableCmdParams_t *) pCSsecurityEnableParams);
  }

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_api.h
 */
bStatus_t ChannelSounding_setDefaultSettings(ChannelSounding_setDefaultSettingsCmdParams_t* pSetDefaultSettingsParams)
{
  bStatus_t status = FAILURE;

  if ( pSetDefaultSettingsParams != NULL )
  {
    status = CS_SetDefaultSettings(
            (CS_setDefaultSettingsCmdParams_t *) pSetDefaultSettingsParams);
  }

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_api.h
 */
bStatus_t ChannelSounding_readRemoteFAETable(ChannelSounding_readRemoteFAETableCmdParams_t* pReadRemoteFAETableParams)
{
  bStatus_t status = FAILURE;

  if ( pReadRemoteFAETableParams != NULL )
  {
    status = CS_ReadRemoteFAETable(
            (CS_readRemoteFAETableCmdParams_t *) pReadRemoteFAETableParams);
  }

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_api.h
 */
bStatus_t ChannelSounding_writeCachedRemoteFAETable(ChannelSounding_writeCachedRemoteFAETableCmdParams_t* pWriteCachedRemoteFAETableParams)
{
  bStatus_t status = FAILURE;

  if ( pWriteCachedRemoteFAETableParams != NULL )
  {
    status = CS_WriteCachedRemoteFAETable(
            (CS_writeCachedRemoteFAETableCmdParams_t *) pWriteCachedRemoteFAETableParams);
  }

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_api.h
 */
bStatus_t ChannelSounding_removeConfig(ChannelSounding_removeConfigCmdParams_t* pRemoveConfigParams)
{
  bStatus_t status = FAILURE;

  if ( pRemoveConfigParams != NULL )
  {
    status = CS_RemoveConfig((CS_removeConfigCmdParams_t *) pRemoveConfigParams);
  }

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_api.h
 */
bStatus_t ChannelSounding_setChannelClassification(ChannelSounding_SetChannelClassificationCmdParams_t* pSetChannelClassificationParams)
{
  bStatus_t status = FAILURE;

  if ( pSetChannelClassificationParams != NULL )
  {
    status = CS_SetChannelClassification(
            (CS_setChannelClassificationCmdParams_t *) pSetChannelClassificationParams);
  }

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_api.h
 */
bStatus_t ChannelSounding_setProcedureParameters(ChannelSounding_setProcedureParamsCmdParams_t* pSetProcedureParametersParams)
{
  bStatus_t status = FAILURE;

  if ( pSetProcedureParametersParams != NULL )
  {
    status = CS_SetProcedureParameters(
            (CS_setProcedureParamsCmdParams_t *) pSetProcedureParametersParams);
  }

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_api.h
 */
bStatus_t ChannelSounding_procedureEnable(ChannelSounding_setProcedureEnableCmdParams_t* pSetProcedureEnableParams)
{
  bStatus_t status = FAILURE;

  if ( pSetProcedureEnableParams != NULL )
  {
    status = CS_ProcedureEnable((CS_setProcedureEnableCmdParams_t *) pSetProcedureEnableParams);
  }

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_api.h
 */
bStatus_t ChannelSounding_setDefaultAntenna(ChannelSounding_setDefaultAntennaCmdParams_t* pSetDefaultAntennaParams)
{
  bStatus_t status = FAILURE;

  if ( pSetDefaultAntennaParams != NULL )
  {
    status = CS_SetDefaultAntenna((CS_setDefaultAntennaCmdParams_t *) pSetDefaultAntennaParams);
  }

  return status;
}


/*********************************************************************
 * @fn      ChannelSounding_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @param   fEventHandler - pointer to the external event handler function.
 *
 * @return  None
 */
void ChannelSounding_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler)
{
  gExtEvtHandler = fEventHandler;
}

/*********************************************************************
 * @fn      ChannelSounding_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the cs
 *          application module and initiate the parser module.
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t ChannelSounding_start()
{
  bStatus_t status = USUCCESS;

  // Register to command complete events
  status = BLEAppUtil_registerEventHandler(&csCmdCompleteEvtHandler);

  if( status == USUCCESS )
  {
    status = BLEAppUtil_registerCsCB();
  }

  // Return status value
  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_api.h
 */
uint8_t ChannelSounding_getRole(uint16_t connHandle, uint8_t configID)
{
    uint8_t ALIGNED role;       // role to return
    csStatus_e status;
    CS_GetRoleCmdParams_t pParams;  // parameters to be sent to the stack

    pParams.role        =   &role;
    pParams.connHandle  =   connHandle;
    pParams.configID    =   configID;

    status = CS_GetRole(&pParams);
    if(status != CS_STATUS_SUCCESS)
    {
        role = 0xFF;
    }

    return role;
}

/***********************************************************************
** Internal Functions
*/

/*********************************************************************
 * @fn      ChannelSounding_EventHandler
 *
 * @brief   The purpose of this function is to handle events
 *          that originate from the HCI GAP and were registered in
 *          @ref BLEAppUtil_registerEventHandler
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
static void ChannelSounding_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if ( pMsgData != NULL )
  {
    switch(event)
    {
      case BLEAPPUTIL_CS_EVENT_CODE:
      {
        ChannelSounding_csTypeEventHandler((csEvtHdr_t *) pMsgData);
        break;
      }

      default:
        break;
    }
  }
}

/*********************************************************************
 * @fn      ChannelSounding_extEvtHandler
 *
 * @brief   Forwards the event to the registered external event handler.
 *
 * @param   eventType - the event type of the event @ref BLEAppUtil_eventHandlerType_e
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  none
 */
static void ChannelSounding_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  // Send the event to the upper layer if its handle exists
  if ( gExtEvtHandler != NULL )
  {
    gExtEvtHandler(eventType, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      ChannelSounding_csTypeEventHandler
 *
 * @brief Handles Gap events for Channel Sounding (CS) operations.
 *
 * This internal function processes Gap events related to Channel Sounding (CS) operations.
 * It checks the event code and casts the event data to the appropriate structure based on the event code.
 * If the event is relevant to CS, it sets a flag to send the event to the external control handler.
 * If an event needs further processing, it will happen here.
 *
 * @param pCsEvt Pointer to the HCI LE event structure.
 *
 * @return None
 */
static void ChannelSounding_csTypeEventHandler(csEvtHdr_t *pCsEvt)
{
  switch( pCsEvt->opcode )
  {
    case CS_READ_REMOTE_SUPPORTED_CAPABILITIES_COMPLETE_EVENT:
    case CS_CONFIG_COMPLETE_EVENT:
    case CS_READ_REMOTE_FAE_TABLE_COMPLETE_EVENT:
    case CS_SECURITY_ENABLE_COMPLETE_EVENT:
    case CS_PROCEDURE_ENABLE_COMPLETE_EVENT:
    case CS_SUBEVENT_RESULT:
    case CS_SUBEVENT_CONTINUE_RESULT:
    {
        break;
    }
    default:
    {
        break;
    }
  }

  ChannelSounding_extEvtHandler(BLEAPPUTIL_CS_TYPE, BLEAPPUTIL_CS_EVENT_CODE, (BLEAppUtil_msgHdr_t *) pCsEvt);
}
