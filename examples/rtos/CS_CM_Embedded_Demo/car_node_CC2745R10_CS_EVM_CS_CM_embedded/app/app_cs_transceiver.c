/******************************************************************************

@file app_cs_transceiver.c

@brief This file contains the implementation of the transceiver control logic
       for the Channel Sounding application.
       It transmits and receives data over L2CAP.

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
#include "ti/ble/host/cs/cs.h"
#include "string.h"
#include "app_cs_api.h"
#include "app_ranging_server_api.h"
#include "ti_ble_config.h"

//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! TYPEDEFS
//*****************************************************************************
//*****************************************************************************
//! Prototypes
//*****************************************************************************


//*****************************************************************************
//! Globals
//*****************************************************************************

//*****************************************************************************
//! Functions
//*****************************************************************************

/*******************************************************************************
 * Public function defined in app_cs_transceiver_api.h
 */
bStatus_t ChannelSoundingTransceiver_sendResults(uint8_t *pMsgData, uint16_t dataLen, ChannelSounding_eventOpcodes_e eventOpcode)
{
  bStatus_t status = SUCCESS;

    switch( eventOpcode )
  {
    case APP_CS_PROCEDURE_ENABLE_COMPLETE_EVENT:
    {
#ifdef RANGING_SERVER
      ChannelSounding_procEnableComplete_t *pAppProcedureEnableComplete = (ChannelSounding_procEnableComplete_t *) pMsgData;
      status = AppRRSP_CsProcedureEnable(pAppProcedureEnableComplete);
#endif // RANGING_SERVER
      break;
    }

    case APP_CS_SUBEVENT_RESULT:
    {
#ifdef RANGING_SERVER
      ChannelSounding_subeventResults_t *pAppSubeventResults = (ChannelSounding_subeventResults_t *) pMsgData;
      // send event to the ranging server application
      status = AppRRSP_CsSubEvent(pAppSubeventResults);
#endif // !defined( RANGING_SERVER )
      break;
    }

    case APP_CS_SUBEVENT_CONTINUE_RESULT:
    {
#ifdef RANGING_SERVER
      ChannelSounding_subeventResultsContinue_t *pAppSubeventResultsCont = (ChannelSounding_subeventResultsContinue_t *) pMsgData;
      // send event to the ranging server application
      status = AppRRSP_CsSubContEvent(pAppSubeventResultsCont);
#endif // !defined( RANGING_SERVER )
      break;
    }

    case APP_CS_SUBEVENT_CONTINUE_RESULT_EXT:
    {
#ifdef RANGING_SERVER
      ChannelSounding_subeventResultsContinueExt_t *pAppSubeventResultsContExt = (ChannelSounding_subeventResultsContinueExt_t *) pMsgData;

      uint16_t eventSize = sizeof(ChannelSounding_subeventResultsContinue_t) + pAppSubeventResultsContExt->dataLen;
      ChannelSounding_subeventResultsContinue_t* appSubeventResultsCont = (ChannelSounding_subeventResultsContinue_t*) ICall_malloc(eventSize);
      if (appSubeventResultsCont != NULL)
      {
        appSubeventResultsCont->csEvtOpcode = pAppSubeventResultsContExt->csEvtOpcode;
        appSubeventResultsCont->connHandle = pAppSubeventResultsContExt->connHandle;
        appSubeventResultsCont->configID = pAppSubeventResultsContExt->configID;
        appSubeventResultsCont->procedureDoneStatus = pAppSubeventResultsContExt->procedureDoneStatus;
        appSubeventResultsCont->subeventDoneStatus = pAppSubeventResultsContExt->subeventDoneStatus;
        appSubeventResultsCont->abortReason = pAppSubeventResultsContExt->abortReason;
        appSubeventResultsCont->numAntennaPath = pAppSubeventResultsContExt->numAntennaPath;
        appSubeventResultsCont->numStepsReported = pAppSubeventResultsContExt->numStepsReported;
        appSubeventResultsCont->dataLen = pAppSubeventResultsContExt->dataLen;
        memcpy(appSubeventResultsCont->data, pAppSubeventResultsContExt->data, pAppSubeventResultsContExt->dataLen);

        // send event to the ranging server application
        status = AppRRSP_CsSubContEvent(appSubeventResultsCont);

        ICall_free(appSubeventResultsCont);
      }
#endif // !defined( RANGING_SERVER )
      break;
    }

    default:
    {
        // Unsupported event opcode
        status = FAILURE;
        break;
    }
  }

  return status;
}

/*******************************************************************************
 * Public function defined in app_cs_transceiver_api.h
 */
bStatus_t ChannelSoundingTransceiver_start( void )
{
  bStatus_t status = FAILURE;

#ifdef RANGING_SERVER
  // Start the Ranging Server application
  status = AppRRSP_start();
#else
  status = USUCCESS;
#endif // !defined( RANGING_SERVER )

  return status;
}


/***********************************************************************
** Internal Functions
*/
