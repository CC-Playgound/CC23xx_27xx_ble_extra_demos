/******************************************************************************

@file  app_ranging_server.c

@brief This file contains the implementation of the Ranging Responder
       (RRSP) APIs and functionality.

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

#ifdef RANGING_SERVER
//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti_ble_config.h"
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include <ti/ble/profiles/ranging/ranging_profile.h>
#include "app_ranging_server_api.h"

//*****************************************************************************
//! Defines
//*****************************************************************************

// Indicate if Real Time Ranging Data characteristic is supported
#if defined(RANGING_SERVER_REAL_TIME)
    // If RANGING_SERVER_REAL_TIME is defined, enable support
    #define RANGING_SERVER_FEATURES  1
#else
    // Otherwise, disable support
    #define RANGING_SERVER_FEATURES  0
#endif // defined(RANGING_SERVER_REAL_TIME)

//*****************************************************************************
//! Prototypes
//*****************************************************************************

static void rrsp_ProcedureStatusCB( uint8_t status , uint16_t connHandle, uint16_t rangingCounter );
static void rrsp_CccUpdateCB( uint16_t connHandle, uint16_t pValue );

//*****************************************************************************
//! Globals
//*****************************************************************************

static RRSP_cb_t ranging_profileCB =
{
 rrsp_CccUpdateCB,
 rrsp_ProcedureStatusCB,
};

/* Procedure counter per connection handle. Note that a valid procedure counter is 16 bits, although we use 32 bits here
   in order to indicate an invalid value (0xFFFFFFFF), as procedure counter valid values are 0 to 0xFFFF. */
static uint32_t gProcedureCounter[MAX_NUM_BLE_CONNS];

static uint8_t  gProcedureAntennaPath[MAX_NUM_BLE_CONNS];

//*****************************************************************************
//! API Functions
//*****************************************************************************

/*********************************************************************
 * @fn      AppRRSP_start
 *
 * @brief   This function is called to start the RRSP module.
 *
 * @return  SUCCESS or an error status indicating the failure reason.
 */
uint8_t AppRRSP_start(void)
{
  uint8_t status;

  // Initialize the procedure globals
  memset(gProcedureCounter, 0xFF, MAX_NUM_BLE_CONNS * sizeof(gProcedureCounter[0]));
  memset(gProcedureAntennaPath, 0, MAX_NUM_BLE_CONNS * sizeof(gProcedureAntennaPath[0]));

  // Start the ranging responder using the features set via Sysconfig
  status = RRSP_start( &ranging_profileCB, RANGING_SERVER_FEATURES );

  return status;
}

/*********************************************************************
 * @fn      AppRRSP_CsProcedureEnable
 *
 * @brief   This function is called when CS procedure enable complete event is received.
 *
 * @param   pEnableCompleteEvent - Pointer to the event data.
 *
 * @return  SUCCESS - if the procedure is successfully enabled,
 *          INVALIDPARAMETER - peer device not register to RAS service,
 *          or other error status indicating the failure reason.
 */
uint8_t AppRRSP_CsProcedureEnable(ChannelSounding_procEnableComplete_t *pEnableCompleteEvent)
{
    uint8_t status = INVALIDPARAMETER;
    uint8_t antennaPathsMask;

    // Check if the connection handle registered to RAS service and
    // accept initiate the relevant parameters only if the procedure wasn't aborted
    if (pEnableCompleteEvent != NULL &&
        pEnableCompleteEvent->csStatus == SUCCESS &&
        RRSP_RegistrationStatus(pEnableCompleteEvent->connHandle) != RRSP_UNREGISTER)
    {
        // get the antenna paths mask
        antennaPathsMask = CS_calcAntPathsMask(pEnableCompleteEvent->ACI);

        // if the antenna paths mask is valid, save it
        if (antennaPathsMask != 0)
        {
            // Keep the antenna paths mask for the procedure by connection handle
            gProcedureAntennaPath[pEnableCompleteEvent->connHandle] = antennaPathsMask;
            // Update status
            status = SUCCESS;
        }
    }
    return status;
}

/*********************************************************************
 * @fn      AppRRSP_CsSubEvent
 *
 * @brief   This function is called when channel sounding subevent is received.
 *
 * @param   pAppSubeventResults - Pointer to the subevent results.
 *
 * @return  SUCCESS - if the procedure is successfully processed,
 *          INVALIDPARAMETER - peer device not register to RAS service,
 *          FAILURE - if invalid antenna paths mask, procedure was aborted
 *          or other error status indicating the failure reason.
 */
uint8_t AppRRSP_CsSubEvent(ChannelSounding_subeventResults_t *pAppSubeventResults)
{
    uint8_t status = INVALIDPARAMETER;
    Ranging_RangingHeader_t rangingHeader;

    // Check if the connection handle registered to RAS service
    if ( (pAppSubeventResults != NULL) &&
         (RRSP_RegistrationStatus(pAppSubeventResults->connHandle) != RRSP_UNREGISTER) )
    {
        status = SUCCESS;

        // Check if starting new repetition/procedure
        if(gProcedureCounter[pAppSubeventResults->connHandle] != (uint32_t) pAppSubeventResults->procedureCounter)
        {
            // Set the new procedure counter
            gProcedureCounter[pAppSubeventResults->connHandle] = (uint32_t) pAppSubeventResults->procedureCounter;

            // get the antenna paths mask
            rangingHeader.antennaPathsMask = gProcedureAntennaPath[pAppSubeventResults->connHandle];

            // Check antenna paths mask is valid
            if (rangingHeader.antennaPathsMask != 0)
            {
                // Set the ranging header values
                rangingHeader.configID = pAppSubeventResults->configID;
                rangingHeader.rangingCounter = pAppSubeventResults->procedureCounter;
                rangingHeader.selectedTxPower = pAppSubeventResults->referencePowerLevel;

                // Send the ranging header to the RAS profile, take only the first 16 bits of the counter
                status = RRSP_ProcedureStarted(pAppSubeventResults->connHandle,
                                               (uint16_t) gProcedureCounter[pAppSubeventResults->connHandle],
                                               (uint8_t*)&rangingHeader);
            }
            else
            {
                // Invalid antenna paths mask, set status to failure
                status = FAILURE;
            }
        }

        if (status == SUCCESS)
        {
            // Send CS sub event to the RAS profile
            status = RRSP_AddSubeventResult((RRSP_csSubEventResults_t*)pAppSubeventResults);

            // if the procedure is done, inform the RAS profile
            if( (pAppSubeventResults->procedureDoneStatus == CS_PROCEDURE_DONE ||
                 pAppSubeventResults->procedureDoneStatus == CS_PROCEDURE_ABORTED) &&
                (status == SUCCESS) )
            {
                // Inform the RAS profile that the procedure is done, take only the first 16 bits of the counter
                status = RRSP_ProcedureDone(pAppSubeventResults->connHandle, (uint16_t) gProcedureCounter[pAppSubeventResults->connHandle]);
            }
        }
    }

    return status;
}

/*********************************************************************
 * @fn      AppRRSP_CsSubContEvent
 *
 * @brief   This function is called when channel sounding subevent continue is received.
 *
 * @param   pAppSubeventResultsCont - Pointer to the subevent results continuation.
 *
 * @return  SUCCESS - if the procedure is successfully processed,
 *          INVALIDPARAMETER - peer device not register to RAS service
 *          FAILURE - if procedure was aborted
 *          or other error status indicating the failure reason.
 */
uint8_t AppRRSP_CsSubContEvent(ChannelSounding_subeventResultsContinue_t *pAppSubeventResultsCont)
{
    uint8_t status = INVALIDPARAMETER;

    // Check if the connection handle registered to RAS service
    if (pAppSubeventResultsCont != NULL &&
        RRSP_RegistrationStatus(pAppSubeventResultsCont->connHandle) != RRSP_UNREGISTER)
    {
        // Send to the RAS profile Subevent continuation
        status = RRSP_AddSubeventContinueResult((RRSP_csSubEventResultsContinue_t*)pAppSubeventResultsCont);

        // if the procedure is done, inform the RAS profile
        if( (status == SUCCESS) &&
            (pAppSubeventResultsCont->procedureDoneStatus == CS_PROCEDURE_DONE ||
             pAppSubeventResultsCont->procedureDoneStatus == CS_PROCEDURE_ABORTED))
        {
            // Inform the RAS profile that the procedure is done, take only the first 16 bits of the counter
            status = RRSP_ProcedureDone(pAppSubeventResultsCont->connHandle, (uint16_t) gProcedureCounter[pAppSubeventResultsCont->connHandle]);
        }
    }

    return status;
}

//*****************************************************************************
//! Local Functions
//*****************************************************************************

/*********************************************************************
 * @fn    rrsp_ProcedureStatusCB
 * @brief This function is called when the ranging procedure status changes.
 *
 * @param  status - The new status of the ranging procedure.
 *
 * @return None
 */
static void rrsp_ProcedureStatusCB( uint8_t status , uint16_t connHandle, uint16_t rangingCounter )
{
    switch (status)
    {
        case RRSP_SENDING_PROCEDURE_STARTED:
            // Procedure Sending Started
            break;

        case RRSP_SENDING_PROCEDURE_ENDED:
            // Procedure Sending Ended
            break;

        case RRSP_STATUS_SENDING_PROCEDURE_ABORTED:
            // Procedure was aborted
            break;

        default:
            // Unknown status
            break;
    }
}

/*********************************************************************
 * @fn    rrsp_CccUpdateCB
 * @brief This function is called when the CCC of the Ranging procedure changes.
 *
 * @param  None
 *
 * @return None
 */
static void rrsp_CccUpdateCB( uint16_t connHandle, uint16_t pValue )
{
    // TODO: Handle the CCC update
    return;
}
#endif // RANGING_SERVER
