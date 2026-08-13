/******************************************************************************

@file  app_ranging_server_api.h

@brief This file contains the ranging server APIs and structures.

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

#ifndef APP_RANGING_SERVER_API_H
#define APP_RANGING_SERVER_API_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "app_cs_api.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * Functions
 */

/*********************************************************************
 * @fn      AppRRSP_start
 *
 * @brief   This function is called to start the RRSP module.
 *
 * @return  SUCCESS or an error status indicating the failure reason.
 */
uint8_t AppRRSP_start(void);

/*********************************************************************
 * @fn      AppRRSP_CsProcedureEnable
 *
 * @brief   This function is called when the ranging procedure starts.
 *
 * @param   pEnableCompleteEvent - Pointer to the procedure enable complete event.
 *
 * @return  SUCCESS - if the procedure is successfully enabled,
 *          INVALIDPARAMETER - peer device not register to RAS service,
 *          or other an error status indicating the failure reason.
 */
uint8_t AppRRSP_CsProcedureEnable(ChannelSounding_procEnableComplete_t *pEnableCompleteEvent);

/*********************************************************************
 * @fn      AppRRSP_CsSubEvent
 *
 * @brief   This function is called when channel sounding subevent is received.
 *
 * @param   pAppSubeventResults - Pointer to the subevent results.
 *
 * @return  SUCCESS - if the procedure is successfully processed,
 *          INVALIDPARAMETER - peer device not register to RAS service,
 *          or other an error status indicating the failure reason.
 */
uint8_t AppRRSP_CsSubEvent(ChannelSounding_subeventResults_t *pAppSubeventResults);

/*********************************************************************
 * @fn      AppRRSP_CsSubContEvent
 *
 * @brief   This function is called when channel sounding subevent continue is received.
 *
 * @param   pAppSubeventResultsCont - Pointer to the subevent results continuation.
 *
 * @return  SUCCESS - if the procedure is successfully processed,
 *          INVALIDPARAMETER - peer device not register to RAS service,
 *          or other an error status indicating the failure reason.
 */
uint8_t AppRRSP_CsSubContEvent(ChannelSounding_subeventResultsContinue_t *pAppSubeventResultsCont);

#ifdef __cplusplus
}
#endif

#endif /* APP_RANGING_SERVER_API_H */
