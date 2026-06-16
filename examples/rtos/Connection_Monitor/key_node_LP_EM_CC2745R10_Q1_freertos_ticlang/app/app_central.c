/******************************************************************************

@file  app_central.c

@brief This example file demonstrates how to activate the central role with
the help of BLEAppUtil APIs.

scanning events structure is used for event handling,
the eventMask is used to specify the events that will be received
and handled.
In addition, structures must be provided for scan init parameters,
scan start and connection init.

In the Central_start() function at the bottom of the file, registration,
initialization and activation are done using the BLEAppUtil API functions,
using the structures defined in the file.

The example provides also APIs to be called from upper layer and support sending events
to an external control module.

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
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )

#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "app_central_api.h"
#include "ti_ble_config.h"
#include "app_main.h"
#include <string.h>

//*****************************************************************************
//! Prototypes
//*****************************************************************************

void Central_ScanEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

//*****************************************************************************
//! Globals
//*****************************************************************************

//! The external event handler
// *****************************************************************//
// ADD YOUR EVENT HANDLER BY CALLING @ref Central_registerEvtHandler
//*****************************************************************//
static ExtCtrl_eventHandler_t gExtEvtHandler = NULL;

// Events handlers struct, contains the handlers and event masks
// of the application central role module
BLEAppUtil_EventHandler_t centralScanHandler =
{
  .handlerType    = BLEAPPUTIL_GAP_SCAN_TYPE,
  .pEventHandler  = Central_ScanEventHandler,
  .eventMask      = BLEAPPUTIL_SCAN_ENABLED |
                    BLEAPPUTIL_SCAN_DISABLED|
                    BLEAPPUTIL_ADV_REPORT
};

const BLEAppUtil_ScanInit_t centralScanInitParams =
{
  /*! Opt SCAN_PRIM_PHY_1M | SCAN_PRIM_PHY_CODED */
  .primPhy                    = DEFAULT_SCAN_PHY,

  /*! Opt SCAN_TYPE_ACTIVE | SCAN_TYPE_PASSIVE */
  .scanType                   = DEFAULT_SCAN_TYPE,

  /*! Scan interval shall be greater than or equal to scan window */
  .scanInterval               = DEFAULT_SCAN_INTERVAL, /* Units of 625 us */

  /*! Scan window shall be less than or equal to scan interval */
  .scanWindow                 = DEFAULT_SCAN_WINDOW, /* Units of 625 us */

  /*! Select which fields of an advertising report will be stored */
  /*! in the AdvRptList, For mor field see @ref Gap_scanner.h     */
  .advReportFields            = ADV_RPT_FIELDS,

  /*! Opt SCAN_PRIM_PHY_1M | SCAN_PRIM_PHY_CODED */
  .scanPhys                   = DEFAULT_INIT_PHY,

  /*! Opt SCAN_FLT_POLICY_ALL | SCAN_FLT_POLICY_AL |   */
  /*! SCAN_FLT_POLICY_ALL_RPA | SCAN_FLT_POLICY_AL_RPA */
  .fltPolicy                  = SCANNER_FILTER_POLICY,

  /*! For more filter PDU @ref Gap_scanner.h */
  .fltPduType                 = SCANNER_FILTER_PDU_TYPE,

  /*! Opt SCAN_FLT_RSSI_ALL | SCAN_FLT_RSSI_NONE */
  .fltMinRssi                 = SCANNER_FILTER_MIN_RSSI,

  /*! Opt SCAN_FLT_DISC_NONE | SCAN_FLT_DISC_GENERAL | SCAN_FLT_DISC_LIMITED
   *  | SCAN_FLT_DISC_ALL | SCAN_FLT_DISC_DISABLE */
  .fltDiscMode                = SCANNER_FILTER_DISC_MODE,

  /*! Opt SCAN_FLT_DUP_ENABLE | SCAN_FLT_DUP_DISABLE | SCAN_FLT_DUP_RESET */
  .fltDup                     = SCANNER_DUPLICATE_FILTER
};

const BLEAppUtil_ConnParams_t centralConnInitParams =
{
   /*! Opt INIT_PHY_ALL | INIT_PHY_1M | INIT_PHY_2M | INIT_PHY_CODED */
  .initPhys              = DEFAULT_INIT_PHY,

  .scanInterval          = INIT_PHYPARAM_SCAN_INT,      /* Units of 0.625ms */
  .scanWindow            = INIT_PHYPARAM_SCAN_WIN,      /* Units of 0.625ms */
  .minConnInterval       = INIT_PHYPARAM_MIN_CONN_INT,  /* Units of 1.25ms  */
  .maxConnInterval       = INIT_PHYPARAM_MAX_CONN_INT,  /* Units of 1.25ms  */
  .connLatency           = INIT_PHYPARAM_CONN_LAT,
  .supTimeout            = INIT_PHYPARAM_SUP_TO         /* Units of 10ms */
};

BLEAppUtil_ScanStart_t centralScanStartParams =
{
  /*! Zero for continuously scanning */
  .scanPeriod     = DEFAULT_SCAN_PERIOD, /* Units of 1.28sec */

  /*! Scan Duration shall be greater than to scan interval,*/
  /*! Zero continuously scanning. */
  .scanDuration   = DEFAULT_SCAN_DURATION, /* Units of 10ms */

  /*! If non-zero, the list of advertising reports will be */
  /*! generated and come with @ref GAP_EVT_SCAN_DISABLED.  */
  .maxNumReport   = APP_MAX_NUM_OF_ADV_REPORTS
};


//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Central_scanStart
 *
 * @brief   Central Scan Start Api, This Api starts the scan with the given params.
 *          If duration is zero period shall be ignored and the scanner
 *          will continue scanning until @ref BLEAppUtil_scanStop is called.
 *          If both period is zero and duration is non-zero, the scanner will
 *          scan once until duration has expired or @ref BLEAppUtil_scanStop
 *          is called. If both the duration and period are non-zero, the
 *          scanner will continue canning periodically until @ref BLEAppUtil_scanStop
 *          is called.
 *
 * @param   pScanParams - pointer to the start scan command params.
 *
 * @return  @ref SUCCESS
 * @return  @ref bleNotReady
 * @return  @ref bleInvalidRange
 * @return  @ref bleMemAllocError
 * @return  @ref bleAlreadyInRequestedMode
 * @return  @ref bleIncorrectMode
 */
bStatus_t Central_scanStart(Central_scanStartCmdParams_t *pScanParams)
{
  bStatus_t status = FAILURE;

  if (pScanParams != NULL)
  {
    centralScanStartParams.scanPeriod = pScanParams->scanPeriod;
    centralScanStartParams.scanDuration = pScanParams->scanDuration;
    centralScanStartParams.maxNumReport = pScanParams->maxNumReport;

    status = BLEAppUtil_scanStart(&centralScanStartParams);
  }

  return status;
}

/*********************************************************************
 * @fn      Central_connect
 *
 * @brief   This function initiates a connection with the peer device specified by the
 *          connection parameters provided. It sets up the connection parameters and
 *          calls the @ref BLEAppUtil_connect function to establish the connection.
 *
 * @param pConnParams Pointer to the connection command parameters.
 *
 * @return @ref SUCCESS
 * @return @ref bleNotReady
 * @return @ref bleInvalidRange
 * @return @ref bleMemAllocError
 * @return @ref bleAlreadyInRequestedMode
 * @return @ref FAILURE
 */
bStatus_t Central_connect(Central_connectCmdParams_t *pConnParams)
{
  uint8_t status = FAILURE;
  BLEAppUtil_ConnectParams_t centralConnParams;

  if ( pConnParams != NULL )
  {
    centralConnParams.peerAddrType = pConnParams->addrType;
    memcpy(centralConnParams.pPeerAddress, pConnParams->addr, B_ADDR_LEN);
    centralConnParams.phys = pConnParams->phy;
    centralConnParams.timeout = pConnParams->timeout;

    status = BLEAppUtil_connect(&centralConnParams);
  }

  return status;
}

/*********************************************************************
 * @fn      Central_scanStop
 *
 * @brief   Central Scan Stop Api, This Api stops currently running scanning operation.
 *
 * @return @ref SUCCESS
 * @return @ref FAILURE
 * @return @ref bleIncorrectMode
 */
bStatus_t Central_scanStop()
{
  bStatus_t status = SUCCESS;

  status = BLEAppUtil_scanStop();

  return status;
}

/*********************************************************************
 * @fn      Central_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void Central_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler)
{
  gExtEvtHandler = fEventHandler;
}

/*********************************************************************
 * @fn      Central_extEvtHandler
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
static void Central_extEvtHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  // Send the event to the upper layer if its handle exists
  if (gExtEvtHandler != NULL)
  {
    gExtEvtHandler(eventType, event, pMsgData);
  }
}
/*********************************************************************
 * @fn      Central_ScanEventHandler
 *
 * @brief   The purpose of this function is to handle scan events
 *          that rise from the GAP and were registered in
 *          @ref BLEAppUtil_registerEventHandler
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void Central_ScanEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    // Send the event to the upper layer
    Central_extEvtHandler(BLEAPPUTIL_GAP_SCAN_TYPE, event, pMsgData);
  }
}

/*********************************************************************
 * @fn      Central_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the central
 *          application module and initiate the parser module.
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Central_start()
{
  bStatus_t status = SUCCESS;

  status = BLEAppUtil_registerEventHandler(&centralScanHandler);
  if (status != SUCCESS)
  {
    // Return status value
    return(status);
  }

  status = BLEAppUtil_scanInit(&centralScanInitParams);
  if (status != SUCCESS)
  {
    // Return status value
    return(status);
  }

  status = BLEAppUtil_setConnParams(&centralConnInitParams);
  if(status != SUCCESS)
  {
      // Return status value
      return(status);
  }

  // Return status value
  return(status);
}

#endif // ( HOST_CONFIG & ( CENTRAL_CFG ) )
