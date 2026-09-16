/******************************************************************************

@file  app_extctrl_central.c

@brief This file parse and process the messages comes form the external control
 dispatcher module, and build the events from the app_central.c application and
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
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
#include "ti/ble/app_util/framework/bleapputil_extctrl_dispatcher.h"
#include <app_extctrl_central.h>

/*********************************************************************
 * PROTOTYPES
 */
static void CentralExtCtrl_sendErrEvt(uint8_t, uint8_t);
static void CentralExtCtrl_commandParser(uint8_t*);
static void CentralExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e, uint32, BLEAppUtil_msgHdr_t*);
static void CentralExtCtrl_extHostEvtHandler(uint8_t*, uint16_t);

/*********************************************************************
 * CONSTANTS
 */
#define CENTRAL_CMD_SCAN_START     0x00
#define CENTRAL_CMD_SCAN_STOP      0x01
#define CENTRAL_CMD_CONNECT        0x02

/*********************************************************************
 * GLOBALS
 */
// The external control host event handler
static ExtCtrlHost_eventHandler_t gExtHostEvtHandler = NULL;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      CentralExtCtrl_commandParser
 *
 * @brief   This function parse the received host message, and call the
 *          the relevant API with the data.
 *
 * @param   pData - pointer to the data message.
 *
 * @return  None
 */
static void CentralExtCtrl_commandParser(uint8_t *pData)
{
  if (pData != NULL)
  {
    bStatus_t status = SUCCESS;
    appMsg_t* appMsg = (appMsg_t*)pData;

    switch (appMsg->cmdOp)
    {
      case CENTRAL_CMD_SCAN_START:
      {
        Central_scanStartCmdParams_t *pParams = (Central_scanStartCmdParams_t *)appMsg->pData;
        status = Central_scanStart(pParams);
        break;
      }

      case CENTRAL_CMD_SCAN_STOP:
      {
        status = Central_scanStop();
        break;
      }

      case CENTRAL_CMD_CONNECT:
      {
        Central_connectCmdParams_t *pParams = (Central_connectCmdParams_t *)appMsg->pData;
        status = Central_connect(pParams);
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
      CentralExtCtrl_sendErrEvt(status, appMsg->cmdOp);
    }
  }
}

/*********************************************************************
 * @fn      CentralExtCtrl_sendErrEvt
 *
 * @brief   This function sends the error event to the external control.
 *
 * @param   errCode - the error code
 * @param   cmdOp   - the command op-code that raised the error
 *
 * @return  None
 */
static void CentralExtCtrl_sendErrEvt(uint8_t errCode, uint8_t cmdOp)
{
  // Create error event structure
  AppExtCtrlErrorEvent_t errEvt;
  errEvt.event        = APP_EXTCTRL_FAILURE;
  errEvt.cmdOp        = cmdOp;
  errEvt.appSpecifier = APP_SPECIFIER_CENTRAL;
  errEvt.errCode      = errCode;

  // Send error event
  CentralExtCtrl_extHostEvtHandler((uint8_t *)&errEvt, sizeof(AppExtCtrlErrorEvent_t));
}

/*********************************************************************
 * @fn      CentralExtCtrl_advReportEvtHandler
 *
 * @brief   This function handles the adv report event.
 *
 * @param   pScanMsg - pointer to the scan message
 *
 * @return  None
 */
static void CentralExtCtrl_advReportEvtHandler(BLEAppUtil_ScanEventData_t *pScanMsg)
{
  if (pScanMsg != NULL && pScanMsg->pBuf != NULL)
  {
    // Create event structure
    AppExtCtrlAdvReportEvent_t advReportEvt;

    advReportEvt.event = APP_EXTCTRL_ADV_REPORT;

    // Copy the address type
    advReportEvt.connInfo.devAddrType = pScanMsg->pBuf->pAdvReport.addrType;

    // Copy the address
    memcpy(advReportEvt.connInfo.devAddr, pScanMsg->pBuf->pAdvReport.addr, B_ADDR_LEN);

    // Copy the data length
    advReportEvt.dataLen =  pScanMsg->pBuf->pAdvReport.dataLen;

    // Copy the data
    memcpy(advReportEvt.pData, pScanMsg->pBuf->pAdvReport.pData, advReportEvt.dataLen);

    // Send the event forward
    CentralExtCtrl_extHostEvtHandler((uint8_t *)&advReportEvt, sizeof(AppExtCtrlAdvReportEvent_t) + advReportEvt.dataLen);
  }
}

/*********************************************************************
 * @fn      CentralExtCtrl_scanDisabledEvtHandler
 *
 * @brief   This function handles the scan disabled event.
 *
 * @param   pScanMsg - pointer to the scan message
 *
 * @return  None
 */
static void CentralExtCtrl_scanDisabledEvtHandler(BLEAppUtil_ScanEventData_t *pScanMsg)
{
  // Check that the pointer is not null and the number of reports
  // is not exceed the SCAN_MAX_REPORT
  if (pScanMsg != NULL && pScanMsg->pBuf != NULL &&
          pScanMsg->pBuf->pScanDis.numReport <= SCAN_MAX_REPORT)
  {
    // If there is scan reports, build the payload event
    AppExtCtrlScanDisabledEvent_t scanDisabledEvt;
    scanDisabledEvt.hdr.event        = APP_EXTCTRL_SCAN_DISABLED;
    scanDisabledEvt.hdr.reason       = pScanMsg->pBuf->pScanDis.reason;
    scanDisabledEvt.hdr.numOfReports = pScanMsg->pBuf->pScanDis.numReport;

    // Get the number of reports that can be receive from the Gap
    uint8_t numOfReports = scanDisabledEvt.hdr.numOfReports;
    // The data length will be related to the number of reports in addition of the
    // event header.
    uint16_t dataLen = sizeof(AppExtCtrlScanDisabledEventHdr_t) + numOfReports * sizeof(AppExtCtrlConnInfo_t);

    // pData is a pointer which will iterate over the scan reports.
    AppExtCtrlConnInfo_t *pData;

    // Add the scan reports one by one
    for (uint8_t i = 0 ; i < numOfReports ; i++)
    {
      GapScan_Evt_AdvRpt_t advReport;
      // Get the address from the report
      GapScan_getAdvReport(i, &advReport);
      // Promote the pointer
      pData = &(scanDisabledEvt.advertisers[i]);
      // Copy the address type and promote the pointer
      memcpy(&(pData->devAddrType), &advReport.addrType, sizeof(uint8_t));
      // Copy the address
      memcpy(pData->devAddr, advReport.addr, B_ADDR_LEN);
    }
    // Send the event forward
    CentralExtCtrl_extHostEvtHandler((uint8_t *)&scanDisabledEvt, dataLen);
  }
}

/*********************************************************************
 * @fn      CentralExtCtrl_scanEnabledEvtHandler
 *
 * @brief   This function handles the scan enabled event.
 *
 * @return  None
 */
static void CentralExtCtrl_scanEnabledEvtHandler(void)
{
  AppExtCtrlEvent_e event = APP_EXTCTRL_SCAN_ENABLED;

  CentralExtCtrl_extHostEvtHandler((uint8_t *)&event, sizeof(AppExtCtrlEvent_e));
}

/*********************************************************************
 * @fn      CentralExtCtrl_gapScanTypeEvtHandler
 *
 * @brief   This function handles events from type @ref BLEAPPUTIL_GAP_SCAN_TYPE
 *
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void CentralExtCtrl_gapScanTypeEvtHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  switch (event)
  {
    case BLEAPPUTIL_SCAN_ENABLED:
    {
      CentralExtCtrl_scanEnabledEvtHandler();
      break;
    }

    case BLEAPPUTIL_SCAN_DISABLED:
    {
      CentralExtCtrl_scanDisabledEvtHandler((BLEAppUtil_ScanEventData_t *)pMsgData);
      break;
    }

    case BLEAPPUTIL_ADV_REPORT:
    {
      CentralExtCtrl_advReportEvtHandler((BLEAppUtil_ScanEventData_t *)pMsgData);
      break;
    }

    default:
    {
      break;
    }
  }
}

/*********************************************************************
 * @fn      CentralExtCtrl_eventHandler
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
static void CentralExtCtrl_eventHandler(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  if (pMsgData != NULL)
  {
    switch (eventType)
    {
      case BLEAPPUTIL_GAP_SCAN_TYPE:
      {
        CentralExtCtrl_gapScanTypeEvtHandler(event, pMsgData);
      }

      default :
      {

        break;
      }
    }
  }
}

/*********************************************************************
 * @fn      CentralExtCtrl_extHostEvtHandler
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
static void CentralExtCtrl_extHostEvtHandler(uint8_t *pData, uint16_t dataLen)
{
  if (gExtHostEvtHandler != NULL && pData != NULL)
  {
    gExtHostEvtHandler(pData, dataLen);
  }
}

/*********************************************************************
 * @fn      CentralExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the central module
 *          to the external control dispatcher, and register the call back
 *          event handler function to the central application.
 *
 * @return  SUCCESS/FAILURE
 */
bStatus_t CentralExtCtrl_start(void)
{
  bStatus_t status = SUCCESS;

  // Register to the Dispatcher module and get the event handler
  gExtHostEvtHandler = Dispatcher_registerMsgHandler(APP_SPECIFIER_CENTRAL,
                                                     &CentralExtCtrl_commandParser,
                                                     APP_CAP_CENTRAL);

  // If the registration succeed, register event handler call back to the central application
  if (gExtHostEvtHandler != NULL)
  {
    Central_registerEvtHandler(&CentralExtCtrl_eventHandler);
  }
  return status;
}

#endif // ( HOST_CONFIG & ( CENTRAL_CFG ) )
