/******************************************************************************

@file  app_key_node.c

@brief This file contains the key node functionalities for the BLE application.
       It handles the advertisement, connection and CS.

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

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti_ble_config.h"
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "ti/ble/host/gap/gap_scanner.h"
#include "app_key_node.h"
#include "app_central_api.h"
#include "app_gatt_api.h"
#include "app_pairing_api.h"
#include "app_connection_api.h"
#include "app_cs_api.h"
#include "app_cs_transceiver_api.h"

//*****************************************************************************
//! Defines
//*****************************************************************************
#define KEY_NODE_NAME   "Basic BLE RSSI"
// #define KEY_NODE_NAME  	"changan-25331454"
#define PEER_NAME_LEN   (sizeof(KEY_NODE_NAME) - 1)

#define KEY_NODE_INVALID_PROCEDURE_COUNTER  0xFFFFFFFF

//*****************************************************************************
//! Prototypes
//*****************************************************************************
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
static void KeyNode_connect(uint8_t peerAddrType, uint8_t *peerAddr, uint8_t phy);
static uint8_t KeyNode_startScan(void);
#endif

#ifdef CHANNEL_SOUNDING
static void KeyNode_csEvtHandler(csEvtHdr_t *pCsEvt);
static void KeyNode_handleCsEvent(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
#endif // CHANNEL_SOUNDING

//*****************************************************************************
//! Globals
//*****************************************************************************
// This is the name of the peer device that the scanner will look for when the
// key node is defined with central configuration
uint8_t carNodeName[PEER_NAME_LEN] = {'C','a', 'r', ' ', 'N', 'o', 'd', 'e'};

// Procedure Counter of the current subevent being processed
uint32_t gCurrProcedureCounter = KEY_NODE_INVALID_PROCEDURE_COUNTER;


//*****************************************************************************
//! Functions
//*****************************************************************************

/*******************************************************************************
 * This API is starts the key node application.
 *
 * Public function defined in app_key_node.h.
 */
uint8_t KeyNode_start(void)
{
    uint8_t status = SUCCESS;

    // Register to receive connection and pairing events
    Connection_registerEvtHandler(&KeyNode_handleConnectionEvent);

    Pairing_registerEvtHandler(&KeyNode_handlePairingEvent);
#ifdef CHANNEL_SOUNDING
    // Register to the Channel Sounding events
    ChannelSounding_registerEvtHandler(&KeyNode_handleCsEvent);
#endif // CHANNEL_SOUNDING

    // Start the Transceiver
    if( status == USUCCESS )
    {
        status = ChannelSoundingTransceiver_start();
    }

    // If Central configuration is required, register to receive different scan events
    // and start scanning
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
    Central_registerEvtHandler(&KeyNode_handleScanEvent);

    status = KeyNode_startScan();
#endif

    return status;
}

/*******************************************************************************
 * This function handle connection events.
 *
 * Public function defined in app_key_node.h.
 */
void KeyNode_handleConnectionEvent(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    if ( pMsgData != NULL )
    {
        if ( event == BLEAPPUTIL_LINK_TERMINATED_EVENT )
        {
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
            // Central config is included. Start scanning again
            KeyNode_startScan();
#endif
        }
        else if ( event == BLEAPPUTIL_LINK_ESTABLISHED_EVENT )
        {
            gapEstLinkReqEvent_t *pGapEstMsg = (gapEstLinkReqEvent_t *)pMsgData;

            // Call MTU exchange
            AppGATT_exchangeMTU(pGapEstMsg->connectionHandle, MAX_PDU_SIZE);
        }
        else if ( event == BLEAPPUTIL_CONNECTING_CANCELLED_EVENT )
        {
            // The connection attempt timed out. Restart scan
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
            KeyNode_startScan();
#endif
        }
    }
}

/*********************************************************************
 * @fn      KeyNode_handlePairingEvent
 *
 * @brief   This function handles pairing events raised
 *
 * @param   eventType - the type of the events @ref BLEAppUtil_eventHandlerType_e.
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
void KeyNode_handlePairingEvent(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    BLEAppUtil_PairStateData_t *pPairEvt = (BLEAppUtil_PairStateData_t *)pMsgData;
    ChannelSounding_securityEnableCmdParams_t secEnableParams = {0};
    uint8_t status = SUCCESS;
    linkDBInfo_t linkInfo = {0};

    if ( pMsgData == NULL )
    {
        return;
    }

    if ( event == BLEAPPUTIL_PAIRING_STATE_COMPLETE || event == BLEAPPUTIL_PAIRING_STATE_ENCRYPTED )
    {
        // Verify pairing was successful
        if ( pPairEvt->status == SUCCESS )
        {
            // Set the connection handle in the Channel Sounding Security Enable command
            secEnableParams.connHandle = pPairEvt->connHandle;

            // Get the link information
            status = linkDB_GetInfo(secEnableParams.connHandle, &linkInfo);

            // Check the link role
            if ( status == SUCCESS && linkInfo.connRole == GAP_PROFILE_CENTRAL )
            {
                // As a Central, enable Channel Sounding security
                ChannelSounding_securityEnable(&secEnableParams);
            }
        }
    }
}

/*******************************************************************************
 * This function handle scan events.
 *
 * Public function defined in app_key_node.h.
 */
void KeyNode_handleScanEvent(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
    if ( event == BLEAPPUTIL_ADV_REPORT )
    {
        BLEAppUtil_ScanEventData_t *pScanEvt = (BLEAppUtil_ScanEventData_t *)pMsgData;
        GapScan_data_t *pScanData = (GapScan_data_t *)(pScanEvt->pBuf);
        BLEAppUtil_GapScan_Evt_AdvRpt_t *pAdvRpt = (BLEAppUtil_GapScan_Evt_AdvRpt_t *)(&(pScanData->pAdvReport));
        uint8_t *pAdvData = (uint8_t *)(pAdvRpt->pData);
        uint16_t dataLen = pAdvRpt->dataLen;
        uint8_t fieldLen = 0;
        uint8_t fieldType;

        // Search for complete name in the advertising data
        if ( (pAdvRpt->evtType & ADV_RPT_EVT_TYPE_CONNECTABLE) != 0 )
        {
            while ( dataLen != 0 )
            {
                // Extract the field length
                fieldLen = pAdvData[ADV_FIELD_LEN_OFFSET];
                fieldType = pAdvData[ADV_FIELD_TYPE_OFFSET];

                // Check if this is the name we are searching for
                // First check that the name length match. If so, check the actual name
                if ( ( fieldType == GAP_ADTYPE_LOCAL_NAME_COMPLETE ) &&
                     ( PEER_NAME_LEN == fieldLen - 1 ) )
                {
                    if ( memcmp(&pAdvData[ADV_FIELD_DATA_OFFSET], KEY_NODE_NAME, PEER_NAME_LEN) == 0 )
                    {
                        // if (pAdvRpt->addr[1] == 0x4D)
                        // {
                             // Disable the scan
                            Central_scanStop();

                            KeyNode_connect(pAdvRpt->addrType, pAdvRpt->addr, pAdvRpt->primPhy);
                        // }
                    }
                }
                dataLen = dataLen - (fieldLen + 1);
                if ( dataLen != 0 )
                {
                    pAdvData = pAdvData + (fieldLen + 1);
                }
            }
        }
    }
#endif
}

/***********************************************************************
** Internal Functions
*/

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
/*********************************************************************
 * @fn      KeyNode_startScan
 *
 * @brief   This function will prepare the scan enable parameters
 *          and will start scan
 *
 * @param   None
 *
 * @return  @ref SUCCESS
 * @return  @ref bleNotReady
 * @return  @ref bleInvalidRange
 * @return  @ref bleMemAllocError
 * @return  @ref bleAlreadyInRequestedMode
 * @return  @ref bleIncorrectMode
 */
static uint8_t KeyNode_startScan(void)
{
    Central_scanStartCmdParams_t scanParams;

    // Scan until receiving a stop command
    scanParams.scanPeriod = 0;
    scanParams.scanDuration = 0;
    scanParams.maxNumReport = 0;

    return Central_scanStart(&scanParams);
}

/*********************************************************************
 * @fn      KeyNode_connect
 *
 * @brief   This will prepare the connection parameter structure and will
 *          call the Central_connect function to create a connection with
 *          a given peer device.
 *
 * @param   peerAddrType - Peer address type
 * @param   peerAddr     - Peer Address
 * @param   phy          - Peer's primay phy
 *
 * @return  None
 */
static void KeyNode_connect(uint8_t peerAddrType, uint8_t *peerAddr, uint8_t phy)
{
    Central_connectCmdParams_t connParams;

    connParams.addrType = peerAddrType;
    memcpy(connParams.addr, peerAddr, B_ADDR_LEN);
    connParams.phy = phy;
    connParams.timeout = 300;

    Central_connect(&connParams);
}
#endif // ( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )

#ifdef CHANNEL_SOUNDING
/*********************************************************************
 * @fn      KeyNode_handleCsEvent
 *
 * @brief   This function handles Channel Sounding events raised
 *
 * @param   eventType - the type of the events @ref BLEAppUtil_eventHandlerType_e.
 * @param   event     - message event.
 * @param   pMsgData  - pointer to message data.
 *
 * @return  None
 */
static void KeyNode_handleCsEvent(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    if ( eventType == BLEAPPUTIL_CS_TYPE && event == BLEAPPUTIL_CS_EVENT_CODE )
    {
        KeyNode_csEvtHandler((csEvtHdr_t *)pMsgData);
    }
}

/*********************************************************************
 * @fn      KeyNode_csEvtHandler
 *
 * @brief   This function handles @ref BLEAPPUTIL_CS_EVENT_CODE
 *
 * @param   pCsEvt - pointer message data
 *
 * @return  None
 */
static void KeyNode_csEvtHandler(csEvtHdr_t *pCsEvt)
{
  uint8_t opcode = pCsEvt->opcode;

  if (NULL != pCsEvt)
  {
    switch( opcode )
    {
        case CS_READ_REMOTE_SUPPORTED_CAPABILITIES_COMPLETE_EVENT:
        {
        break;
        }

        case CS_CONFIG_COMPLETE_EVENT:
        {
        break;
        }

        case CS_READ_REMOTE_FAE_TABLE_COMPLETE_EVENT:
        {
        break;
        }

        case CS_SECURITY_ENABLE_COMPLETE_EVENT:
        {
        // Assume reflector
        // call default settings
        ChannelSounding_setDefaultSettingsCmdParams_t params; // TODO
        params.connHandle = 0;              //!< Connection handle
        params.roleEnable = 3;              //!< Role enable flag
        params.csSyncAntennaSelection = 1;  //!< CS sync antenna selection
        params.maxTxPower = 10;             //!< Maximum TX power in dBm
        ChannelSounding_setDefaultSettings(&params);
        break;
        }

        case CS_PROCEDURE_ENABLE_COMPLETE_EVENT:
        {
        ChannelSounding_procEnableComplete_t *pAppProcedureEnableComplete = (ChannelSounding_procEnableComplete_t *) pCsEvt;
        ChannelSoundingTransceiver_sendResults((uint8_t*) pAppProcedureEnableComplete, sizeof(ChannelSounding_procEnableComplete_t), APP_CS_PROCEDURE_ENABLE_COMPLETE_EVENT);
        break;
        }

        case CS_SUBEVENT_RESULT:
        {
            ChannelSounding_subeventResults_t *pAppSubeventResults = (ChannelSounding_subeventResults_t *) pCsEvt;

            gCurrProcedureCounter = pAppSubeventResults->procedureCounter;

            ChannelSoundingTransceiver_sendResults((uint8_t*) pAppSubeventResults, sizeof(ChannelSounding_subeventResults_t) + pAppSubeventResults->dataLen, APP_CS_SUBEVENT_RESULT);
            
            if (pAppSubeventResults->procedureDoneStatus == CS_PROCEDURE_DONE ||
                pAppSubeventResults->procedureDoneStatus == CS_PROCEDURE_ABORTED)
            {
                // Reset the procedure counter
                gCurrProcedureCounter = KEY_NODE_INVALID_PROCEDURE_COUNTER;
            }

            break;
        }

        case CS_SUBEVENT_CONTINUE_RESULT:
        {
            ChannelSounding_subeventResultsContinue_t *pAppSubeventResultsCont = (ChannelSounding_subeventResultsContinue_t *) pCsEvt;

            // Calculate the size of the extended event
            uint16_t resultsExtSize = sizeof(ChannelSounding_subeventResultsContinueExt_t) + pAppSubeventResultsCont->dataLen;

            // Allocate memory for the extended event
            ChannelSounding_subeventResultsContinueExt_t* appSubeventResultsContExt =
                                                (ChannelSounding_subeventResultsContinueExt_t*)ICall_malloc(resultsExtSize);

            // If memory allocation was successful, fill the extended event
            if (NULL != appSubeventResultsContExt && gCurrProcedureCounter != KEY_NODE_INVALID_PROCEDURE_COUNTER)
            {
                appSubeventResultsContExt->csEvtOpcode           = pAppSubeventResultsCont->csEvtOpcode;        
                appSubeventResultsContExt->connHandle            = pAppSubeventResultsCont->connHandle;
                appSubeventResultsContExt->configID              = pAppSubeventResultsCont->configID;
                appSubeventResultsContExt->procedureCounter      = (uint16_t) gCurrProcedureCounter;   // Extended parameter
                appSubeventResultsContExt->procedureDoneStatus   = pAppSubeventResultsCont->procedureDoneStatus;
                appSubeventResultsContExt->subeventDoneStatus    = pAppSubeventResultsCont->subeventDoneStatus;
                appSubeventResultsContExt->abortReason           = pAppSubeventResultsCont->abortReason;
                appSubeventResultsContExt->numAntennaPath        = pAppSubeventResultsCont->numAntennaPath;
                appSubeventResultsContExt->numStepsReported      = pAppSubeventResultsCont->numStepsReported;
                appSubeventResultsContExt->dataLen               = pAppSubeventResultsCont->dataLen;

                // Copy subevent data
                memcpy(appSubeventResultsContExt->data, pAppSubeventResultsCont->data, pAppSubeventResultsCont->dataLen);

                // Send the event
                ChannelSoundingTransceiver_sendResults((uint8_t*) appSubeventResultsContExt, resultsExtSize, APP_CS_SUBEVENT_CONTINUE_RESULT_EXT);

                ICall_free(appSubeventResultsContExt);
            }

            if (pAppSubeventResultsCont->procedureDoneStatus == CS_PROCEDURE_DONE ||
                pAppSubeventResultsCont->procedureDoneStatus == CS_PROCEDURE_ABORTED)
            {
                // Reset the procedure counter
                gCurrProcedureCounter = KEY_NODE_INVALID_PROCEDURE_COUNTER;
            }

            break;
        }

        default:
        {
            break;
        }
    }
  }
}

#endif // CHANNEL_SOUNDING
