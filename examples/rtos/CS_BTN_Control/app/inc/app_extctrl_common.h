/******************************************************************************

@file  app_extctrl_common.h

@brief This file contain the events and structures interface of the external
application control module.

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

#ifndef APP_EXTCTRL_COMMON_H
#define APP_EXTCTRL_COMMON_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "ti_ble_config.h"

/*********************************************************************
 * MACROS
 */

#define EXTCTRL_PHY_UPDATE_COMPLETE_EVENT_DATA_LEN    ( sizeof(hciEvt_BLEPhyUpdateComplete_t) - sizeof(hciEvtHdr_t) )
#define EXTCTRL_DATA_LENGTH_CHANGE_EVENT_DATA_LEN     ( sizeof(hciEvt_BLEDataLengthChange_t) - sizeof(hciEvtHdr_t) )
#define EXTCTRL_CHANNEL_MAP_UPDATE_EVENT_DATA_LEN     ( sizeof(hciEvt_BLEChannelMapUpdate_t) - sizeof(hciEvtHdr_t) )
#define EXTCTRL_CONN_PARAM_UPDATE_EVENT_DATA_LEN      ( sizeof(hciEvt_BLEConnUpdateComplete_t) - sizeof(hciEvtHdr_t) )

/*********************************************************************
 * Enumerators
 */

// This enum contains the events opcodes for the external application
// control module.
typedef enum
{
  APP_EXTCTRL_ADV_START_AFTER_ENABLE             = 0x0001,
  APP_EXTCTRL_ADV_END_AFTER_DISABLE              = 0x0002,
  APP_EXTCTRL_SCAN_ENABLED                       = 0x0003,
  APP_EXTCTRL_SCAN_DISABLED                      = 0x0004,
  APP_EXTCTRL_ADV_REPORT                         = 0x0005,
  APP_EXTCTRL_LINK_ESTABLISHED_EVENT             = 0x0006,
  APP_EXTCTRL_LINK_TERMINATED_EVENT              = 0x0007,
  APP_EXTCTRL_LINK_PARAM_UPDATE_REQ_EVENT        = 0x0008,
  APP_EXTCTRL_LINK_PARAM_UPDATE_EVENT            = 0x0009,
  APP_EXTCTRL_CONN_NOTI_CONN_EVENT               = 0x000A,
  APP_EXTCTRL_HANDOVER_SN_DATA                   = 0x000B,
  APP_EXTCTRL_PAIRING_STATE_STARTED              = 0x000C,
  APP_EXTCTRL_PAIRING_STATE_COMPLETE             = 0x000D,
  APP_EXTCTRL_PAIRING_STATE_ENCRYPTED            = 0x000E,
  APP_EXTCTRL_PAIRING_STATE_BOND_SAVED           = 0x000F,
  APP_EXTCTRL_HANDOVER_CN_STATUS                 = 0x0010,
  APP_EXTCTRL_CM_SRV_DATA                        = 0x0011,
  APP_EXTCTRL_CM_REPORT                          = 0x0012,
  APP_EXTCTRL_CM_START_EVENT                     = 0x0013,
  APP_EXTCTRL_CM_STOP_EVENT                      = 0x0014,
  APP_EXTCTRL_CM_CONN_UPDATE_EVENT               = 0x0015,
  APP_EXTCTRL_COMMON_DEVICE_ADDRESS              = 0x0020,
  APP_EXTCTRL_COMMON_COMMAND_STATUS_EVENT        = 0x0021,
  APP_EXTCTRL_COMMON_HCI_COMPLETE_EVENT          = 0x0022,
  APP_EXTCTRL_COMMON_HCI_LE_EVENT                = 0x0023,
  APP_EXTCTRL_GATT_MTU_UPDATED_EVENT             = 0X0024,
  APP_EXTCTRL_L2CAP_DATA_RECEIVED                = 0x0030,
  APP_EXTCTRL_L2CAP_CHANNEL_ESTABLISHED          = 0x0031,
  APP_EXTCTRL_L2CAP_CHANNEL_TERMINATED           = 0x0032,
  APP_EXTCTRL_L2CAP_OUT_OF_CREDIT                = 0x0033,
  APP_EXTCTRL_CS_READ_REMOTE_CAPS                = 0x0040,
  APP_EXTCTRL_CS_CONFIG_COMPLETE                 = 0x0041,
  APP_EXTCTRL_CS_READ_REMOTE_FAE_TABLE           = 0x0042,
  APP_EXTCTRL_CS_SECURITY_ENABLE_COMPLETE        = 0x0043,
  APP_EXTCTRL_CS_PROCEDURE_ENABLE_COMPLETE       = 0x0044,
  APP_EXTCTRL_CS_SUBEVENT_RESULTS                = 0x0045,
  APP_EXTCTRL_CS_SUBEVENT_RESULTS_CONTINUE       = 0x0046,
  APP_EXTCTRL_CS_READ_LOCAL_CAPS_CMD_COMPLETE    = 0x0047,
  APP_EXTCTRL_CS_WRITE_REMOTE_FAE_TABLE_COMPLETE = 0x0048,
  APP_EXTCTRL_CS_SET_DEFAULT_SETTINGS_COMPLETE   = 0x0049,
  APP_EXTCTRL_CS_SET_CHNL_CLASS_COMPLETE         = 0x004A,
  APP_EXTCTRL_CS_SET_PROC_PARAMS_COMPLETE        = 0x004B,
  APP_EXTCTRL_CS_APP_DISTANCE_RESULTS            = 0x0060,
  APP_EXTCTRL_CS_APP_DISTANCE_EXTENDED_RESULTS   = 0x0061,
  APP_EXTCTRL_CA_SERVER_CCC_UPDATE               = 0x0080,
  APP_EXTCTRL_CA_SERVER_INDICATION_CNF           = 0x0081,
  APP_EXTCTRL_CA_SERVER_LONG_WRITE_DONE          = 0x0082,
  APP_EXTCTRL_PAIRING_MAX_NUM_CHAR_CFG           = 0x0090,
  APP_EXTCTRL_PAIRING_READ_BOND                  = 0x0091,
  APP_EXTCTRL_PAIRING_WRITE_BOND                 = 0x0092,
  APP_EXTCTRL_PAIRING_GET_LOCAL_OOB_DATA         = 0x0093,
  APP_EXTCTRL_RREQ_DATA_READY                    = 0x00A0,
  APP_EXTCTRL_RREQ_STATUS                        = 0x00A1,
  APP_EXTCTRL_RRSP_SEND_CS_ENABLE_EVENT          = 0x00B1,
  APP_EXTCTRL_RRSP_SEND_CS_EVENT                 = 0x00B2,
  APP_EXTCTRL_RRSP_SEND_CS_EVENT_CONT            = 0x00B3,
  APP_EXTCTRL_LINK_PARAM_UPDATE_REJECT_EVENT     = 0x00B4,
  APP_EXTCTRL_FAILURE                            = 0xFFFE,
} AppExtCtrlEvent_e;

/*********************************************************************
 * CONSTANTS
 */
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
#define SCAN_MAX_REPORT           APP_MAX_NUM_OF_ADV_REPORTS
#endif // ! (defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) ) )

/*********************************************************************
 * TYPEDEFS
 */
// The external control event handler prototype, the external control module will register
// to the application with event handler with this prototype.
typedef void (*ExtCtrl_eventHandler_t)(BLEAppUtil_eventHandlerType_e eventType, uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

// This is a registration function which will determine where to register the event handler
typedef void (*ExtCtrl_eventHandlerRegister_t)(ExtCtrl_eventHandler_t eventHandler);

/*********************************************************************
 * Structures
 */

// External application control Host message structure
typedef struct
{
  uint16_t            dataLen;      //!< Length of the data
  uint8_t             appSpecifier; //!< Application specifier
  uint8_t             cmdOp;        //!< Command operation
  uint8_t             cmdType;      //!< Command type
  uint8_t             *pData;       //!< Pointer to the data
} appMsg_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e  event;        //!< Event type
  uint8_t         appSpecifier;    //!< Application specifier
  uint8_t         cmdOp;           //!< Command operation
  uint8_t         errCode;         //!< Error code
} AppExtCtrlErrorEvent_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e  event;      //!< Event type
  uint8_t            advHandle;  //!< Advertising handle
} AppExtCtrlAdvStatusEvt_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;        //!< Event type
  uint8_t           cmdOp;        //!< Command operation
  uint8_t           status;       //!< Status of the event
} AppExtCtrlRRSPStatus_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;
  uint8_t           status;       //!< @ref GAP_ConnEvtStat_t
  uint16_t          handle;       //!< connection handle
  uint8_t           channel;      //!< BLE RF channel index (0-39)
  uint8_t           phy;          //!< @ref GAP_ConnEvtPhy_t
  int8_t            lastRssi;     //!< RSSI of last packet received
  uint16_t          packets;      //!<  Number of packets received for this connection event
  uint16_t          errors;       //!< Total number of CRC errors for the entire connection
  uint16_t          nextTaskType; //!< Type of next BLE task @ref GAP_ConnEvtTaskType_t
  uint32_t          nextTaskTime; //!< Time to next BLE task (in us). 0xFFFFFFFF if there is no next task.
  uint16_t          eventCounter; //!< event Counter
  uint32_t          timeStamp;    //!< timestamp (anchor point)
  uint8_t           eventType;    //!< @ref GAP_CB_Event_e.
} AppExtCtrlConnCbEvt_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e  event;              //!< Event type
  GAP_Addr_Modes_t   devAddressMode;     //!< Device address mode
  uint8_t            idAddr[B_ADDR_LEN]; //!< Identity address
  uint8_t            rpAddr[B_ADDR_LEN]; //!< Resolvable private address
} AppExtCtrlConnDevAddressEvt_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e  event;        //!< Event type
  uint8_t     cmdStatus;           //!< Command status
  uint8_t     numHciCmdPkt;        //!< Number of HCI command packets
  uint16_t    cmdOpcode;           //!< Command opcode
} AppExtCtrlCommandStatusEvt_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;         //!< Event type
  uint8_t           BLEEventCode;  //!< BLE event code
  uint16_t          len;           //!< Length of the data
  uint8_t           pData[];       //!< Pointer to the data
} AppExtCtrlHciLeEvent_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;         //!< Event type
  uint16_t          BLEEventCode;  //!< BLE event code
  uint16_t          len;           //!< Length of the data
  uint8_t           pData[];       //!< Pointer to the data
} AppExtCtrlHciCompleteEvent_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e  event;              //!< Event type
  uint8_t  status;                       //!< Status of the event
  uint8_t  opcode;                       //!< Opcode of the event
  uint8_t  devAddrType;                  //!< Device address type
  uint8_t  devAddr[B_ADDR_LEN];          //!< Device address
  uint16_t connectionHandle;             //!< Connection handle
  uint8_t  connRole;                     //!< Connection role
  uint16_t connInterval;                 //!< Connection interval
  uint16_t connLatency;                  //!< Connection latency
  uint16_t connTimeout;                  //!< Connection timeout
  uint8_t  clockAccuracy;                //!< Clock accuracy
} AppExtCtrlEstLinkEvent_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e  event;              //!< Event type
  uint8_t  status;                       //!< Status of the event
  uint8_t  opcode;                       //!< Opcode of the event
  uint16_t connectionHandle;             //!< Connection handle
  uint8_t  reason;                       //!< Reason for termination
} AppExtCtrlTerminateLinkEvent_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e  event;              //!< Event type
  uint8_t  status;                       //!< Status of the event
  uint8_t  opcode;                       //!< Opcode of the event
  uint16_t connectionHandle;             //!< Connection handle
  uint16_t connInterval;                 //!< Connection interval
  uint16_t connLatency;                  //!< Connection latency
  uint16_t connTimeout;                  //!< Connection timeout
} AppExtCtrlParamUpdateEvent_t;

typedef struct __attribute__((packed))
{
  uint8_t  devAddrType;                  //!< Device address type
  uint8_t  devAddr[B_ADDR_LEN];          //!< Device address
} AppExtCtrlConnInfo_t;

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;     //!< Event type
  uint8_t        reason;       //!< Reason for scan disable
  uint8_t        numOfReports; //!< Number of reports
} AppExtCtrlScanDisabledEventHdr_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlScanDisabledEventHdr_t hdr;                          //!< Header of the scan disabled event
  AppExtCtrlConnInfo_t             advertisers[SCAN_MAX_REPORT]; //!< Advertisers information
} AppExtCtrlScanDisabledEvent_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e   event;
  AppExtCtrlConnInfo_t connInfo;
  uint16_t  dataLen;
  uint8_t   pData[];
} AppExtCtrlAdvReportEvent_t;

#endif // ! (defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) ) )

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e  event;        //!< Event type
  uint16_t  connHandle;            //!< Connection handle
  uint8_t   state;                 //!< Pairing state
  uint8_t   status;                //!< Status of the pairing
} AppExtCtrlPairingStateEvent_t;

#if defined (BLE_V41_FEATURES) && (BLE_V41_FEATURES & L2CAP_COC_CFG)
typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;         //!< Event type
  uint16_t          connHandle;    //!< Connection handle
  uint16_t          CID;           //!< Connection Identifier
  uint16_t          len;           //!< Length of the data
  uint8_t           pData[];       //!< Pointer to the data
} AppExtCtrlL2capCocDataEvent_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;         //!< Event type
  uint16_t result;                 //!< Result of the operation
  uint16_t CID;                    //!< Connection Identifier
  uint16_t psm;                    //!< Protocol/Service Multiplexer
  uint16_t mtu;                    //!< Maximum Transmission Unit
  uint16_t mps;                    //!< Maximum PDU Size
  uint16_t credits;                //!< Number of credits
  uint16_t peerCID;                //!< Peer Connection Identifier
  uint16_t peerMtu;                //!< Peer Maximum Transmission Unit
  uint16_t peerMps;                //!< Peer Maximum PDU Size
  uint16_t peerCredits;            //!< Peer number of credits
  uint16_t peerCreditThreshold;    //!< Peer credit threshold
} AppExtCtrlL2capChannelEstEvent_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;         //!< Event type
  uint16_t CID;                    //!< Connection Identifier
  uint16_t peerCID;                //!< Peer Connection Identifier
  uint16_t reason;                 //!< Reason for termination
} AppExtCtrlL2capChannelTermEvent_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;         //!< Event type
  uint16_t CID;                    //!< Connection Identifier
  uint16_t peerCID;                //!< Peer Connection Identifier
  uint16_t credits;                //!< Number of credits
} AppExtCtrlL2capCreditEvent_t;

#endif // BLE_V41_FEATURES & L2CAP_COC_CFG

/*********************************************************************
 * Functions
 */

/*********************************************************************
 * @fn      CommonExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the common module
 *          to the external control dispatcher, and register the call back
 *          event handler function to the common application.
 *
 * @return  SUCCESS/FAILURE
 */
bStatus_t CommonExtCtrl_start(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_EXTCTRL_COMMON_H */
