/******************************************************************************

@file  app_cs_api.h

@brief This file contains the CS APIs and structures.


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

#ifndef APP_CS_API_H
#define APP_CS_API_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/stack_util/cs_types.h"
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include <app_extctrl_common.h>
#include "ti/ble/host/cs/cs.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */


#define CS_RANGING_PCT_ARRAY_SIZE             75      // CS Ranging library PCT results array size for 1 antenna path
#define CS_RANGING_MAX_ANT_PATHS              4       // CS Ranging library maximum number of antenna paths

// CS Ranging library PCT results array size for all antenna paths
#define CS_RANGING_PCT_ARRAY_SIZE_PATHS      (CS_RANGING_PCT_ARRAY_SIZE * CS_RANGING_MAX_ANT_PATHS)


/*********************************************************************
 * MACROS
 */

#define CS_GET_OPPOSITE_ROLE(role)     ((role == CS_ROLE_REFLECTOR) ? CS_ROLE_INITIATOR : CS_ROLE_REFLECTOR)

/*********************************************************************
 * TYPEDEFS
 */

/*
 * Indicates a source of a single subevent results.
 */
typedef enum
{
    CS_RESULTS_MODE_LOCAL       = 0x00, //!< The source of the results is the local device
    CS_RESULTS_MODE_PROP        = 0x01, //!< The source of the results is the remote device, received using L2CAP
    CS_RESULTS_MODE_RAS         = 0x02, //!< The source of the results is the remote device, received using RAS profile
    CS_RESULTS_MODE_BTCS        = 0x03, //!< The source of the results is the remote device, received using L2CAP BTCS
    CS_RESULTS_MODE_END         = 0x04, //!< Indicates the maximum value of this enum
} ChannelSounding_resultsSourceMode_e;

typedef enum
{
  APP_CS_PROCEDURE_ENABLE_COMPLETE_EVENT,  //!< CS Procedure Enable Complete @ref ChannelSounding_procEnableCompleteEvt_t
  APP_CS_SUBEVENT_RESULT                ,  //!< CS Subevent Result @ref ChannelSounding_subeventResultsEvt_t
  APP_CS_SUBEVENT_CONTINUE_RESULT       ,  //!< CS Subevent continue Result @ref ChannelSounding_subeventResultsContinueEvt_t
  APP_CS_SUBEVENT_CONTINUE_RESULT_EXT   ,  //!< CS Subevent continue Result @ref ChannelSounding_subeventResultsContinueExt_t
} ChannelSounding_eventOpcodes_e;

/*
 * Command Structures
 */
typedef struct
{
  uint16_t     connHandle;
} ChannelSounding_readRemoteCapCmdParams_t;

typedef struct
{
  uint16_t     connHandle;              //!< Connection handle
  csCapabilities_t remoteCapabilities;  //!< Remote CS capabilities
} ChannelSounding_writeCachedRemoteCapCmdParams_t;

typedef struct
{
  uint16_t connHandle;              //!< Connection handle
  uint8_t  configID;                //!< Configuration ID
  uint8_t  createContext;           //!< Create context flag
  uint8_t  mainMode;                //!< Main mode @ref CS_Mode
  uint8_t  subMode;                 //!< Sub mode @ref CS_Mode
  uint8_t  mainModeMinSteps;        //!< Minimum steps for main mode
  uint8_t  mainModeMaxSteps;        //!< Maximum steps for main mode
  uint8_t  mainModeRepetition;      //!< Main mode repetition
  uint8_t  modeZeroSteps;           //!< Steps for mode zero
  uint8_t  role;                    //!< Role initiator/reflector @ref CS_Role
  uint8_t  rttType;                 //!< RTT type @ref CS_RTT_Type
  uint8_t  csSyncPhy;               //!< CS sync PHY @ref CS_Sync_Phy_Supported
  csChm_t  channelMap;              //!< Channel map
  uint8_t  chMRepetition;           //!< Channel map repetition
  uint8_t  chSel;                   //!< Channel selection algorithm to be used @ref CS_Chan_Sel_Alg
  uint8_t  ch3cShape;               //!< Channel 3C shape
  uint8_t  ch3CJump;                //!< Channel 3C jump
} ChannelSounding_createConfigCmdParams_t;

typedef struct
{
  uint16_t connHandle;             //!< Connection handle
} ChannelSounding_securityEnableCmdParams_t;

typedef struct
{
  uint16_t connHandle;              //!< Connection handle
  uint8_t  roleEnable;              //!< Role enable flag
  uint8_t  csSyncAntennaSelection;  //!< CS sync antenna selection
  int8_t   maxTxPower;              //!< Maximum TX power in dBm
} ChannelSounding_setDefaultSettingsCmdParams_t;

typedef struct
{
  uint16_t connHandle;              //!< Connection handle
} ChannelSounding_readRemoteFAETableCmdParams_t;

typedef struct
{
  uint16_t    connHandle;       //!< Connection handle
  csFaeTbl_t  remoteFaeTable;   //!< Remote FAE table
} ChannelSounding_writeCachedRemoteFAETableCmdParams_t;

typedef struct
{
  uint16_t connHandle;           //!< Connection handle
  uint8_t  configID;             //!< Configuration ID
} ChannelSounding_removeConfigCmdParams_t;

typedef struct
{
  csChm_t channelClassification;   //!< Channel classification
} ChannelSounding_SetChannelClassificationCmdParams_t;

typedef struct
{
  uint16_t connHandle;             //!< Connection handle
  uint8_t  configID;               //!< Configuration ID
  uint16_t maxProcedureDur;        //!< Maximum procedure duration in 0.625 milliseconds
  uint16_t minProcedureInterval;   //!< Minimum number of connection events between consecutive CS procedures
  uint16_t maxProcedureInterval;   //!< Maximum number of connection events between consecutive CS procedures
  uint16_t maxProcedureCount;      //!< Maximum number of CS procedures to be scheduled (0 - indefinite)
  uint32_t minSubEventLen;         //!< Minimum SubEvent length in microseconds, range 1250us to 4s
  uint32_t maxSubEventLen;         //!< Maximum SubEvent length in microseconds, range 1250us to 4s
  csACI_e  aci;                    //!< Antenna Configuration Index @ref csACI_e
  uint8_t  phy;                    //!< PHY @ref CS_Phy_Supported
  uint8_t  txPwrDelta;             //!< Tx Power Delta, in signed dB
  uint8_t  preferredPeerAntenna;   //!< Preferred peer antenna
  uint8_t  snrCtrlI;               //!< SNR Control Initiator
  uint8_t  snrCtrlR;               //!< SNR Control Reflector
  uint8_t  enable;                 //!< Is procedure enabled @ref CS_Enable
} ChannelSounding_setProcedureParamsCmdParams_t;

typedef struct
{
  uint16_t   connHandle;           //!< Connection handle
  uint8_t    configID;             //!< Configuration ID
  uint8_t    enable;               //!< Enable or disable the procedure @ref CS_Enable
} ChannelSounding_setProcedureEnableCmdParams_t;

typedef struct
{
  uint8_t   defaultAntennaIndex;   //!< Index of the antenna to set as a default antenna for common BLE communications
} ChannelSounding_setDefaultAntennaCmdParams_t;

/*
 * Event Structures
 */
typedef struct
{
  uint8_t  csEvtOpcode;            //!< CS Event Code @ref csEventOpcodes_e
  uint16_t connHandle;             //!< Connection handle
  csStatus_e csStatus;             //!< CS status @ref csStatus_e
  uint8_t  numConfig;              //!< Number of CS configurations supported per conn
  uint16_t maxProcedures;          //!< Max num of CS procedures supported
  uint8_t  numAntennas;            //!< The number of antenna elements that are available for CS tone exchanges
  uint8_t  maxAntPath;             //!< Max number of antenna paths that are supported
  uint8_t  role;                   //!< Initiator or reflector or both @ref CS_Role
  uint8_t  optionalModes;          //!< Indicates which of the optional CS modes are supported
  uint8_t  rttCap;                 //!< Indicate which of the time-of-flight accuracy requirements are met
  uint8_t  rttAAOnlyN;             //!< Number of CS steps of single packet exchanges needed
  uint8_t  rttSoundingN;           //!< Number of CS steps of single packet exchanges needed
  uint8_t  rttRandomPayloadN;      //!< Num of CS steps of single packet exchange needed
  uint16_t nadmSounding;           //!< NADM Sounding Capability
  uint16_t nadmRandomSeq;          //!< NADM Random Sequence Capability
  uint8_t  optionalCsSyncPhy;      //!< Supported CS sync PHYs, bit mapped field
  uint8_t  noFAE;                  //!< No FAE support
  uint8_t  chSel3c;                //!< Channel selection 3c support
  uint8_t  csBasedRanging;         //!< CS based ranging support
  uint16_t tIp1Cap;                //!< tIP1 Capability
  uint16_t tIp2Cap;                //!< tTP2 Capability
  uint16_t tFcsCap;                //!< tFCS Capability
  uint16_t tPmCsap;                //!< tPM Capability
  uint8_t  tSwCap;                 //!< Antenna switch time capability
  uint8_t  snrTxCap;               //!< Spec defines an additional byte for RFU
} ChannelSounding_readRemoteCapabEvent_t;

typedef struct
{
  uint8_t          csEvtOpcode;        //!< CS Event Code @ref csEventOpcodes_e
  uint16_t         connHandle;         //!< Connection handle
  csStatus_e       csStatus;           //!< CS status @ref csStatus_e
  uint8_t          configId;           //!< CS configuration ID
  uint8_t          state;              //!< 0b00 disabled, 0b01 enabled
  uint8_t          mainMode;           //!< Which CS modes are to be used @ref CS_Mode
  uint8_t          subMode;            //!< Which CS modes are to be used @ref CS_Mode
  uint8_t          mainModeMinSteps;   //!< Range of Main_Mode steps to be executed before
                                       //!< A Sub_Mode step is executed
  uint8_t          mainModeMaxSteps;   //!< Range of Main_Mode steps to be executed before
  uint8_t          mainModeRepetition; //!< Num of main mode steps from the last CS subevent to be repeated
  uint8_t          modeZeroSteps;      //!< Number of mode 0 steps to be included at the beginning of each CS Subevent
  uint8_t          role;               //!< Initiator or reflector role @ref CS_Role
  uint8_t          rttType;            //!< Which RTT variant is to be used @ref CS_RTT_Type
  uint8_t          csSyncPhy;          //!< Transmit and receive PHY to be used @ref CS_Sync_Phy_Supported
  csChm_t          channelMap;         //!< Channel map @ref csChm_t
  uint8_t          chMRepetition;      //!< Number of times the ChM field will be cycled through
                                       //!< For non-mode 0 steps within a CS procedure
  uint8_t          chSel;              //!< Channel selection algorithm to be used @ref CS_Chan_Sel_Alg
  uint8_t          ch3cShape;          //!< Selected shape to be rendered
  uint8_t          ch3CJump;           //!< One of the valid CSChannelJump values defined in table 32
  uint8_t          rfu0;               //!< Reserved for future use
  uint8_t          tIP1;               //!< Index of the period used between RTT packets
  uint8_t          tIP2;               //!< Index of the interlude period used between CS tones
  uint8_t          tFCs;               //!< Index used for frequency changes
  uint8_t          tPM;                //!< Index for the measurement period of CS tones
  uint8_t          rfu;                //!< Reserved for future use
} ChannelSounding_configComplete_t;

typedef struct
{
  uint8_t        csEvtOpcode;      //!< CS Event Code @ref csEventOpcodes_e
  uint16_t       connHandle;       //!< Connection handle
  csStatus_e     csStatus;         //!< CS status @ref csStatus_e
  csFaeTbl_t     faeTable;         //!< Remote CS FAE table @ref csFaeTbl_t
} ChannelSounding_readRemFAEComplete_t;

typedef struct
{
  uint8_t       csEvtOpcode;      //!< CS Event Code @ref csEventOpcodes_e
  uint16_t      connHandle;       //!< Connection handle
  csStatus_e    csStatus;         //!< CS status @ref csStatus_e
} ChannelSounding_securityEnableComplete_t;

typedef struct
{
  uint8_t          csEvtOpcode;       //!< CS Event Code @ref csEventOpcodes_e
  csStatus_e       csStatus;          //!< CS status @ref csStatus_e
  uint16_t         connHandle;        //!< Connection handle
  uint8_t          configId;          //!< CS configuration ID
  uint8_t          enable;            //!< Enable/disable @ref CS_Enable
  csACI_e          ACI;               //!< Antenna Configuration Index @ref csACI_e
  int8_t           selectedTxPower;   //!< Transmit power level used for CS procedure. Units: dBm
  uint32_t         subEventLen;       //!< Units microseconds, range 1250us to 4s
  uint8_t          subEventsPerEvent; //!< Num of CS SubEvents in a CS Event
  uint16_t         subEventInterval;  //!< Units 625 us
  uint16_t         eventInterval;     //!< Units of connInt
  uint16_t         procedureInterval; //!< Units of connInt
  uint16_t         procedureCount;    //!< Number of procedures
} ChannelSounding_procEnableComplete_t;

typedef struct
{
  uint8_t  csEvtOpcode;             //!< CS Event Code @ref csEventOpcodes_e
  uint16_t connHandle;              //!< Connection handle
  uint8_t  configID;                //!< Configuration ID
  uint16_t startAclConnectionEvent; //!< Start ACL connection event
  uint16_t procedureCounter;        //!< Procedure counter
  int16_t  frequencyCompensation;   //!< Frequency compensation
  int8_t   referencePowerLevel;     //!< Reference power level
  uint8_t  procedureDoneStatus;     //!< Procedure done status @ref CS_Procedure_Done_Status
  uint8_t  subeventDoneStatus;      //!< Subevent done status
  uint8_t  abortReason;             //!< Abort reason @ref CS_Abort_Reason
  uint8_t  numAntennaPath;          //!< Number of antenna paths
  uint8_t  numStepsReported;        //!< Number of steps reported
  uint16_t dataLen;                 //!< Data length
  uint8_t  data[];                  //!< Data
} ChannelSounding_subeventResults_t;

typedef struct
{
  uint8_t  csEvtOpcode;          //!< CS Event Code @ref csEventOpcodes_e
  uint16_t connHandle;           //!< Connection handle
  uint8_t  configID;             //!< Configuration ID
  uint8_t  procedureDoneStatus;  //!< Procedure done status @ref CS_Procedure_Done_Status
  uint8_t  subeventDoneStatus;   //!< Subevent done status
  uint8_t  abortReason;          //!< Abort reason @ref CS_Abort_Reason
  uint8_t  numAntennaPath;       //!< Number of antenna paths
  uint8_t  numStepsReported;     //!< Number of steps reported
  uint16_t  dataLen;             //!< Data length
  uint8_t  data[];               //!< Data
} ChannelSounding_subeventResultsContinue_t;

typedef struct
{
  uint8_t  csEvtOpcode;          //!< CS Event Code @ref csEventOpcodes_e
  uint16_t connHandle;           //!< Connection handle
  uint8_t  configID;             //!< Configuration ID
  uint16_t procedureCounter;     //!< Procedure counter
  uint8_t  procedureDoneStatus;  //!< Procedure done status @ref CS_Procedure_Done_Status
  uint8_t  subeventDoneStatus;   //!< Subevent done status
  uint8_t  abortReason;          //!< Abort reason @ref CS_Abort_Reason
  uint8_t  numAntennaPath;       //!< Number of antenna paths
  uint8_t  numStepsReported;     //!< Number of steps reported
  uint16_t  dataLen;             //!< Data length
  uint8_t  data[];               //!< Data
} ChannelSounding_subeventResultsContinueExt_t;

// Channel Sounding Application Events of type: @ref BLEAPPUTIL_CS_APP_EVENT_CODE
typedef enum
{
    CS_APP_DISTANCE_RESULTS_EVENT,          //!< CS procedure distance results reported
    CS_APP_DISTANCE_EXTENDED_RESULTS_EVENT, //!< CS procedure distance extended results
    // Add new events here
} csAppEventOpcodes_e;

// Channel Sounding Application Distance Results Event of type: @ref CS_APP_DISTANCE_RESULTS_EVENT
typedef struct
{
  uint8_t      csEvtOpcode;     //!< CS App event opcode of @ref CS_APP_DISTANCE_RESULTS_EVENT
  uint8_t      status;          //!< status of the results
  uint16_t     connHandle;      //!< Connection Handle
  uint32_t     distance;        //!< Estimated distance in [cm].
  uint32_t     quality;         //!< Average quality of the ranging measurement.
  uint32_t     confidence;      //!< Confidence level of the distance estimation.
} ChannelSounding_appDistanceResultsEvent_t;

// Channel Sounding Application Distance Extended Results Event of type: @ref CS_APP_DISTANCE_EXTENDED_RESULTS_EVENT
typedef struct
{
    uint8_t  csEvtOpcode;                       //!< CS App event opcode of @ref CS_APP_DISTANCE_EXTENDED_RESULTS_EVENT
    uint8_t  status;                            //!< status of the results
    uint16_t connHandle;                        //!< Connection Handle
    uint32_t distance;                          //!< Estimated distance in [cm].
    uint32_t quality;                           //!< Average quality of the ranging measurement.
    uint32_t confidence;                        //!< Confidence level of the distance estimation.
    uint16_t numMpc;                            //!< Number of multipath
    uint32_t distanceMusic[CS_MAX_ANT_PATHS];   //!< Estimated distance by music per path
    uint32_t distanceNN[CS_MAX_ANT_PATHS];      //!< Estimated distance by NN per path
    uint16_t numMpcPaths[CS_MAX_ANT_PATHS];     //!< Number of multipath per path
    uint32_t qualityPaths[CS_MAX_ANT_PATHS];    //!< Quality of measurement per path
    uint32_t confidencePaths[CS_MAX_ANT_PATHS]; //!< Confidence of estimation per path

    // Local / Remote RPL (Reference Power Level) for each subevent
    int8_t localRpl[CS_MAX_SUBEVENTS_PER_PROCEDURE];
    int8_t remoteRpl[CS_MAX_SUBEVENTS_PER_PROCEDURE];

    // Raw results data
    CS_modeZeroInitStep_t modeZeroStepsInit[CS_MAX_MODE_ZERO_PER_PROCEDURE]; //!< Initiator mode-0 steps. Ordered by step index
    CS_modeZeroReflStep_t modeZeroStepsRef[CS_MAX_MODE_ZERO_PER_PROCEDURE];  //!< Reflector mode-0 steps. Ordered by step index

    // Contains Local device steps antenna permutations, 75 steps.
    // Ordered by channel numbers, starts from channel 2.
    uint8_t permutationIndexLocal[CS_RANGING_PCT_ARRAY_SIZE];

    // Contains Local device steps data: (I, Q, Quality), max 75 steps for max 4 antenna paths.
    // Ordered by channel numbers for each antenna path.
    // Steps starts from channel 2.
    // Paths starts from path 1 to 4.
    uint32_t stepsDataLocal[CS_RANGING_PCT_ARRAY_SIZE_PATHS];

    // Remote antenna permutation index, similar to local.
    // Note: Depends on the protocol used for receiving the remote data,
    //       if using CCC, this array is not relevant
    uint8_t permutationIndexRemote[CS_RANGING_PCT_ARRAY_SIZE];

    // Remote steps data, similar to local.
    uint32_t stepsDataRemote[CS_RANGING_PCT_ARRAY_SIZE_PATHS];
} ChannelSounding_appExtendedResultsEvent_t;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      ChannelSounding_registerEvtHandler
 *
 * @brief   This function is called to register @ref ExtCtrl_eventHandler_t.
 *
 * @return  None
 */
void ChannelSounding_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler);

/*********************************************************************
 * @fn      ChannelSounding_readLocalSupportedCapabilities
 *
 * @brief   This function is used to read the local Supported CS capabilities.
 *          @ref csCapabilities_t is passed and filled with the local capabilities.
 *
 * @return  @ref SUCCESS
 */
bStatus_t ChannelSounding_readLocalSupportedCapabilities(void);

/*********************************************************************
 * @fn      ChannelSounding_readRemoteSupportedCapabilities
 *
 * @brief   This function is used to read the remote Supported CS capabilities.
 *
 * @param   pReadRSCParams - pointer to the read remote supported capabilities parameters.
 *
 * @return  @ref SUCCESS
 * @return  @ref FAILURE
 */
bStatus_t ChannelSounding_readRemoteSupportedCapabilities(ChannelSounding_readRemoteCapCmdParams_t* pReadRSCParams);

/*********************************************************************
 * @fn      ChannelSounding_writeCachedRemoteSupportedCapabilities
 *
 * @brief   This function is used to write cached remote Supported CS capabilities.
 *
 * @param   pReadRSCParams - pointer to the remote supported capabilities parameters.
 *
 * @return  @ref SUCCESS
 * @return  @ref FAILURE
 */
bStatus_t ChannelSounding_writeCachedRemoteSupportedCapabilities(ChannelSounding_writeCachedRemoteCapCmdParams_t* pWriteCRSCParams);


/*********************************************************************
 * @fn      ChannelSounding_createConfig
 *
 * @brief   This function creates a new CS configuration both locally and on the peer device.
 *
 * @param   pCreateConfigParams - pointer to the create config params.
 *
 * @return  @ref SUCCESS
 * @return  @ref FAILURE
 */
bStatus_t ChannelSounding_createConfig(ChannelSounding_createConfigCmdParams_t* pCreateConfigParams);

/*********************************************************************
 * @fn      ChannelSounding_securityEnable
 *
 * @brief   This function is used to start or restart the CS security procedure.
 *
 * @param   pCSsecurityEnableParams - pointer to the security enable params.
 *
 * @return  @ref SUCCESS
 * @return  @ref FAILURE
 */
bStatus_t ChannelSounding_securityEnable(ChannelSounding_securityEnableCmdParams_t* pCSsecurityEnableParams);

/*********************************************************************
 * @fn      ChannelSounding_setDefaultSettings
 *
 * @brief   This function is used to set default CS settings in the local device.
 *
 * @param   pSetDefaultSettingsParams - pointer to the set default settings params.
 *
 * @return  @ref SUCCESS
 * @return  @ref FAILURE
 */
bStatus_t ChannelSounding_setDefaultSettings(ChannelSounding_setDefaultSettingsCmdParams_t* pSetDefaultSettingsParams);

/*********************************************************************
 * @fn      ChannelSounding_readRemoteFAETable
 *
 * @brief   This function is used to write the per-channel Mode 0 FAE table of
 *          the remote device in a reflector role.
 *
 * @param   pReadRemoteFAETableParams - pointer to the read remote FAE table params.
 *
 * @return  @ref SUCCESS
 * @return  @ref FAILURE
 */
bStatus_t ChannelSounding_readRemoteFAETable(ChannelSounding_readRemoteFAETableCmdParams_t* pReadRemoteFAETableParams);

/*********************************************************************
 * @fn      ChannelSounding_writeCachedRemoteFAETable
 *
 * @brief   This function is used to write the per-channel Mode 0 FAE table of
 *          the remote Controller in a reflector role.
 *
 * @param   pWriteRemoteFAETableParams - pointer to the write remote FAE table params.
 *
 * @return  @ref SUCCESS
 * @return  @ref FAILURE
 */
bStatus_t ChannelSounding_writeCachedRemoteFAETable(ChannelSounding_writeCachedRemoteFAETableCmdParams_t* pWriteRemoteFAETableParams);

/*********************************************************************
 * @fn      ChannelSounding_removeConfig
 *
 * @brief   This function is used to remove a CS configuration from the local device.
 *
 * @param   pRemoveConfigParams - pointer to the write remove config params.
 *
 * @return  @ref SUCCESS
 * @return  @ref FAILURE
 */
bStatus_t ChannelSounding_removeConfig(ChannelSounding_removeConfigCmdParams_t* pRemoveConfigParams);

/*********************************************************************
 * @fn      ChannelSounding_setChannelClassification
 *
 * @brief   This function is used to to update the channel classification based on its
 *          local information.
 *
 * @param   pSetChannelClassificationParams - pointer to the set channel classification params.
 *
 * @return  @ref SUCCESS
 * @return  @ref FAILURE
 */
bStatus_t ChannelSounding_setChannelClassification(ChannelSounding_SetChannelClassificationCmdParams_t* pSetChannelClassificationParams);

/*********************************************************************
 * @fn      ChannelSounding_setProcedureParameters
 *
 * @brief   This function is used to set the parameters of one or more CS procedures by
 *          the local device with the remote device.
 *
 * @param   pSetProcedureParametersParams - pointer to the set procedure parameters params.
 *
 * @return  @ref SUCCESS
 * @return  @ref FAILURE
 */
bStatus_t ChannelSounding_setProcedureParameters(ChannelSounding_setProcedureParamsCmdParams_t* pSetProcedureParametersParams);

/*********************************************************************
 * @fn      ChannelSounding_procedureEnable
 *
 * @brief   This function is used to enable or disable the CS procedures
 *          by the local device with the remote device
 *
 * @param   pSetProcedureEnableParams - pointer to the procedure enable params.
 *
 * @return  @ref SUCCESS
 * @return  @ref FAILURE
 */
bStatus_t ChannelSounding_procedureEnable(ChannelSounding_setProcedureEnableCmdParams_t* pSetProcedureEnableParams);

/*********************************************************************
 * @fn          ChannelSounding_setDefaultAntenna
 *
 * @brief       This function is used to set the default antenna to be
 *              used for common BLE communications.
 *
 * @param       pSetDefaultAntennaParams - pointer to the index parameter.
 *
 * @return      @ref CS_STATUS_SUCCESS - The parameters has been set successfully
 *              @ref CS_STATUS_UNEXPECTED_PARAMETER - The given index is out of range
 *              @ref CS_STATUS_COMMAND_DISALLOWED - The device is not in Idle mode.
 *              @ref CS_STATUS_INVOKE_FUNC_FAIL - Function failed to execute.
 *
 * @attention   It is recommended to Call this function only when the device is idle
 *              to avoid frequent errors.
 *
 * @note        Calling this function is particularly useful during device
 *              initialization to ensure proper antenna configuration.
 */
bStatus_t ChannelSounding_setDefaultAntenna(ChannelSounding_setDefaultAntennaCmdParams_t* pSetDefaultAntennaParams);

/*******************************************************************************
 * @fn           ChannelSounding_getRole
 *
 * @brief        This function gets the role of the local device based on
 *               Channel Sounding connection handle and config id
 *
 * @param        connHandle - Connection Handle
 * @param        configID - CS config Id
 *
 * @return       @ref CS_Mode_Role
 * @return       0xFF If the given connection is not activate or the config Id is invalid
 */
uint8_t ChannelSounding_getRole(uint16_t connHandle, uint8_t configID);

#ifdef __cplusplus
}
#endif

#endif /* APP_CS_API_H */
