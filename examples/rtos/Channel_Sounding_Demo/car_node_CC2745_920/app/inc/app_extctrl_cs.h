/******************************************************************************

@file  app_extctrl_cs.h

@brief This header file defines the structures, constants, and function prototypes
       for the external control of the Channel Sounding (CS) module. It includes
       definitions for handling various CS events, such as reading local and remote
       capabilities, configuring CS settings, enabling security, and managing CS
       procedures. The corresponding source file, app_extctrl_cs.c, implements the
       parsing and processing of messages from the external control dispatcher module
       and builds events to be sent back to the dispatcher.

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

#ifndef APP_CS_PARSER_H
#define APP_CS_PARSER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <app_cs_api.h>

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * Enumerators
 */

/*********************************************************************
 * Command Structures
 */
PACKED_TYPEDEF_STRUCT
{
  uint16_t     connHandle;
} AppExtCtrl_csReadRemoteCapCmdParams_t;

PACKED_TYPEDEF_STRUCT
{
  uint16_t connHandle;          //!< Connection handle
  uint8_t  configID;            //!< Configuration ID
  uint8_t  createContext;       //!< Create context flag
  uint8_t mainMode;             //!< Main mode @ref CS_Mode
  uint8_t subMode;              //!< Sub mode @ref CS_Mode
  uint8_t mainModeMinSteps;     //!< Minimum steps for main mode
  uint8_t mainModeMaxSteps;     //!< Maximum steps for main mode
  uint8_t mainModeRepetition;   //!< Main mode repetition
  uint8_t modeZeroSteps;        //!< Steps for mode zero
  uint8_t role;                 //!< Role @ref CS_Role
  uint8_t rttType;              //!< RTT type @ref CS_RTT_Type
  uint8_t csSyncPhy;            //!< CS sync PHY @ref CS_Sync_Phy_Supported
  csChm_t channelMap;           //!< Channel map @ref csChm_t
  uint8_t chMRepetition;        //!< Channel map repetition
  uint8_t chSel;                //!< Channel selection algorithm to be used @ref CS_Chan_Sel_Alg
  uint8_t ch3cShape;            //!< Channel 3C shape
  uint8_t ch3CJump;             //!< Channel 3C jump
} AppExtCtrl_csCreateConfigCmdParams_t;

PACKED_TYPEDEF_STRUCT
{
  uint16_t connHandle;             //!< Connection handle
} AppExtCtrl_csSecurityEnableCmdParams_t;

PACKED_TYPEDEF_STRUCT
{
  uint16_t connHandle;              //!< Connection handle
  uint8_t  roleEnable;              //!< Role enable flag
  uint8_t  csSyncAntennaSelection;  //!< CS sync antenna selection
  int8_t   maxTxPower;              //!< Maximum TX power in dBm
} AppExtCtrl_csSetDefaultSettingsCmdParams_t;

PACKED_TYPEDEF_STRUCT
{
  uint16_t connHandle;             //!< Connection handle
} AppExtCtrl_csReadRemoteFAETableCmdParams_t;

PACKED_TYPEDEF_STRUCT
{
  uint16_t         connHandle;          //!< Connection handle
  csFaeTbl_t       remoteFaeTable;      //!< Pointer to the remote FAE table
} AppExtCtrl_csWriteRemoteFAETableCmdParams_t;

PACKED_TYPEDEF_STRUCT
{
  uint16_t connHandle;                //!< Connection handle
  uint8_t  configID;                  //!< Configuration ID
} AppExtCtrl_csRemoveConfigCmdParams_t;

PACKED_TYPEDEF_STRUCT
{
  csChm_t channelClassification;     //!< Channel map classification
} AppExtCtrl_csSetChannelClassificationCmdParams_t;

PACKED_TYPEDEF_STRUCT
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
  uint8_t phy;                     //!< PHY @ref CS_Phy_Supported
  uint8_t  txPwrDelta;             //!< Tx Power Delta, in signed dB
  uint8_t  preferredPeerAntenna;   //!< Preferred peer antenna
  uint8_t  snrCtrlI;               //!< SNR Control Initiator
  uint8_t  snrCtrlR;               //!< SNR Control Reflector
  uint8_t  enable;                 //!< Is procedure enabled @ref CS_Enable
} AppExtCtrl_csSetProcedureParamsCmdParams_t;

PACKED_TYPEDEF_STRUCT
{
  uint16_t    connHandle;           //!< Connection handle
  uint8_t     configID;             //!< Configuration ID
  uint8_t     enable;               //!< Enable or disable the procedure @ref CS_Enable
} AppExtCtrl_csSetProcedureEnableCmdParams_t;

PACKED_TYPEDEF_STRUCT
{
  uint8_t   defaultAntennaIndex;         //!< Index of the antenna to set as a default antenna for common BLE communications
} AppExtCtrl_csSetDefaultAntennaCmdParams_t;

/*********************************************************************
 * Event Structures
 */
PACKED_TYPEDEF_STRUCT
{
  AppExtCtrlEvent_e event;         //!< Event type
  uint16_t connHandle;             //!< Connection handle
  csStatus_e csStatus;             //!< CS status @ref csStatus_e
  uint8_t  numConfig;              //!< Number of CS configurations supported per conn
  uint16_t maxProcedures;          //!< Max num of CS procedures supported
  uint8_t  numAntennas;            //!< The number of antenna elements that are available for CS tone exchanges
  uint8_t  maxAntPath;             //!< Max number of antenna paths that are supported
  uint8_t  role;                   //!< Initiator or reflector or both @ref CS_Role
  uint8_t  optionalModes;          //!< Indicates which of the optional CS modes are supported @ref csMode_e
  uint8_t  rttCap;                 //!< Indicate which of the time-of-flight accuracy requirements are met
  uint8_t  rttAAOnlyN;             //!< Number of CS steps of single packet exchanges needed
  uint8_t  rttSoundingN;           //!< Number of CS steps of single packet exchanges needed
  uint8_t  rttRandomPayloadN;      //!< Num of CS steps of single packet exchange needed
  uint16_t nadmSounding;           //!< NADM Sounding Capability
  uint16_t nadmRandomSeq;          //!< NADM Random Sequence Capability
  uint8_t  optionalCsSyncPhy;      //!< Supported CS sync PHYs, bit mapped field @ref CS_Sync_Phy_Supported
  uint8_t  noFAE;                  //!< No FAE support
  uint8_t  chSel3c;                //!< Channel selection 3c support
  uint8_t  csBasedRanging;         //!< CS based ranging support
  uint16_t tIp1Cap;                //!< tIP1 Capability
  uint16_t tIp2Cap;                //!< tTP2 Capability
  uint16_t tFcsCap;                //!< tFCS Capability
  uint16_t tPmCsap;                //!< tPM Capability
  uint8_t  tSwCap;                 //!< Antenna switch time capability
  uint8_t  snrTxCap;               //!< Spec defines an additional byte for RFU
} AppExtCtrlCsReadRemoteCapabEvent_t;

PACKED_TYPEDEF_STRUCT
{
  AppExtCtrlEvent_e event;           //!< Event type
  uint16_t   connHandle;             //!< Connection handle
  csStatus_e csStatus;               //!< CS status
  uint8_t    configId;               //!< CS configuration ID
  uint8_t    state;                  //!< 0b00 disabled, 0b01 enabled
  uint8_t    mainMode;               //!< Which CS modes are to be used @ref CS_Mode
  uint8_t    subMode;                //!< Which CS modes are to be used @ref CS_Mode
  uint8_t    mainModeMinSteps;       //!< Range of Main_Mode steps to be executed before
                                     //!< A Sub_Mode step is executed
  uint8_t    mainModeMaxSteps;       //!< Range of Main_Mode steps to be executed before
  uint8_t    mainModeRepetition;     //!< Num of main mode steps from the last CS subevent to be repeated
  uint8_t    modeZeroSteps;          //!< Number of mode 0 steps to be included at the beginning of each CS Subevent
  uint8_t    role;                   //!< Initiator or reflector role @ref CS_Role
  uint8_t    rttType;                //!< Which RTT variant is to be used @ref CS_RTT_Type
  uint8_t    csSyncPhy;              //!< Transmit and receive PHY to be used @ref CS_Sync_Phy_Supported
  csChm_t    channelMap;             //!< Channel map @ref csChm_t
  uint8_t    chMRepetition;          //!< Number of times the ChM field will be cycled through
                                     //!< For non-mode 0 steps within a CS procedure
  uint8_t    chSel;                  //!< Channel selection algorithm to be used @ref CS_Chan_Sel_Alg
  uint8_t    ch3cShape;              //!< Selected shape to be rendered
  uint8_t    ch3CJump;               //!< One of the valid CSChannelJump values defined in table 32
  uint8_t    rfu0;                   //!< Reserved for future use
  uint8_t    tIP1;                   //!< Index of the period used between RTT packets
  uint8_t    tIP2;                   //!< Index of the interlude period used between CS tones
  uint8_t    tFCs;                   //!< Index used for frequency changes
  uint8_t    tPM;                    //!< Index for the measurement period of CS tones
  uint8_t    rfu;                    //!< Reserved for future use
} AppExtCtrlCsConfigComplete_t;

PACKED_TYPEDEF_STRUCT
{
  AppExtCtrlEvent_e event;            //!< Event type @ref AppExtCtrlEvent_e
  uint16_t          connHandle;       //!< Connection handle
  csStatus_e        csStatus;         //!< CS status @ref csStatus_e
  csFaeTbl_t        faeTable;         //!< Remote CS FAE table @ref csFaeTbl_t
} AppExtCtrlCsReadRemFAEComplete_t;

PACKED_TYPEDEF_STRUCT
{
  AppExtCtrlEvent_e     event;       //!< Event type @ref AppExtCtrlEvent_e
  uint16_t              connHandle;  //!< Connection handle
  csStatus_e            csStatus;    //!< CS status @ref csStatus_e
} AppExtCtrlCsSecurityEnableComplete_t;

PACKED_TYPEDEF_STRUCT
{
  AppExtCtrlEvent_e event;             //!< Event type @ref AppExtCtrlEvent_e
  csStatus_e        csStatus;          //!< CS status @ref csStatus_e
  uint16_t          connHandle;        //!< Connection handle
  uint8_t           configId;          //!< CS configuration ID
  uint8_t           enable;            //!< Enable/disable @ref CS_Enable
  csACI_e           ACI;               //!< Antenna Configuration Index @ref csACI_e
  int8_t            selectedTxPower;   //!< Transmit power level used for CS procedure. Units: dBm
  uint32_t          subEventLen;       //!< Units microseconds, range 1250us to 4s
  uint8_t           subEventsPerEvent; //!< Num of CS SubEvents in a CS Event
  uint16_t          subEventInterval;  //!< Units 625 us
  uint16_t          eventInterval;     //!< Units of connInt
  uint16_t          procedureInterval; //!< Units of connInt
  uint16_t          procedureCount;    //!< Number of procedures
} AppExtCtrlCsProcEnableComplete_t;

PACKED_TYPEDEF_STRUCT
{
  AppExtCtrlEvent_e event;              //!< Event type @ref AppExtCtrlEvent_e
  uint16_t connHandle;                  //!< Connection handle
  uint8_t  configID;                    //!< Configuration ID
  uint16_t startAclConnectionEvent;     //!< Start ACL connection event
  uint16_t procedureCounter;            //!< Procedure counter
  int16_t  frequencyCompensation;       //!< Frequency compensation
  int8_t   referencePowerLevel;         //!< Reference power level
  uint8_t  procedureDoneStatus;         //!< Procedure done status @ref CS_Procedure_Done_Status
  uint8_t  subeventDoneStatus;          //!< Subevent done status
  uint8_t  abortReason;                 //!< Abort reason @ref CS_Abort_Reason
  uint8_t  numAntennaPath;              //!< Number of antenna paths
  uint8_t  numStepsReported;            //!< Number of steps reported
  uint16_t  dataLen;                    //!< Data length
  uint8_t  data[];                      //!< Data
} AppExtCtrlCsSubeventResults_t;

PACKED_TYPEDEF_STRUCT
{
  AppExtCtrlEvent_e event;          //!< Event type @ref AppExtCtrlEvent_e
  uint16_t connHandle;              //!< Connection handle
  uint8_t  configID;                //!< Configuration ID
  uint8_t  procedureDoneStatus;     //!< Procedure done status @ref CS_Procedure_Done_Status
  uint8_t  subeventDoneStatus;      //!< Subevent done status
  uint8_t  abortReason;             //!< Abort reason @ref CS_Abort_Reason
  uint8_t  numAntennaPath;          //!< Number of antenna paths
  uint8_t  numStepsReported;        //!< Number of steps reported
  uint16_t  dataLen;                //!< Data length
  uint8_t  data[];                  //!< Data
} AppExtCtrlCsSubeventResultsContinue_t;

PACKED_TYPEDEF_STRUCT
{
  AppExtCtrlEvent_e event;  //!< Event type @ref AppExtCtrlEvent_e
  uint8_t  status;          //!< status of the results
  uint16_t connHandle;      //!< Connection Handle
  uint32_t distance;        //!< Estimated distance in [cm].
  uint32_t quality;         //!< Average quality of the ranging measurement.
  uint32_t confidence;      //!< Confidence level of the distance estimation.
} AppExtCtrlCsAppDistanceResultsEvent_t;

PACKED_TYPEDEF_STRUCT
{
    AppExtCtrlEvent_e event;                    //!< Event type @ref AppExtCtrlEvent_e
    uint8_t  status;                            //!< Status of the results
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
    //       if using BTCS, this array is not relevant
    uint8_t permutationIndexRemote[CS_RANGING_PCT_ARRAY_SIZE];

    // Remote steps data, similar to local.
    uint32_t stepsDataRemote[CS_RANGING_PCT_ARRAY_SIZE_PATHS];
} AppExtCtrlCsAppExtendedResultsEvent_t;

/*********************************************************************
 * @fn      CSExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the cs module
 *          to the external control dispatcher, and register the call back
 *          event handler function using the given registration function.
 *
 * @param   fRegisterFun - A registration function to be used by extCtrl in order
 *                         to register its CS event handler function.
 *
 * @return  SUCCESS/FAILURE
 */
bStatus_t CSExtCtrl_start(ExtCtrl_eventHandlerRegister_t fRegisterFun);

#ifdef __cplusplus
}
#endif

#endif /* APP_CS_PARSER_H */
