/******************************************************************************

@file  app_cs_process_api.h

@brief This file contains the CS Ranging API and its structures.
       The module is used to collect Channel Sounding step results for both
       initiator and reflector, and calculates the distance using an external
       algorithm.

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

#ifndef APP_CS_PROCESS_API_H
#define APP_CS_PROCESS_API_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/stack_util/bcomdef.h"
#include "app_cs_api.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

#define CS_PROCESS_MAX_SESSIONS     MAX_NUM_BLE_CONNS   // Maximum number of sessions
#define CS_PROCESS_INVALID_SESSION  0xFFFF              // Invalid Session ID
#define CS_PROCESS_INVALID_CHANNEL  0xFF                // Invalid channel

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/**
 * @enum csProcessStatus_e
 * @brief Enumerates the possible statuses for the CS process.
 *
 * This enumeration defines the various statuses that can be returned by CS process API functions.
 */
typedef enum csProcessStatus_e {
    CS_PROCESS_SUCCESS = 0x0,                   //!< Operation completed successfully
    CS_PROCESS_GENERAL_FAILURE,                 //!< General failure to be aligned with Stack API, not in use
    CS_PROCESS_INVALID_PARAM,                   //!< Invalid parameter provided
    CS_PROCESS_RESULTS_MODE_NOT_SUPPORTED,      //!< Results mode not supported (e.g. one of @ref ChannelSounding_resultsSourceMode_e modes)
    CS_PROCESS_INVALID_STEP_PARAM,              //!< Invalid step parameter
    CS_PROCESS_SESSION_NOT_OPENED,              //!< Session has not been opened
    CS_PROCESS_PROCEDURE_NOT_ACTIVE,            //!< No procedure currently being processed
    CS_PROCESS_ANOTHER_SESSION_IN_PROCESS,      //!< Another session is currently in process
    CS_PROCESS_MODE_NOT_SUPPORTED,              //!< CS results Main Mode not supported
    CS_PROCESS_TOO_MANY_STEPS_PROVIDED,         //!< Too many steps provided - The number of steps in the subevent exceeds the maximum allowed steps
    CS_PROCESS_STEPS_PROCESSING_FAILED,         //!< Failure during the steps processing
    CS_PROCESS_SUBEVENT_STATUS_INVALID,         //!< Invalid subevent status for steps
    CS_PROCESS_TOO_MANY_SUBEVENTS_PROVIDED,     //!< Too many subevents provided - The number of subevents in the procedure exceeds the maximum allowed subevents
    CS_PROCESS_SUBEVENT_ABORTED,                //!< Subevent was aborted
    CS_PROCESS_PROCEDURE_ABORTED,               //!< Procedure was aborted
    CS_PROCESS_PROCEDURE_PROCESSING_PENDING,    //!< Procedure processing is pending, waiting for more subevent results
    CS_PROCESS_DISTANCE_ESTIMATION_FAILED,      //!< Results processed successfully, distance estimation failed
    CS_PROCESS_DISTANCE_ESTIMATION_PENDING,     //!< All subevent results has been processed and distance may now be estimated
    CS_PROCESS_LOCAL_RESULTS_NOT_READY,         //!< Local subevent results not ready, it is required that the local results will be provided before the remote
} csProcessStatus_e;

#ifdef CS_PROCESS_EXT_RESULTS
/**
 * @brief Structure to hold extended ranging results.
 *
 * Mirrors @ref BleCsRanging_DebugResult_t with integer types.
 * 
 */
typedef struct {
    uint32_t distanceMusic;                            //!< MUSIC algorithm distance estimate, cm (scaled x100 from meters).
    uint32_t distanceNN;                               //!< NN algorithm distance estimate, cm (scaled x100 from meters).
    uint32_t distanceIFFT;                             //!< IFFT algorithm distance estimate, cm (scaled x100 from meters).
    uint32_t confidence;                               //!< Confidence of the estimation (scaled x100).
    uint16_t numMpc;                                   //!< Number of MUSIC multipath components.
    uint32_t qualityPaths[CS_RANGING_MAX_ANT_PATHS];   //!< Quality metric QQ3 per antenna path (scaled x100).
    uint32_t tqiScore[CS_RANGING_MAX_ANT_PATHS];       //!< TQI score per antenna path (scaled x100). Reserved for future use.
    uint32_t dcand;                                    //!< Debug: distance candidate (scaled x100). Reserved for future use.
    uint32_t cf;                                       //!< Debug: correction factor (scaled x100). Reserved for future use.
    uint32_t dVar;                                     //!< Debug: distance variance (scaled x100). Reserved for future use.
    uint16_t classLabel;                               //!< Debug: NN classification label. Reserved for future use.
    uint32_t runtimeMs;                                //!< Debug: algorithm runtime in ms (scaled x100). Reserved for future use.
    uint32_t runtimeProfile[10];                       //!< Debug: per-stage runtime breakdown (scaled x100). Reserved for future use.
    uint16_t peakBinIFFT;                              //!< Peak bin index from IFFT.
    uint16_t peakCountIFFT;                            //!< Number of peaks found by IFFT.
    uint16_t ifftValid;                                //!< IFFT validity flag.
} CSProcess_ExtendedResults_t;
#endif

/**
 * @brief Structure to hold ranging results.
 *
 * This structure contains the estimated distance, quality average,
 * and confidence of the estimation for ranging operations.
 */
typedef struct {
    uint32_t distance;    //!< Estimated distance in centimeters.
    uint32_t quality;     //!< Average quality of the ranging measurement.
    uint32_t confidence;  //!< Confidence level of the distance estimation.
    uint32_t velocity;    //!< Estimated velocity (meters/second) used in motion compensation.
#ifdef CS_PROCESS_EXT_RESULTS
    CSProcess_ExtendedResults_t* extendedResults;   //!< Extended Results
#endif
} CSProcess_Results_t;

/**
 * @brief Structure to hold procedure initialization parameters.
 *
 * This structure contains the parameters required to initialize a procedure.
 */
typedef struct {
    uint16_t handle;                    //!< CS Process session handle for this procedure
    uint8_t numAntPath;                 //!< Number of antenna paths (1-4), computed from ACI
    uint8_t localRole;                  //!< The role of the local device. @ref CS_ROLE_INITIATOR or @ref CS_ROLE_REFLECTOR
    // CS procedure configuration parameters.
    uint8_t tFCS;                       //!< T_FCS: frequency change/settle period (us)
    uint8_t tIP1;                       //!< T_IP1: interlude period between CS packets (us)
    uint8_t tIP2;                       //!< T_IP2: interlude period between CS tones (us)
    uint8_t tPM;                        //!< T_PM: phase measurement period (us)
    uint8_t tSW;                        //!< T_SW: antenna switch period (us), Derived from both devices
    uint8_t csSync;                     //!< CS_SYNC type (see BT Core Spec CS_SYNC_PHY)
    uint8_t rttType;                    //!< RTT type (see BT Core Spec RTT_Type)
    uint8_t mode0Steps;                 //!< Number of Mode 0 steps per subevent
    uint8_t mainModeType;               //!< Main_Mode_Type (1=Mode1, 2=Mode2, 3=Mode3)
    uint8_t mainModeRepetition;         //!< Main_Mode_Repetition count (0..3
    uint8_t toneAntennaConfigSelection; //!< Antenna Configuration Index (0..7)
} CSProcess_InitProcedureParams_t;

/*
 * @brief Parameters structure to be sent to @ref CSProcess_AddSubeventResults function
 */
typedef struct {

    ChannelSounding_resultsSourceMode_e resultsSourceMode;  //!< The source of the subevent results
    int8_t referencePowerLevel;                             //!< Reference Power Level, should be between @ref CS_MIN_TX_POWER_VALUE and @ref CS_MAX_TX_POWER_VALUE.
                                                            //!< Set this value to @ref CS_INVALID_TX_POWER if this parameter is not relevant
    int16_t frequencyCompensation;                          //!< Frequency compensation (CFO) in units of 0.01 ppm. unused for continuation events.
    uint8_t subeventDoneStatus;                             //!< Status of the Subevent Done.
    uint8_t procedureDoneStatus;                            //!< Status of the Procedure Done.
    uint8_t numAntennaPath;                                 //!< Number of antenna paths supported.
    uint8_t numStepsReported;                               //!< Number of steps reported in the given subevent.
    uint8_t totalSubeventStepsCount;                        //!< Total number of steps the will be reported in this subevent.
    uint8_t* data;                                          //!< Pointer to the subevent results steps.
    uint32_t* totalBytesProcessed;                          //!< Output parameter - will be set to the total length (bytes) of processed data. If NULL - will be discarded.
} CSProcess_AddSubeventResultsParams_t;

/*********************************************************************
 * FUNCTIONS
 */

/*******************************************************************************
 * @fn          CSProcess_Start
 *
 * @brief       Initializes the CS Process module DB and internal variables.
 *              Should be called once at device startup and before any other
 *              CS Process API function.
 *
 * @param       None
 *
 * @return      CS_PROCESS_SUCCESS - Operation completed successfully.
 */
csProcessStatus_e CSProcess_Start( void );

/*******************************************************************************
 * @fn          CSProcess_OpenSession
 *
 * @brief       Opens a new session for multiple distance measurements.
 *              If filtering is being used, it will be common to those
 *              different measurements.
 *
 * @param       None
 *
 * @return      Session handle to be used by the caller. Range: 0 to @ref (CS_PROCESS_MAX_SESSIONS - 1)
 * @return      @ref CS_PROCESS_INVALID_SESSION - No space left for a new session.
 */
uint16_t CSProcess_OpenSession( void );

/*******************************************************************************
 * @fn          CSProcess_CloseSession
 *
 * @brief       Closes a session. If the session has a procedure currently active,
 *              it will be terminated.
 *              If the session is not opened or an invalid handle is provided,
 *              nothing will be done.
 *
 * @param       handle - Session handle to close
 *
 * @return      CS_PROCESS_SUCCESS
 */
csProcessStatus_e CSProcess_CloseSession(uint16_t handle);

/*******************************************************************************
 * @fn          CSProcess_InitProcedure
 *
 * @brief       This function initiates the current session with a new procedure.
 *              If a procedure is already activated for this session but distance results have not
 *              been generated yet, the old data will be discarded and the new procedure will be
 *              initiated (only after verifying the function's parameters).
 *
 * @param       pParams - Pointer to function parameters of type @ref CSProcess_InitProcedureParams_t
 *
 * @return      CS_PROCESS_SUCCESS - Operation completed successfully.
 * @return      CS_PROCESS_INVALID_PARAM - If pParams is NULL, or if one of the inner-parameters is invalid.
 * @return      CS_PROCESS_SESSION_NOT_OPENED - If no session is opened for the given handle.
 * @return      CS_PROCESS_ANOTHER_SESSION_IN_PROCESS - If a procedure for another session is in process.
 *
 * @note        If a procedure is already activated (no matter the session) and this function
 *              is called with invalid parameters, the old procedure data will be maintained.
 */
csProcessStatus_e CSProcess_InitProcedure(CSProcess_InitProcedureParams_t *pParams);

/*******************************************************************************
 * @fn          CSProcess_TerminateProcedure
 *
 * @brief       This function clears the current procedure data.
 *              If there is no procedure currently active - nothing will be done.
 *
 * @param       None
 *
 * @return      None
 */
void CSProcess_TerminateProcedure( void );

/*******************************************************************************
 * @fn          CSProcess_AddSubeventResults
 *
 * @brief       This function handles the subevent results and adds them to the Ranging module.
 *              If the module is active and all data has been collected (Initiator and Reflector) -
 *              it estimates the distance.
 *              It is required that the local results would be processed before the remote results.
 *
 * @param       pParams - Pointer to the function parameters
 *
 * @return      CS_PROCESS_SUCCESS - Operation completed successfully.
 * @return      CS_PROCESS_DISTANCE_ESTIMATION_PENDING - Operation completed successfully and distance results can be estimated using @ref CSProcess_EstimateDistance.
 * @return      CS_PROCESS_INVALID_PARAM - If pParams is NULL, or if one of the inner-parameters is invalid.
 * @return      CS_PROCESS_PROCEDURE_NOT_ACTIVE - If there is no procedure currently being processed
 * @return      CS_PROCESS_TOO_MANY_STEPS_PROVIDED - Too many steps provided.
 * @return      CS_PROCESS_STEPS_PROCESSING_FAILED - Invalid parameters or step data length.
 * @return      CS_PROCESS_MODE_NOT_SUPPORTED - Results mode or step mode not supported.
 * @return      CS_PROCESS_RESULTS_MODE_NOT_SUPPORTED - Results mode not supported.
 * @return      CS_PROCESS_PROCEDURE_ABORTED - Procedure was aborted.
 * @return      CS_PROCESS_SUBEVENT_STATUS_INVALID - Invalid subevent status.
 * @return      CS_PROCESS_LOCAL_RESULTS_NOT_READY - If local results is not yet ready for processing remote results
 */
csProcessStatus_e CSProcess_AddSubeventResults(CSProcess_AddSubeventResultsParams_t *pParams);

/*******************************************************************************
 * @fn          CSProcess_EstimateDistance
 *
 * @brief       Estimate the distance based on the collected subevent results.
 * @note        Should be called after @ref CSProcess_AddSubeventResults returned
 *              @ref CS_PROCESS_DISTANCE_ESTIMATION_PENDING.
 *
 * @param[out]  distanceResults - Pointer to distance results of type @ref CSProcess_Results_t
 *
 * @return      @ref CS_PROCESS_SUCCESS - when distance results have been successfully calculated
 *              @ref CS_PROCESS_DISTANCE_ESTIMATION_FAILED - if the module couldn't calculate the distance
 *              @ref CS_PROCESS_PROCEDURE_PROCESSING_PENDING - if the procedure is still being processed
 *              @ref CS_PROCESS_PROCEDURE_NOT_ACTIVE - if there is no procedure currently being processed
 */
csProcessStatus_e CSProcess_EstimateDistance(CSProcess_Results_t* pDistanceResults);

#ifdef __cplusplus
}
#endif

#endif /* APP_CS_PROCESS_API_H */
