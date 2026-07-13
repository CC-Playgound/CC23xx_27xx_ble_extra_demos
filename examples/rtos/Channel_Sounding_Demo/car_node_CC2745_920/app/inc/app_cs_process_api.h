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
#include <ti/ble/app_util/cs_ranging/include/BleCsRanging.h>
#include "app_cs_api.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

#define CS_PROCESS_MAX_SESSIONS     MAX_NUM_BLE_CONNS   // Maximum number of sessions
#define CS_PROCESS_INVALID_SESSION  0xFFFF              // Invalid Session ID

/* Size of the array which maps collected local steps reports to channels */
#define CS_PROCESS_CHANNELS_MAP_ARRAY_SIZE      CS_RANGING_PCT_ARRAY_SIZE

/* Maximum number of main-mode to process for a single procedure, as the number of available channels */
#define CS_PROCESS_MAX_STEPS                    CS_PROCESS_CHANNELS_MAP_ARRAY_SIZE - 3

/* Number of main-mode steps allowed for a single procedure */
#define CS_PROCESS_ALLOWED_NUMBER_OF_STEPS      CS_PROCESS_MAX_STEPS

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
    CS_PROCESS_MODE_ZERO_CHECK_FAILED,          //!< Mode zero check failed - None of the mode-0 steps in the subevent contained Packet Quality of '0'
    CS_PROCESS_TOO_MANY_STEPS_PROVIDED,         //!< Too many steps provided - The number of steps in the subevent exceeds the maximum allowed steps
    CS_PROCESS_INCORRECT_NUMBER_OF_STEPS,       //!< Incorrect number of steps - The total number of main mode steps provided does not match the expected number of steps @ref CS_PROCESS_ALLOWED_NUMBER_OF_STEPS
    CS_PROCESS_STEPS_PROCESSING_FAILED,         //!< Failure during the steps processing
    CS_PROCESS_SUBEVENT_STATUS_INVALID,         //!< Invalid subevent status for steps
    CS_PROCESS_TOO_MANY_SUBEVENTS_PROVIDED,     //!< Too many subevents provided - The number of subevents in the procedure exceeds the maximum allowed subevents
    CS_PROCESS_SUBEVENT_ABORTED,                //!< Subevent was aborted
    CS_PROCESS_PROCEDURE_ABORTED,               //!< Procedure was aborted
    CS_PROCESS_PROCEDURE_PROCESSING_PENDING,    //!< Procedure processing is pending, waiting for more subevent results
    CS_PROCESS_DISTANCE_ESTIMATION_FAILED,      //!< Results processed successfully, distance estimation failed
    CS_PROCESS_DISTANCE_ESTIMATION_PENDING,     //!< All subevent results has been processed and distance may now be estimated
} csProcessStatus_e;

#ifdef CS_PROCESS_EXT_RESULTS
/**
 * @brief Structure to hold extended ranging results.
 *
 * This structure contains the estimated distance, quality average, and confidence
 * of the estimation for ranging operations, along with extended data used
 * for the distance estimation.
 */
typedef struct {
    // BleCsRanging library debug info
    uint16_t numMpc;                              //!< Number of multipath
    uint32_t distanceMusic[CS_MAX_ANT_PATHS];     //!< Estimated distance by MUSIC per path
    uint32_t distanceNN[CS_MAX_ANT_PATHS];        //!< Estimated distance by NN per path
    uint16_t numMpcPaths[CS_MAX_ANT_PATHS];       //!< Number of multipath per path
    uint32_t qualityPaths[CS_MAX_ANT_PATHS];      //!< Quality of measurement per path
    uint32_t confidencePaths[CS_MAX_ANT_PATHS];   //!< Confidence of estimation per path

    // Local / Remote RPL (Reference Power Level) for each subevent
    int8_t localRpl[CS_MAX_SUBEVENTS_PER_PROCEDURE];
    int8_t remoteRpl[CS_MAX_SUBEVENTS_PER_PROCEDURE];

    // Raw results data
    CS_modeZeroInitStep_t modeZeroStepsInit[CS_MAX_MODE_ZERO_PER_PROCEDURE]; //!< Initiator mode-0 steps. Each group ordered by subevent indices
    CS_modeZeroReflStep_t modeZeroStepsRef[CS_MAX_MODE_ZERO_PER_PROCEDURE];  //!< Reflector mode-0 steps. Each group ordered by subevent indices

    // Contains Local device steps antenna permutations, 75 steps.
    // Ordered by channel numbers, starts from channel 2.
    uint8_t permutationIndexLocal[CS_RANGING_PCT_ARRAY_SIZE];

    // Contains Local device steps data: (I, Q, Quality), max 75 steps per max 4 antenna paths.
    // Ordered by channel numbers, starts from channel 2.
    uint32_t stepsDataLocal[CS_RANGING_PCT_ARRAY_SIZE_PATHS];

    // Remote antenna permutation index, similar to local.
    // Note: Depends on the protocol used for receiving the remote data,
    //       if using BTCS, this array is not relevant
    uint8_t permutationIndexRemote[CS_RANGING_PCT_ARRAY_SIZE];

    // Remote steps data, similar to local.
    uint32_t stepsDataRemote[CS_RANGING_PCT_ARRAY_SIZE_PATHS];
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
    uint16_t handle;             //!< CS Process session handle for this procedure
    csACI_e aci;                 //!< ACI (Antenna Configuration Index) for this procedure
    uint8_t localRole;           //!< The role of the local device. @ref CS_ROLE_INITIATOR or @ref CS_ROLE_REFLECTOR

    // Tx Power (dBm) used by the local device for CS procedures.
    // Should be between @ref CS_MIN_TX_POWER_VALUE and @ref CS_MAX_TX_POWER_VALUE.
    // Set this value to @ref CS_INVALID_TX_POWER if pathLoss is not required
    int8_t selectedTxPower;

    // Antenna switching period
    uint8_t localTsw;
    uint8_t remoteTsw;

    // CS Configuration parameters
    uint8_t tIP1;     //!< Index of the period used between RTT packets
    uint8_t tIP2;     //!< Index of the interlude period used between CS tones
    uint8_t tFCs;     //!< Index used for frequency changes
    uint8_t tPM;      //!< Index for the measurement period of CS tones
} CSProcess_InitProcedureParams_t;

/*
 * @brief Parameters structure to be sent to @ref CSProcess_AddSubeventResults function
 */
typedef struct {

    ChannelSounding_resultsSourceMode_e resultsSourceMode;  //!< The source of the subevent results
    int8_t referencePowerLevel;                             //!< Reference Power Level, should be between @ref CS_MIN_TX_POWER_VALUE and @ref CS_MAX_TX_POWER_VALUE.
                                                            //!< Set this value to @ref CS_INVALID_TX_POWER if this parameter is not relevant
    uint8_t subeventDoneStatus;                             //!< Status of the Subevent Done.
    uint8_t procedureDoneStatus;                            //!< Status of the Procedure Done.
    uint8_t numAntennaPath;                                 //!< Number of antenna paths supported.
    uint8_t numStepsReported;                               //!< Number of steps reported in the given subevent.
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
 * @return      CS_PROCESS_INCORRECT_NUMBER_OF_STEPS - procedureDoneStatus is 0 but the total main mode steps reported is not @ref CS_PROCESS_ALLOWED_NUMBER_OF_STEPS.
 * @return      CS_PROCESS_MODE_ZERO_CHECK_FAILED - Mode-0 steps check failed.
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
