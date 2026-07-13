/******************************************************************************

@file  app_cs_process.c

@brief This file implements the CS Ranging process logic.

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
#ifdef CHANNEL_SOUNDING
#include "ti/ble/stack_util/icall/app/icall.h"
#include "ti/ble/host/cs/cs.h"
#include "ti/ble/controller/ll/ll_rat.h"
#include "app_ranging_client_api.h"
#include "app_cs_process_api.h"
#include "app_cs_api.h"
#include "app_btcs_api.h"
#include <ti/ble/app_util/cs_ranging/include/BleCsRanging.h>
#include <ti/ble/app_util/cs_ranging/include/BleCsRangingFilters.h>
#include <ti/drivers/utils/Math.h>

/*******************************************************************************
 * CONSTANTS
 */

/* CS Number of antenna permutation indices */
#define CS_PROCESS_NUM_PERMUTATIONS 24

/* Antenna Paths: A1 - A4 */
#define A1 0
#define A2 1
#define A3 2
#define A4 3

/* Macro to build a permutation out of different antenna paths */
#define CS_PERM(a0, a1, a2, a3) ((a0 & 0x3) | ((a1 & 0x3) << 2) | ((a2 & 0x3) << 4) | ((a3 & 0x3) << 6))

#define CS_PROCESS_MIN_ANT_PATHS    1
#define CS_PROCESS_MAX_ANT_PATHS    4

/* This table represents all possible antenna permutations.
 * Matches the table from the BLE spec - Version 6.0 | Vol 6, Part H - 4.7.5 */
const uint8_t antennaPermutations[CS_PROCESS_NUM_PERMUTATIONS] =
{
    CS_PERM(A1,A2,A3,A4), CS_PERM(A2,A1,A3,A4), CS_PERM(A1,A3,A2,A4), CS_PERM(A3,A1,A2,A4), CS_PERM(A3,A2,A1,A4), CS_PERM(A2,A3,A1,A4),
    CS_PERM(A1,A2,A4,A3), CS_PERM(A2,A1,A4,A3), CS_PERM(A1,A4,A2,A3), CS_PERM(A4,A1,A2,A3), CS_PERM(A4,A2,A1,A3), CS_PERM(A2,A4,A1,A3),
    CS_PERM(A1,A4,A3,A2), CS_PERM(A4,A1,A3,A2), CS_PERM(A1,A3,A4,A2), CS_PERM(A3,A1,A4,A2), CS_PERM(A3,A4,A1,A2), CS_PERM(A4,A3,A1,A2),
    CS_PERM(A4,A2,A3,A1), CS_PERM(A2,A4,A3,A1), CS_PERM(A4,A3,A2,A1), CS_PERM(A3,A4,A2,A1), CS_PERM(A3,A2,A4,A1), CS_PERM(A2,A3,A4,A1)
};

#define CS_PROCESS_CEIL_DIVIDE_2(a) (((a) + ((a) % 2)) / 2)

/*********************************************************************
 * TYPEDEFS
 */

// Module state flags. Flags are determined by bits.
typedef enum CSProcess_stateFlags_e
{
    CS_PROCESS_STATE_IDLE                       =   0x00, //!< Waiting for data to arrive
    CS_PROCESS_STATE_PROCEDURE_ACTIVE           =   0x01, //!< A procedure has started and data may be collected
    CS_PROCESS_STATE_INITIATOR_DATA_READY       =   0x02, //!< Finished collecting initiator data
    CS_PROCESS_STATE_REFLECTOR_DATA_READY       =   0x04, //!< Finished collecting reflector data
    CS_PROCESS_STATE_INITIATOR_MODE_ZERO_VALID  =   0x08, //!< Collected initiator mode-0 results are valid
    CS_PROCESS_STATE_REFLECTOR_MODE_ZERO_VALID  =   0x10, //!< Collected reflector mode-0 results are valid
} CSProcess_stateFlags_e;

// Filtering DB structure. Holds filtering data across multiple runs of the CSProcess module.
typedef struct
{
    uint8_t initDone;                                   //!< Indicates if the filtering has already been initialized
    uint32_t lastTimeTicks;                             //!< Holds the time in RAT ticks of the last system time pull
    BleCsRanging_SlewRateLimiterFilter_t  srlfFilter;   //!< SRLF filter
} CSProcessFiltersDb_t;

typedef struct
{
    uint8_t isInitiated;              //!< Indicates if this session was initiated (not necessarily active) by @ref CSProcess_OpenSession
    CSProcessFiltersDb_t filteringDb; //!< Holds the filtering data for this session
} CSProcessSessionData;

// The module DB structure. Holds collected results and configurations
typedef struct
{
    /* Internal indicators */
    uint8_t localRole;                                                          //!< The role of the local device
    uint8_t state;                                                              //!< Current state flags of the module
    uint8_t subeventCounterLocal;                                               //!< Subevent counter of the local device
    uint8_t subeventCounterRemote;                                              //!< Subevent counter of the remote device

    /* RAS related */
    uint8_t currentStepLocal;                                                   //!< Global variable to store the next local step index to be processed
    uint8_t currentStepRemote;                                                  //!< Global variable to store the next remote step index to be processed
    uint8_t stepsIdxToChnlMap[CS_PROCESS_MAX_STEPS];                            //!< Mapping collected local steps indices to channel numbers

    /* Sessions data */
    uint16_t currSession;                                                       //!< The current procedure session handle as passed by @ref CSProcess_InitProcedure
    CSProcessSessionData sessionsDb[CS_PROCESS_MAX_SESSIONS];                   //!< Sessions Database

    /* BleCsRanging module related */
    BleCsRanging_Tone_t initPathsData[CS_RANGING_PCT_ARRAY_SIZE_PATHS];         //!< Array of 75 elements for each antenna path, Initiator role.
    BleCsRanging_Tone_t refPathsData[CS_RANGING_PCT_ARRAY_SIZE_PATHS];          //!< Array of 75 elements for each antenna path, Reflector role.
    int8_t peerMaxRssi;                                                         //!< Maximum RSSI (dBm) value received from the peer mode-0 steps.
    int8_t selectedTxPower;                                                     //!< Tx Power (dBm) used by the local device for the CS procedure.
    int8_t localRpl[CS_MAX_SUBEVENTS_PER_PROCEDURE];                            //!< Local RPL (Reference Power Level) for each subevent of the local device
    int8_t remoteRpl[CS_MAX_SUBEVENTS_PER_PROCEDURE];                           //!< Remote RPL (Reference Power Level) for each subevent of the remote device
    BleCsRanging_Config_t config;                                               //!< Configuration to be sent to the BleCsRanging API.

    /* BTCS related */
    uint8_t stepsIdxToAntPermutationMap[CS_PROCESS_MAX_STEPS];                  //!< Mapping collected steps indices to antenna permutation (not indices)
#ifdef CS_PROCESS_EXT_RESULTS
    /* extended information */
    uint8_t currentModeZeroStepInit;                                            //!< Global iterator for initiator mode-0 steps
    uint8_t currentModeZeroStepRef;                                             //!< Global iterator for reflector mode-0 steps
    CS_modeZeroInitStep_t modeZeroStepsInit[CS_MAX_MODE_ZERO_PER_PROCEDURE];    //!< Initiator mode-0 steps. Ordered by step index
    CS_modeZeroReflStep_t modeZeroStepsRef[CS_MAX_MODE_ZERO_PER_PROCEDURE];     //!< Reflector mode-0 steps. Ordered by step index
    uint8_t permutationIndexLocal[CS_RANGING_PCT_ARRAY_SIZE];                   //!< Local antenna permutation indices. Ordered by channels
    uint8_t permutationIndexRemote[CS_RANGING_PCT_ARRAY_SIZE];                  //!< Remote antenna permutation indices. Ordered by channels
#endif

} CSProcessDb_t;

//*****************************************************************************
//! Prototypes
//*****************************************************************************

/********** Helper functions **********/
uint8_t csProcess_GetStepLengthBTCS(uint8_t mode, uint8_t role, uint8_t numAntennaPath);
csProcessStatus_e csProcess_CheckSessionHandle(uint16_t handle);

/********** Internal DB managing functions **********/
void csProcess_ClearProcedureData(void);
csProcessStatus_e csProcess_StepsPostProcess(uint8_t role, uint8_t subeventDoneStatus, uint8_t procedureDoneStatus);

/********** Subevents processing functions **********/
csProcessStatus_e csProcess_HandleSubeventResults(CSProcess_AddSubeventResultsParams_t *pParams);
csProcessStatus_e csProcess_ProcessSubeventResultsSteps(csSubeventResultsStep_t *subeventResultsSteps, uint8_t numAntennaPath,
                                                        uint8_t numStepsReported, uint8_t role, uint32_t *totalBytesProcessed);
#ifdef RANGING_CLIENT
csProcessStatus_e csProcess_ProcessSubeventResultsStepsRAS(Ranging_subEventResultsStep_t *subeventResultsSteps, uint8_t numAntennaPath,
                                                           uint8_t numStepsReported, uint8_t role, uint32_t *totalBytesProcessed);
#endif

csProcessStatus_e csProcess_ProcessSubeventResultsStepsBTCS(uint8_t *subeventResultsSteps, uint8_t numAntennaPath,
                                                            uint8_t numStepsReported, uint8_t role, uint32_t *totalBytesProcessed);
uint8_t csProcess_GetSubeventCounter(ChannelSounding_resultsSourceMode_e mode);
bool csProcess_IsAllowedToIncrementSubeventCounter(ChannelSounding_resultsSourceMode_e mode);
void csProcess_IncrementSubeventCounter(ChannelSounding_resultsSourceMode_e mode);

/********** Rpl functions **********/
int8_t csProcess_GetSubeventRpl(ChannelSounding_resultsSourceMode_e mode, uint8_t subeventIndex);
void csProcess_SetSubeventRpl(ChannelSounding_resultsSourceMode_e mode, uint8_t subeventIndex, int8_t rpl);

/********** Steps processing functions **********/
csProcessStatus_e csProcess_ProcessStep(uint8_t mode, uint8_t *stepData, uint8_t role, uint8_t stepChannel, uint8_t numAntennaPath);
csProcessStatus_e csProcess_ProcessStepBTCS(uint8_t mode, uint8_t *stepData, uint8_t role, uint8_t stepChannel, uint8_t numAntennaPath);
csProcessStatus_e csProcess_ProcessModeTwoStepBTCS(BTCS_modeTwoStep_t* stepDataToAdd, uint8_t role, uint8_t stepChannel, uint8_t numAntennaPath);
void csProcess_ProcessModeZeroStep(uint8_t *stepData, uint8_t role);
csProcessStatus_e csProcess_ProcessModeTwoStep(CS_modeTwoStep_t* stepDataToAdd, uint8_t role, uint8_t stepChannel, uint8_t numAntennaPath);
void csProcess_ModeTwoStepPostProcess(uint8_t role, uint8_t stepChannel);

/********** Distance results generating functions **********/
csProcessStatus_e csProcess_CheckIfDone();
int8_t csProcess_CalculateRPL(int8_t *rpl, uint8_t numOfRpl);
csProcessStatus_e csProcess_CalcResults(CSProcess_Results_t *csResults);


//*****************************************************************************
//! Globals
//*****************************************************************************

/*
 * Holds a single session DB
 */
CSProcessDb_t gCsProcessDb;

//*****************************************************************************
//! Public Functions
//*****************************************************************************

/*******************************************************************************
 * Public function defined in app_cs_process.h.
 */
csProcessStatus_e CSProcess_Start( void )
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;

    // Clear the module DB
    memset(&gCsProcessDb, 0, sizeof(CSProcessDb_t));

    // Clear procedure data
    csProcess_ClearProcedureData();

    // Initialize BleCsRanging module configuration.
    // Note that numAntPath parameter is still unknown at this point
    // TODO: Make these values configurable from outside of the module
    BleCsRanging_initConfig(&gCsProcessDb.config);

    // Use adaptive algorithm
    gCsProcessDb.config.algorithm = BleCsRanging_Algorithm_Adaptive;

    // Set multiple antenna path preprocessing method
    gCsProcessDb.config.sumAntPath = BleCsRanging_MAP_Averaging;

    // Reset numAntPath to 1 until procedure is initialized
    gCsProcessDb.config.numAntPath = CS_PROCESS_MIN_ANT_PATHS;

    // Set expected number of channels as the PCT array size
    gCsProcessDb.config.numChannels = CS_RANGING_PCT_ARRAY_SIZE;

    // Set IIR filter coefficient
    gCsProcessDb.config.iirCoeff = 0.5;

    return status;
}

/*******************************************************************************
 * Public function defined in app_cs_process.h.
 */
uint16_t CSProcess_OpenSession( void )
{
    uint16_t handle = CS_PROCESS_INVALID_SESSION;

    // Find an available slot
    for (uint16_t i = 0; i < CS_PROCESS_MAX_SESSIONS; i++)
    {
        if (gCsProcessDb.sessionsDb[i].isInitiated == FALSE)
        {
            handle = i;
            break;
        }
    }

    if (handle != CS_PROCESS_INVALID_SESSION)
    {
        // Mark the session as active
        gCsProcessDb.sessionsDb[handle].isInitiated = TRUE;

        // Clear filtering DB
        memset(&(gCsProcessDb.sessionsDb[handle].filteringDb), 0, sizeof(CSProcessFiltersDb_t));

        // Mark filtering as not initialized
        gCsProcessDb.sessionsDb[handle].filteringDb.initDone = FALSE;
    }

    return handle;
}

/*******************************************************************************
 * Public function defined in app_cs_process.h.
 */
csProcessStatus_e CSProcess_CloseSession(uint16_t handle)
{
    if (csProcess_CheckSessionHandle(handle) == CS_PROCESS_SUCCESS)
    {
        // Release filtering data if it has been initialized
        if(gCsProcessDb.sessionsDb[handle].filteringDb.initDone == TRUE)
        {
            gCsProcessDb.sessionsDb[handle].filteringDb.initDone = FALSE;
        }

        // Mark the session as inactive
        gCsProcessDb.sessionsDb[handle].isInitiated = FALSE;

        // If closing a session that currently has a procedure active
        if (gCsProcessDb.currSession == handle)
        {
            // Clear procedure data
            csProcess_ClearProcedureData();
        }
    }

    return CS_PROCESS_SUCCESS;
}

/*******************************************************************************
 * Public function defined in app_cs_process.h.
 */
csProcessStatus_e CSProcess_InitProcedure(CSProcess_InitProcedureParams_t *pParams)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;
    uint8_t finalTsw = CS_T_0US;

    // Check parameters
    if ((NULL == pParams) ||
        (pParams->aci > ACI_A2_B2) ||
        (pParams->localRole != CS_ROLE_INITIATOR && pParams->localRole != CS_ROLE_REFLECTOR) ||
        (((pParams->selectedTxPower != CS_INVALID_TX_POWER) &&
         (pParams->selectedTxPower < CS_MIN_TX_POWER_VALUE)) || (pParams->selectedTxPower > CS_MAX_TX_POWER_VALUE)))
    {
        status = CS_PROCESS_INVALID_PARAM;
    }

    // Calculate final Tsw if previous checks passed
    if (status == CS_PROCESS_SUCCESS)
    {
        finalTsw = CS_GetTswByACI(pParams->aci, pParams->localTsw, pParams->remoteTsw);

        // If aci requires more than 1 antenna path, validate that tsw is above 0
        if (pParams->aci > ACI_A1_B1 && finalTsw == CS_T_0US)
        {
            status = CS_PROCESS_INVALID_PARAM;
        }
    }

    // Check that the given session handle is valid and initiated
    if (status == CS_PROCESS_SUCCESS)
    {
        status = csProcess_CheckSessionHandle(pParams->handle);
    }

    // Check that we are not in the middle of a processing of another session
    if (status == CS_PROCESS_SUCCESS &&
        gCsProcessDb.currSession != CS_PROCESS_INVALID_SESSION &&
        gCsProcessDb.currSession != pParams->handle)
    {
        status = CS_PROCESS_ANOTHER_SESSION_IN_PROCESS;
    }

    // Initiate DB parameters
    if (status == CS_PROCESS_SUCCESS)
    {
        // Clear the procedure data, even if another procedure is in process
        csProcess_ClearProcedureData();

        // Set the configuration parameters for the BleCsRanging configuration
        gCsProcessDb.config.numAntPath = CS_calcNumPaths(pParams->aci);

        // Set timing parameters for BleCsRanging config
        gCsProcessDb.config.timingParams.tIP1 = pParams->tIP1;
        gCsProcessDb.config.timingParams.tIP2 = pParams->tIP2;
        gCsProcessDb.config.timingParams.tPM  = pParams->tPM;
        gCsProcessDb.config.timingParams.tFCs = pParams->tFCs;
        gCsProcessDb.config.timingParams.tSw  = finalTsw;


        // Save the local role and tx power for the procedure
        gCsProcessDb.localRole = pParams->localRole;
        gCsProcessDb.selectedTxPower = pParams->selectedTxPower;

        // Mark that a procedure as active
        gCsProcessDb.state |= CS_PROCESS_STATE_PROCEDURE_ACTIVE;

        // Set the current session handle
        gCsProcessDb.currSession = pParams->handle;
    }

    return status;
}

/*******************************************************************************
 * Public function defined in app_cs_process.h.
 */
void CSProcess_TerminateProcedure( void )
{
    if (gCsProcessDb.currSession != CS_PROCESS_INVALID_SESSION)
    {
        csProcess_ClearProcedureData();
    }
}

/*******************************************************************************
 * Public function defined in app_cs_process.h.
 */
csProcessStatus_e CSProcess_AddSubeventResults(CSProcess_AddSubeventResultsParams_t *pParams)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;

    // Ensure that there is a procedure active and validate function parameters
    if (gCsProcessDb.currSession == CS_PROCESS_INVALID_SESSION ||
        (gCsProcessDb.state & CS_PROCESS_STATE_PROCEDURE_ACTIVE) != CS_PROCESS_STATE_PROCEDURE_ACTIVE)
    {
        status = CS_PROCESS_PROCEDURE_NOT_ACTIVE;
    }
    else if (NULL == pParams || NULL == pParams->data ||
             pParams->resultsSourceMode  >= CS_RESULTS_MODE_END)
    {
        status = CS_PROCESS_INVALID_PARAM;
    }

    // Send parameters to the internal handler
    if (status == CS_PROCESS_SUCCESS)
    {
        status = csProcess_HandleSubeventResults(pParams);
    }

    return status;
}

/*******************************************************************************
 * Public function defined in app_cs_process.h.
 */
csProcessStatus_e CSProcess_EstimateDistance(CSProcess_Results_t* pDistanceResults)
{
    csProcessStatus_e status;

    if (gCsProcessDb.currSession == CS_PROCESS_INVALID_SESSION)
    {
        status = CS_PROCESS_PROCEDURE_NOT_ACTIVE;
    }
    else if (csProcess_CheckIfDone() == CS_PROCESS_DISTANCE_ESTIMATION_PENDING)
    {
        status = csProcess_CalcResults(pDistanceResults);

        csProcess_ClearProcedureData();
    }
    else
    {
        status = CS_PROCESS_PROCEDURE_PROCESSING_PENDING;
    }

    return status;
}

//*****************************************************************************
//! Internal Functions
//*****************************************************************************\

/*******************************************************************************
 * @fn          csProcess_GetStepLengthBTCS
 *
 * @brief       This function calculates the length of a step data in BTCS format,
 *              depends on role, mode, and number of antenna paths.
 *
 * @param       mode - Step mode. Should be one of:
 *                     @ref CS_MODE_0
 *                     @ref CS_MODE_1
 *                     @ref CS_MODE_2
 *                     @ref CS_MODE_3
 *
 * @param       role - Role of the device measured the step of types:
 *                     @ref CS_ROLE_INITIATOR or @ref CS_ROLE_REFLECTOR
 *
 * @param       numAntennaPath - Number of antenna paths used in the step
 *
 * @return      Length of the relevant step data
 * @return      0 for one of the following cases:
 *              - Invalid mode has been given.
 *              - mode is 0 and role is invalid.
 */
uint8_t csProcess_GetStepLengthBTCS(uint8_t mode, uint8_t role, uint8_t numAntennaPath)
{
    uint8_t stepDataLen = 0;

    switch(mode)
    {
      case CS_MODE_0:
      {
        if (role == CS_ROLE_INITIATOR)
        {
            stepDataLen = sizeof(BTCS_modeZeroInitStep_t);
        }
        else if(role == CS_ROLE_REFLECTOR)
        {
            stepDataLen = sizeof(BTCS_modeZeroReflStep_t);
        }
        else
        {
            // Consider failure - leave stepDataLen as 0
        }
        break;
      }
      case CS_MODE_1:
      {
        stepDataLen = sizeof(BTCS_modeOneStep_t);
        break;
      }
      case CS_MODE_2:
      {
        stepDataLen = sizeof(BTCS_modeTwoStep_t) + numAntennaPath * 3; // 3 is the size of @ref BTCS_modeTwoStepData_t
        break;
      }
      case CS_MODE_3:
      {
        stepDataLen = sizeof(BTCS_modeThreeStep_t) + numAntennaPath * 3; // 3 is the size of @ref BTCS_modeTwoStepData_t
        break;
      }
      default:
      {
        break;
      }
    }

    return stepDataLen;
}

/*******************************************************************************
 * @fn          csProcess_CheckSessionHandle
 *
 * @brief Checks if a session handle is valid and has been initiated.
 *
 * This function verifies whether the provided session handle corresponds to a valid and active session.
 *
 * @param session_handle The handle of the session to check.
 *
 * @return  CS_PROCESS_SUCCESS The session handle is valid and initiated.
 * @return  CS_PROCESS_SESSION_NOT_OPENED The session handle is invalid or no initiated.
 */
// Check that a session handle is valid and initiated
csProcessStatus_e csProcess_CheckSessionHandle(uint16_t handle)
{
    if ((handle >= CS_PROCESS_MAX_SESSIONS) ||
        (gCsProcessDb.sessionsDb[handle].isInitiated == FALSE))
    {
        return CS_PROCESS_SESSION_NOT_OPENED;
    }

    return CS_PROCESS_SUCCESS;
}

/*******************************************************************************
 * @fn          csProcess_ClearProcedureData
 *
 * @brief       This function clears all of the data collected by the initiator
 *              and the reflector and sets the state to 'Idle'.
 *
 * @param       None
 *
 * @return      None
 */
void csProcess_ClearProcedureData(void)
{
    // Set the state to 'Idle'
    gCsProcessDb.state = (uint8_t) CS_PROCESS_STATE_IDLE;

    // Set subevent counters to zero
    gCsProcessDb.subeventCounterLocal = 0;
    gCsProcessDb.subeventCounterRemote = 0;

    // Set local / remote rpl values to be invalid
    memset(gCsProcessDb.localRpl, CS_INVALID_TX_POWER, sizeof(gCsProcessDb.localRpl));
    memset(gCsProcessDb.remoteRpl, CS_INVALID_TX_POWER, sizeof(gCsProcessDb.remoteRpl));

    // Reset RAS related data
    gCsProcessDb.currentStepLocal = 0;
    gCsProcessDb.currentStepRemote = 0;
    memset(gCsProcessDb.stepsIdxToChnlMap, 0, sizeof(gCsProcessDb.stepsIdxToChnlMap));

    // Reset current procedure session
    gCsProcessDb.currSession = CS_PROCESS_INVALID_SESSION;

    // Initialize the tones results values to be zero
    memset(gCsProcessDb.initPathsData, 0, sizeof(gCsProcessDb.initPathsData));
    memset(gCsProcessDb.refPathsData, 0, sizeof(gCsProcessDb.refPathsData));

    // Initialize the max RSSI and Tx Power to invalid values
    gCsProcessDb.peerMaxRssi = CS_INVALID_TX_POWER;
    gCsProcessDb.selectedTxPower = CS_INVALID_TX_POWER;

    // Reset CCC related data
    memset(gCsProcessDb.stepsIdxToAntPermutationMap, 0, sizeof(gCsProcessDb.stepsIdxToAntPermutationMap));

#ifdef  CS_PROCESS_EXT_RESULTS
    gCsProcessDb.currentModeZeroStepInit = 0;
    gCsProcessDb.currentModeZeroStepRef = 0;
    memset(gCsProcessDb.modeZeroStepsInit, 0xFF, sizeof(gCsProcessDb.modeZeroStepsInit));
    memset(gCsProcessDb.modeZeroStepsRef, 0xFF, sizeof(gCsProcessDb.modeZeroStepsRef));
    memset(gCsProcessDb.permutationIndexLocal, 0xFF, sizeof(gCsProcessDb.permutationIndexLocal));
    memset(gCsProcessDb.permutationIndexRemote, 0xFF, sizeof(gCsProcessDb.permutationIndexRemote));
#endif
}

/*******************************************************************************
 * @fn          csProcess_HandleSubeventResults
 *
 * @brief       This function handles the subevent results and adds them to the Ranging module.
 *              If the module is active and all data has been collected (Initiator and Reflector) -
 *              it estimates the distance.
 *
 * @warning     For internal use only, does not check parameters.
 *
 * @param       pParams - Pointer to the function parameters
 *
 * @return      CS_PROCESS_SUCCESS - Operation completed successfully.
 * @return      CS_PROCESS_TOO_MANY_STEPS_PROVIDED - Too many steps provided.
 * @return      CS_PROCESS_STEPS_PROCESSING_FAILED - Invalid parameters or step data length.
 * @return      CS_PROCESS_MODE_NOT_SUPPORTED - Results mode or step mode not supported.
 * @return      CS_PROCESS_RESULTS_MODE_NOT_SUPPORTED - Results mode not supported.
 * @return      CS_PROCESS_SUBEVENT_ABORTED - Subevent was aborted.
 * @return      CS_PROCESS_PROCEDURE_ABORTED - Procedure was aborted.
 * @return      CS_PROCESS_SUBEVENT_STATUS_INVALID - Invalid subevent status
 * @return      CS_PROCESS_INCORRECT_NUMBER_OF_STEPS - Total main mode steps reported is not @ref CS_PROCESS_ALLOWED_NUMBER_OF_STEPS.
 * @return      CS_PROCESS_MODE_ZERO_CHECK_FAILED - Mode-0 steps check failed.
 * @return      CS_PROCESS_TOO_MANY_SUBEVENTS_PROVIDED - Too many subevents provided.
 */
csProcessStatus_e csProcess_HandleSubeventResults(CSProcess_AddSubeventResultsParams_t *pParams)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;
    uint8_t role;           // The role of the device that reported about the subevent
    uint8_t subeventCounter = csProcess_GetSubeventCounter(pParams->resultsSourceMode); // Subevent counter for the current results

    if (subeventCounter >= CS_MAX_SUBEVENTS_PER_PROCEDURE)
    {
        // Too many subevents provided
        status = CS_PROCESS_TOO_MANY_SUBEVENTS_PROVIDED;
    }

    // Validate the subevent counter and statuses
    if (status == CS_PROCESS_SUCCESS)
    {
        if (pParams->subeventDoneStatus == CS_SUBEVENT_ABORTED)
        {
            status = CS_PROCESS_SUBEVENT_ABORTED;
        }
        else if(pParams->procedureDoneStatus == CS_PROCEDURE_ABORTED)
        {
            status = CS_PROCESS_PROCEDURE_ABORTED;
        }
        else if (pParams->procedureDoneStatus == CS_PROCEDURE_DONE && pParams->subeventDoneStatus == CS_SUBEVENT_ACTIVE)
        {
            // If the procedure is done, but the subevent is still active, we shouldn't process it
            status = CS_PROCESS_SUBEVENT_STATUS_INVALID;
        }
    }

    // Increment the subevent counter if needed and allowed to
    if (((status == CS_PROCESS_SUCCESS) || (status == CS_PROCESS_SUBEVENT_ABORTED)) &&
        (pParams->subeventDoneStatus != CS_SUBEVENT_ACTIVE) &&
        (csProcess_IsAllowedToIncrementSubeventCounter(pParams->resultsSourceMode)))
    {
        // Save current subevent counter and increment it
        csProcess_IncrementSubeventCounter(pParams->resultsSourceMode);
    }

    // Add RPL parameter
    if ((status == CS_PROCESS_SUCCESS) &&
        (csProcess_GetSubeventRpl(pParams->resultsSourceMode, subeventCounter) == CS_INVALID_TX_POWER))
    {
        csProcess_SetSubeventRpl(pParams->resultsSourceMode, subeventCounter, pParams->referencePowerLevel);
    }

    // If all checks passed, proceed with processing the subevent steps
    if (status == CS_PROCESS_SUCCESS)
    {
        // Add results to the Ranging module
        switch (pParams->resultsSourceMode)
        {
        case CS_RESULTS_MODE_LOCAL:
        {
            // The role is the same as in the DB
            role = gCsProcessDb.localRole;

            // Process local data
            status = csProcess_ProcessSubeventResultsSteps((csSubeventResultsStep_t*)pParams->data, pParams->numAntennaPath,
                                                            pParams->numStepsReported, role, pParams->totalBytesProcessed);
            break;
        }
        case CS_RESULTS_MODE_PROP:
        {
            // The remote device role is the opposite of the local role
            role = CS_GET_OPPOSITE_ROLE(gCsProcessDb.localRole);

            // Process remote data from TI L2CAPCOC protocol
            status = csProcess_ProcessSubeventResultsSteps((csSubeventResultsStep_t*)pParams->data, pParams->numAntennaPath,
                                                            pParams->numStepsReported, role, pParams->totalBytesProcessed);
            break;
        }
        case CS_RESULTS_MODE_RAS:
        {
    #ifdef RANGING_CLIENT
            // The remote device role is the opposite of the local role
            role = CS_GET_OPPOSITE_ROLE(gCsProcessDb.localRole);

            // Process remote data provided by RAS profile
            status = csProcess_ProcessSubeventResultsStepsRAS((Ranging_subEventResultsStep_t*)pParams->data, pParams->numAntennaPath,
                                                                pParams->numStepsReported, role, pParams->totalBytesProcessed);
    #else
            status = CS_PROCESS_RESULTS_MODE_NOT_SUPPORTED;
    #endif
            break;
        }
        case CS_RESULTS_MODE_BTCS:
        {
            // The remote device role is the opposite of the local role
            role = CS_GET_OPPOSITE_ROLE(gCsProcessDb.localRole);

            // Process remote data provided by BTCS
            status = csProcess_ProcessSubeventResultsStepsBTCS(pParams->data, pParams->numAntennaPath,
                                                                pParams->numStepsReported, role, pParams->totalBytesProcessed);

            break;
        }
        default:
        {
            status = CS_PROCESS_RESULTS_MODE_NOT_SUPPORTED;
            break;
        }
        }
    }

    // Validate mode-0 steps
    if (status == CS_PROCESS_SUCCESS)
    {
        status = csProcess_StepsPostProcess(role, pParams->subeventDoneStatus, pParams->procedureDoneStatus);
    }

    // Check if distance is ready to be calculated and report if it is
    if (status == CS_PROCESS_SUCCESS)
    {
        status = csProcess_CheckIfDone();
    }
    else
    {
        // In case of any failure - clear all of the data
        // TODO: Can be removed if handled outside of this module
        csProcess_ClearProcedureData();
    }

    return status;
}

/*******************************************************************************
 * @fn    csProcess_ProcessSubeventResultsSteps
 *
 * @brief Processes CS subevent results steps and adds them to the Ranging module.
 *
 * This function handles the processing of CS subevent results steps and integrates
 * them into the Ranging module for further use.
 *
 * @param      subeventResultsSteps - Pointer to the source subevent results steps.
 * @param      numAntennaPath       - Number of antenna paths supported.
 * @param      numStepsReported     - Number of steps reported in the given subevent.
 * @param      role                 - The role of the device which measured the steps.
 * @param[out] totalBytesProcessed  - Pointer to store the total number of bytes
 *                                    processed by the function.
 *
 * @return CS_PROCESS_SUCCESS The operation completed successfully.
 * @return CS_PROCESS_TOO_MANY_STEPS_PROVIDED Too many steps provided.
 * @return CS_PROCESS_STEPS_PROCESSING_FAILED The operation failed due to invalid parameters or step data length.
 * @return CS_PROCESS_MODE_NOT_SUPPORTED The given mode parameter is not supported or not recognized.
 * @return CS_PROCESS_INVALID_STEP_PARAM - One of step parameters is invalid.
 */
csProcessStatus_e csProcess_ProcessSubeventResultsSteps(csSubeventResultsStep_t *subeventResultsSteps, uint8_t numAntennaPath,
                                                        uint8_t numStepsReported, uint8_t role, uint32_t *totalBytesProcessed)
{
  csProcessStatus_e status = CS_PROCESS_SUCCESS;
  uint8_t* tempResults = (uint8_t *)subeventResultsSteps;
  csSubeventResultsStep_t *srcStep;
  uint8_t expectedStepDataLen = 0;
  uint32_t bytesProcessed = 0;

  for (int i = 0; i < numStepsReported; i++)
  {
    // Ensure we don't process extra steps
    if ((role == gCsProcessDb.localRole && gCsProcessDb.currentStepLocal >= CS_PROCESS_MAX_STEPS) ||
        (role != gCsProcessDb.localRole && gCsProcessDb.currentStepRemote >= CS_PROCESS_MAX_STEPS))
    {
        status = CS_PROCESS_TOO_MANY_STEPS_PROVIDED;
    }

    // If any failure occurred during the process - break the loop
    if (status != CS_PROCESS_SUCCESS)
    {
        break;
    }

    // Determine the current step we are working on
    srcStep = (csSubeventResultsStep_t *)tempResults;

    // Get the expected data length for this mode
    expectedStepDataLen = CS_GetStepLength(srcStep->stepMode, role, numAntennaPath);

    // Check that the previous function returned a valid length,
    // and ensure that step data length equals to the expected length.
    if (expectedStepDataLen <= 0 ||
        expectedStepDataLen != srcStep->stepDataLen)
    {
        status = CS_PROCESS_STEPS_PROCESSING_FAILED;
    }

    // Process the step data depending on which role it is
    if (status == CS_PROCESS_SUCCESS)
    {
        status = csProcess_ProcessStep(srcStep->stepMode, (uint8_t*) &(srcStep->stepData), role, srcStep->stepChnl, numAntennaPath);
    }

    // Move the pointer to the next step and update bytes processed
    uint32_t stepLength = STEP_HDR_LEN + srcStep->stepDataLen;
    tempResults += stepLength;
    bytesProcessed += stepLength;
  }

  // Output the total bytes processed
  if (totalBytesProcessed != NULL)
  {
    *totalBytesProcessed = bytesProcessed;
  }

  return status;
}

#ifdef RANGING_CLIENT
/**
 * @fn    csProcess_ProcessSubeventResultsStepsRAS
 *
 * @brief Processes RAS subevent results steps and adds them to the Ranging module.
 *
 * This function handles the processing of RAS subevent results steps and integrates
 * them into the Ranging module for further use.
 *
 * @param      subeventResultsSteps - Pointer to the source subevent results steps (RAS).
 * @param      numAntennaPath       - Number of antenna paths supported.
 * @param      numStepsReported     - Number of steps reported in the given subevent.
 * @param      role                 - The role of the device which measured the steps.
 * @param[out] totalBytesProcessed  - Pointer to store the total number of bytes
 *                                    processed by the function.
 *
 * @return CS_PROCESS_SUCCESS The operation completed successfully.
 * @return CS_PROCESS_TOO_MANY_STEPS_PROVIDED Too many steps provided.
 * @return CS_PROCESS_STEPS_PROCESSING_FAILED The operation failed due to invalid parameters or step data length.
 * @return CS_PROCESS_MODE_NOT_SUPPORTED The given mode parameter is not supported or not recognized.
 * @return CS_PROCESS_INVALID_STEP_PARAM - One of step parameters is invalid.
 */
csProcessStatus_e csProcess_ProcessSubeventResultsStepsRAS(Ranging_subEventResultsStep_t *subeventResultsSteps, uint8_t numAntennaPath,
                                                           uint8_t numStepsReported, uint8_t role, uint32_t *totalBytesProcessed)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;
    uint8_t* tempResults = (uint8_t *)subeventResultsSteps;
    Ranging_subEventResultsStep_t *srcStep;
    uint8_t expectedStepDataLen = 0;
    uint32_t bytesProcessed = 0;

    for (int i = 0; i < numStepsReported; i++)
    {

      // Ensure we don't process extra steps
      if (gCsProcessDb.currentStepRemote >= CS_PROCESS_MAX_STEPS)
      {
          status = CS_PROCESS_TOO_MANY_STEPS_PROVIDED;
      }

      // If any failure occurred during the process - break the loop
      if (status != CS_PROCESS_SUCCESS)
      {
          break;
      }

      // Determine the current step we are working on
      srcStep = (Ranging_subEventResultsStep_t *)tempResults;

      // Calculate the expected step data length
      expectedStepDataLen = CS_GetStepLength(srcStep->stepMode, role, numAntennaPath);

      // Check that the previous function returned a valid length.
      // Ensure step data length equals to the expected one
      if (expectedStepDataLen <= 0)
      {
          status = CS_PROCESS_STEPS_PROCESSING_FAILED;
      }

      // Process the step data depending on which role it is
      if (status == CS_PROCESS_SUCCESS)
      {
          uint8_t stepChannel = gCsProcessDb.stepsIdxToChnlMap[gCsProcessDb.currentStepRemote];
          status = csProcess_ProcessStep(srcStep->stepMode, (uint8_t *)&(srcStep->stepData), role, stepChannel, numAntennaPath);
      }

      // Move the pointer to the next step and update bytes processed
      uint32_t stepLength = RAS_STEP_HDR_LEN + expectedStepDataLen;
      tempResults += stepLength;
      bytesProcessed += stepLength;
    }

    // Output the total bytes processed
    if (totalBytesProcessed != NULL)
    {
        *totalBytesProcessed = bytesProcessed;
    }

    return status;
}
#endif

/*******************************************************************************
 * @fn    csProcess_ProcessSubeventResultsStepsBTCS
 *
 * @brief Processes CS subevent results steps in BTCS format and adds them to the Ranging module.
 *
 * This function handles the processing of CS subevent results steps and integrates
 * them into the Ranging module for further use.
 *
 * @param      subeventResultsSteps - Pointer to the source subevent results steps.
 * @param      numAntennaPath       - Number of antenna paths supported.
 * @param      numStepsReported     - Number of steps reported in the given subevent.
 * @param      role                 - The role of the device which measured the steps.
 * @param[out] totalBytesProcessed  - Pointer to store the total number of bytes
 *                                    processed by the function.
 *
 * @return CS_PROCESS_SUCCESS The operation completed successfully.
 * @return CS_PROCESS_TOO_MANY_STEPS_PROVIDED Too many steps provided.
 * @return CS_PROCESS_MODE_NOT_SUPPORTED Mode not supported or invalid.
 * @return CS_PROCESS_INVALID_STEP_PARAM Invalid parameters.
 */
csProcessStatus_e csProcess_ProcessSubeventResultsStepsBTCS(uint8_t *subeventResultsSteps, uint8_t numAntennaPath,
                                                            uint8_t numStepsReported, uint8_t role, uint32_t *totalBytesProcessed)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;
    uint8_t* modeInfoIter = subeventResultsSteps;
    uint8_t* stepsIter = modeInfoIter +  CS_PROCESS_CEIL_DIVIDE_2(numStepsReported);
    uint32_t bytesProcessed = 0;

    for (int i = 0; i < numStepsReported; i++)
    {
      BTCS_stepMode_t currStepMode;

      // Ensure we don't process extra steps
      if (gCsProcessDb.currentStepRemote >= CS_PROCESS_MAX_STEPS)
      {
          status = CS_PROCESS_TOO_MANY_STEPS_PROVIDED;
      }

      // If any failure occurred during the process - break the loop
      if (status != CS_PROCESS_SUCCESS)
      {
          break;
      }

      // Determine the current step we are working on
      if (i % 2 == 0)
      {
          currStepMode.mode = BTCS_EXTRACT_MODE_FIRST(*modeInfoIter);
          currStepMode.status = BTCS_EXTRACT_STATUS_FIRST(*modeInfoIter);
      }
      else
      {
          currStepMode.mode = BTCS_EXTRACT_MODE_SECOND(*modeInfoIter);
          currStepMode.status = BTCS_EXTRACT_STATUS_SECOND(*modeInfoIter);
      }

      // Process the step data depending on which role it is
      if (status == CS_PROCESS_SUCCESS && currStepMode.status == 0)
      {
          uint8_t stepChannel = gCsProcessDb.stepsIdxToChnlMap[gCsProcessDb.currentStepRemote];
          status = csProcess_ProcessStepBTCS(currStepMode.mode, stepsIter, role, stepChannel, numAntennaPath);
      }

      if (status == CS_PROCESS_SUCCESS)
      {
          // Calculate step length
          uint8_t stepLength = csProcess_GetStepLengthBTCS(currStepMode.mode, role, numAntennaPath);

          // Increment steps iterator
          stepsIter += stepLength;
          bytesProcessed += stepLength;

          // Increment byteProcessed by 1 for step mode processing.
          // Do it only for even steps indices because each byte of the stepmodes data includes 2 steps (4 bits for a step)
          if (i % 2 == 0)
          {
              bytesProcessed += 1;
          }

          // Increment mode iterator only when the loop iterator is odd
          if (i % 2 == 1)
          {
              modeInfoIter += 1;
          }
      }
    }

    if (status == CS_PROCESS_SUCCESS && (numStepsReported % 2 == 1))
    {
        // If the number of reported steps is odd - we didn't process the last mode_step,
        // therefore we need to compensate for it.
        bytesProcessed += 1;
    }

    // Output the total bytes processed
    if (totalBytesProcessed != NULL)
    {
        *totalBytesProcessed = bytesProcessed;
    }

    return status;
}

/*******************************************************************************
 * @fn    csProcess_GetSubeventCounter
 *
 * @brief Returns the current subevent counter for local \ remote device as
 *        saved in the DB.
 *        The range of the subevent counter is between 0 to @ref CS_MAX_SUBEVENTS_PER_PROCEDURE.
 *        When subeventCounter reaches to @ref CS_MAX_SUBEVENTS_PER_PROCEDURE, it
 *        means that no more subevents should be processed.
 *
 * @param mode - Source of the requested subevent counter
 *
 * @return local \ remote subevent counter
 */
uint8_t csProcess_GetSubeventCounter(ChannelSounding_resultsSourceMode_e mode)
{
    return mode == CS_RESULTS_MODE_LOCAL ? gCsProcessDb.subeventCounterLocal : gCsProcessDb.subeventCounterRemote;
}

/*******************************************************************************
 * @fn    csProcess_IsAllowedToIncrementSubeventCounter
 *
 * @brief Checks if the subevent counter for local \ remote device can be incremented
 *        Maximum value for the subevent counter is @ref CS_MAX_SUBEVENTS_PER_PROCEDURE,
 *        therefore the subevent counter can be incremented only if it is less than this value.
 *
 * @param mode - Source of the requested subevent counter
 *
 * @return true if the subevent counter can be incremented, false otherwise
 */
bool csProcess_IsAllowedToIncrementSubeventCounter(ChannelSounding_resultsSourceMode_e mode)
{
    if (mode == CS_RESULTS_MODE_LOCAL)
    {
        return (gCsProcessDb.subeventCounterLocal < CS_MAX_SUBEVENTS_PER_PROCEDURE) ? true : false;
    }
    else
    {
        return (gCsProcessDb.subeventCounterRemote < CS_MAX_SUBEVENTS_PER_PROCEDURE) ? true : false;
    }
}

/*******************************************************************************
 * @fn    csProcess_IncrementSubeventCounter
 *
 * @brief Increment the current subevent counter for local \ remote device as
 *        saved in the DB.
 *        When subeventCounter reaches to @ref CS_MAX_SUBEVENTS_PER_PROCEDURE, it
 *        means that no more subevents should be processed.
 *        Use this function after validating that the subevent counter can be incremented
 *        using @ref csProcess_IsAllowedToIncrementSubeventCounter.
 *
 * @param mode - Source of the requested subevent counter
 *
 * @return None
 */
void csProcess_IncrementSubeventCounter(ChannelSounding_resultsSourceMode_e mode)
{
    if (mode == CS_RESULTS_MODE_LOCAL)
    {
        gCsProcessDb.subeventCounterLocal++;
    }
    else
    {
        gCsProcessDb.subeventCounterRemote++;
    }
}

/*******************************************************************************
 * @fn    csProcess_GetSubeventRpl
 *
 * @brief Returns and RPL (Reference Power Level) for a specific subevent.
 *
 * @param mode - Source of the requested subevent counter
 * @param subeventIndex - The subevent index for which the RPL is requested.
 *                        Range is between 0 to @ref CS_MAX_SUBEVENTS_PER_PROCEDURE - 1.
 *
 * @return CS_INVALID_TX_POWER - if the subeventIndex is invalid or if the RPL is not set.
 * @return The RPL value for the requested subevent.
 */
int8_t csProcess_GetSubeventRpl(ChannelSounding_resultsSourceMode_e mode, uint8_t subeventIndex)
{
    if (subeventIndex >= CS_MAX_SUBEVENTS_PER_PROCEDURE)
    {
        return CS_INVALID_TX_POWER;
    }

    return mode == CS_RESULTS_MODE_LOCAL ? gCsProcessDb.localRpl[subeventIndex] : gCsProcessDb.remoteRpl[subeventIndex];
}

/*******************************************************************************
 * @fn    csProcess_SetSubeventRpl
 *
 * @brief Sets an RPL (Reference Power Level) for a specific subevent.
 *
 * @param mode - Source of the requested subevent counter
 * @param subeventIndex - The subevent index for which the RPL is set.
 *                        Range is between 0 to @ref CS_MAX_SUBEVENTS_PER_PROCEDURE - 1.
 * @param rpl - The RPL value to set for the subevent.
 *              Range is between CS_MIN_TX_POWER_VALUE and CS_MAX_TX_POWER_VALUE.
 *
 * @return None
 */
void csProcess_SetSubeventRpl(ChannelSounding_resultsSourceMode_e mode, uint8_t subeventIndex, int8_t rpl)
{
    if ((rpl >= CS_MIN_TX_POWER_VALUE) &&
        (rpl <= CS_MAX_TX_POWER_VALUE) &&
        (subeventIndex < CS_MAX_SUBEVENTS_PER_PROCEDURE))
    {
        if (mode == CS_RESULTS_MODE_LOCAL)
        {
            gCsProcessDb.localRpl[subeventIndex] = rpl;
        }
        else
        {
            gCsProcessDb.remoteRpl[subeventIndex] = rpl;
        }
    }
}

/**
 * @fn    csProcess_ProcessStep
 *
 * @brief Processes a single step.
 *
 * @param mode           - The mode of the step.
 * @param stepData       - Pointer to the data associated with the current step.
 * @param role           - The role of the device reported about the results.
 * @param stepChannel    - The channel associated with the current step.
 * @param numAntennaPath - The number of antenna paths used for this step.
 *
 * @return CS_PROCESS_SUCCESS The step has been successfully processed.
 * @return CS_PROCESS_MODE_NOT_SUPPORTED The given mode parameter is not supported or not recognized.
 * @return CS_PROCESS_INVALID_STEP_PARAM - One of the other parameters is invalid.
 */
csProcessStatus_e csProcess_ProcessStep(uint8_t mode, uint8_t *stepData, uint8_t role, uint8_t stepChannel, uint8_t numAntennaPath)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;

    switch(mode)
    {
      case CS_MODE_0:
      {
        csProcess_ProcessModeZeroStep(stepData, role);
        break;
      }
      case CS_MODE_1:
      {
        // Mode not supported
        status = CS_PROCESS_MODE_NOT_SUPPORTED;
        break;
      }
      case CS_MODE_2:
      {
        // Cast step data to Mode-2 step data
        CS_modeTwoStep_t *modeTwoStep = (CS_modeTwoStep_t *)stepData;

        // Add step results to the Ranging module - In case of failure break on the next loop
        status = csProcess_ProcessModeTwoStep(modeTwoStep, role, stepChannel, numAntennaPath);
        break;
      }
      case CS_MODE_3:
      {
        // Cast step data to Mode-3 step data
        CS_modeThreeStep_t *modeThreeStep = (CS_modeThreeStep_t *)stepData;

        // Add step results to the Ranging module - In case of failure break on the next loop
        status = csProcess_ProcessModeTwoStep((CS_modeTwoStep_t *)&modeThreeStep->antennaPermutationIndex, role, stepChannel, numAntennaPath);
        break;
      }
      default:
      {
          status = CS_PROCESS_MODE_NOT_SUPPORTED;
          break;
      }
    }

    return status;
}

/**
 * @fn    csProcess_ProcessStepBTCS
 *
 * @brief Processes a single step in BTCS format.
 *
 * @param mode           - The mode of the step.
 * @param stepData       - Pointer to the data associated with the current step.
 * @param role           - The role of the device reported about the results.
 * @param stepChannel    - The channel associated with the current step.
 * @param numAntennaPath - The number of antenna paths used for this step.
 *
 * @return CS_PROCESS_SUCCESS The step has been successfully processed.
 * @return CS_PROCESS_MODE_NOT_SUPPORTED The given mode parameter is not supported or not recognized.
 * @return CS_PROCESS_INVALID_STEP_PARAM One of the step parameters is invalid.
 */
csProcessStatus_e csProcess_ProcessStepBTCS(uint8_t mode, uint8_t *stepData, uint8_t role, uint8_t stepChannel, uint8_t numAntennaPath)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;

    switch(mode)
    {
      case CS_MODE_0:
      {
          csProcess_ProcessModeZeroStep(stepData, role);
        break;
      }
      case CS_MODE_1:
      {
        // Mode not supported
        status = CS_PROCESS_MODE_NOT_SUPPORTED;
        break;
      }
      case CS_MODE_2:
      {
        // Cast step data to Mode-2 step data
        BTCS_modeTwoStep_t *modeTwoStep = (BTCS_modeTwoStep_t *)stepData;

        // Add step results to the Ranging module
        status = csProcess_ProcessModeTwoStepBTCS(modeTwoStep, role, stepChannel, numAntennaPath);
        break;
      }
      case CS_MODE_3:
      {
        // Cast step data to Mode-3 step data
        BTCS_modeThreeStep_t *modeThreeStep = (BTCS_modeThreeStep_t *)stepData;

        // Add step results to the Ranging module
        status = csProcess_ProcessModeTwoStepBTCS(&(modeThreeStep->data), role, stepChannel, numAntennaPath);
        break;
      }
      default:
      {
        status = CS_PROCESS_MODE_NOT_SUPPORTED;
        break;
      }
    }

    return status;
}

/**
 * @fn    csProcess_ProcessModeZeroStep
 *
 * @brief Processes a mode-0 step for the current procedure.
 *
 * @param stepData       - Pointer to the data associated with the current step.
 *                         One of the following types: @ref CS_modeZeroInitStep_t or @ref CS_modeZeroReflStep_t
 * @param role           - The role of the device reported about the results.
 *
 * @return None
 */
void csProcess_ProcessModeZeroStep(uint8_t *stepData, uint8_t role)
{
    uint8_t packetQuality;      // Packet quality value given by the step data
    uint8_t packetRssi;         // Packet RSSI value given by the step data
    uint8_t modeZeroValidFlag;  // State flag for checking if mode-0 steps has been processed successfully (vary by role)
    uint8_t peerRole = CS_GET_OPPOSITE_ROLE(gCsProcessDb.localRole); // The role of the peer device

    // Extract step data
    if (role == CS_ROLE_INITIATOR)
    {
        modeZeroValidFlag = CS_PROCESS_STATE_INITIATOR_MODE_ZERO_VALID;
        packetQuality = ((CS_modeZeroInitStep_t *)stepData)->packetQuality;
        packetRssi = ((CS_modeZeroInitStep_t *)stepData)->packetRssi;
    }
    else
    {
        modeZeroValidFlag = CS_PROCESS_STATE_REFLECTOR_MODE_ZERO_VALID;
        packetQuality = ((CS_modeZeroReflStep_t *)stepData)->packetQuality;
        packetRssi = ((CS_modeZeroReflStep_t *)stepData)->packetRssi;
    }

    // if the step data is valid based on the packet quality
    if (packetQuality == 0)
    {
        // Mark that the results data are valid if at least one valid mode-0 step
        gCsProcessDb.state |= modeZeroValidFlag;

        // If processing peer's step - update the peer's max RSSI, if needed
        if (role == peerRole)
        {
            if (gCsProcessDb.peerMaxRssi == CS_INVALID_TX_POWER)
            {
                // This is the first step data received from the peer, set the max RSSI to the current one
                gCsProcessDb.peerMaxRssi = (int8_t) packetRssi;
            }
            else
            {
                // Take the maximum of the current max RSSI and the new one
                gCsProcessDb.peerMaxRssi = Math_MAX(gCsProcessDb.peerMaxRssi, ((int8_t) packetRssi));
            }
        }
    }

#ifdef CS_PROCESS_EXT_RESULTS
    // Add mode-0 steps data for extended results
    if (role == CS_ROLE_INITIATOR && gCsProcessDb.currentModeZeroStepInit < CS_MAX_MODE_ZERO_PER_PROCEDURE)
    {
        gCsProcessDb.modeZeroStepsInit[gCsProcessDb.currentModeZeroStepInit] = *((CS_modeZeroInitStep_t *)(stepData));
        gCsProcessDb.currentModeZeroStepInit++;
    }
    else if (role == CS_ROLE_REFLECTOR && gCsProcessDb.currentModeZeroStepRef < CS_MAX_MODE_ZERO_PER_PROCEDURE)
    {
        gCsProcessDb.modeZeroStepsRef[gCsProcessDb.currentModeZeroStepRef] = *((CS_modeZeroReflStep_t *)(stepData));
        gCsProcessDb.currentModeZeroStepRef++;
    }
#endif
}

/**
 * @fn csProcess_ProcessModeTwoStep
 *
 * @brief Adds step data to the CSProcess database.
 *
 * This function adds step data for a specific role (initiator or reflector)
 * and channel index. It validates the input parameters and updates the
 * database with the provided data.
 *
 * @param stepDataToAdd  -  Pointer to the step data to add.
 * @param role           -  Role of the device (initiator or reflector).
 * @param stepChannel    -  Channel used for this the step (2 to 76, excluding 23-25).
 * @param numAntennaPath -  Number of antenna paths reported for this step.
 *
 * @return CS_PROCESS_SUCCESS if data is added successfully.
 * @return CS_PROCESS_INVALID_STEP_PARAM one of the step parameters is invalid.
 */
csProcessStatus_e csProcess_ProcessModeTwoStep(CS_modeTwoStep_t* stepDataToAdd, uint8_t role, uint8_t stepChannel, uint8_t numAntennaPath)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;

    // Check Parameters
    if ((gCsProcessDb.currSession == CS_PROCESS_INVALID_SESSION) ||
        (stepDataToAdd == NULL) || (role > CS_ROLE_REFLECTOR) ||
        (stepChannel < 2 || stepChannel > 76 ||  (stepChannel >= 23 && stepChannel <= 25)) || // Channels which are not in use
        (stepDataToAdd->antennaPermutationIndex > CS_RANGING_MAX_PERMUTATION_INDEX) ||
        (numAntennaPath == 1 && stepDataToAdd->antennaPermutationIndex > CS_MAX_PERMUTATION_INDEX_1_ANT) ||
        (numAntennaPath == 2 && stepDataToAdd->antennaPermutationIndex > CS_MAX_PERMUTATION_INDEX_2_ANT) ||
        (numAntennaPath == 3 && stepDataToAdd->antennaPermutationIndex > CS_MAX_PERMUTATION_INDEX_3_ANT) ||
        (numAntennaPath == 4 && stepDataToAdd->antennaPermutationIndex > CS_MAX_PERMUTATION_INDEX_4_ANT))
    {
        status = CS_PROCESS_INVALID_STEP_PARAM;
    }
    else
    {
        // convert channel to channel index in order match the DB array indices
        uint8_t channelIndex = stepChannel - 2;

        // Get the permutation derived by permutation index
        uint8_t permutation = antennaPermutations[stepDataToAdd->antennaPermutationIndex];

        // Set the permutation into the DB for this step to be used later when the remote results will arrive
        if (role == gCsProcessDb.localRole)
        {
            gCsProcessDb.stepsIdxToAntPermutationMap[gCsProcessDb.currentStepLocal] = permutation;
        }

        for (uint8_t j = 0; j < numAntennaPath; j++)
        {
            BleCsRanging_Tone_t data;

            // Get the antenna path derived by the permutation
            uint8_t pathIndex = permutation & 0x3;
            uint16_t pathOffset = (pathIndex * CS_RANGING_PCT_ARRAY_SIZE);

            // Copy the given step results
            data.i = stepDataToAdd->data[j].i;
            data.q = stepDataToAdd->data[j].q;
            data.quality = stepDataToAdd->data[j].tqi;

            // Add the given data to DB based on the role
            if (role == CS_ROLE_INITIATOR)
            {
                gCsProcessDb.initPathsData[channelIndex + pathOffset] = data;
            }
            else
            {
                gCsProcessDb.refPathsData[channelIndex + pathOffset] = data;
            }

            // Move the permutation by two bits in order to get the next path for the next loop
            permutation = permutation >> 2;
        }

        // Step post process
        csProcess_ModeTwoStepPostProcess(role, stepChannel);

#ifdef CS_PROCESS_EXT_RESULTS
        // Save the permutation indices for extended results
        if (role == gCsProcessDb.localRole)
        {
            gCsProcessDb.permutationIndexLocal[channelIndex] = stepDataToAdd->antennaPermutationIndex;
        }
        else
        {
            gCsProcessDb.permutationIndexRemote[channelIndex] = stepDataToAdd->antennaPermutationIndex;
        }
#endif
    }

    return status;
}

/**
 * @fn csProcess_ProcessModeTwoStepBTCS
 *
 * @brief Adds step data of BTCS format to the CSProcess database.
 *
 * This function adds step data for a specific role (initiator or reflector)
 * and channel index. It validates the input parameters and updates the
 * database with the provided data.
 *
 * @param stepDataToAdd  -  Pointer to the step data to add.
 * @param role           -  Role of the device (initiator or reflector).
 * @param stepChannel    -  Channel used for this the step (2 to 76, excluding 23-25).
 * @param numAntennaPath -  Number of antenna paths reported for this step.
 *
 * @return CS_PROCESS_SUCCESS if data is added successfully.
 * @return CS_PROCESS_INVALID_STEP_PARAM one of the step parameters is invalid.
 */
csProcessStatus_e csProcess_ProcessModeTwoStepBTCS(BTCS_modeTwoStep_t* stepDataToAdd, uint8_t role, uint8_t stepChannel, uint8_t numAntennaPath)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;

    // Check Parameters
    if ((gCsProcessDb.currSession == CS_PROCESS_INVALID_SESSION) ||
        (stepDataToAdd == NULL) || (role > CS_ROLE_REFLECTOR) ||
        (stepChannel < 2 || stepChannel > 76 ||  (stepChannel >= 23 && stepChannel <= 25)) || // Channels which are not in use
        (numAntennaPath > 4))
    {
        status = CS_PROCESS_INVALID_STEP_PARAM;
    }
    else
    {
        // correct the channel index to match the DB array indices
        uint8_t channelIndex = stepChannel - 2;

        // Get the permutation from the DB
        uint8_t permutation = gCsProcessDb.stepsIdxToAntPermutationMap[gCsProcessDb.currentStepRemote];

        for (uint8_t j = 0; j < numAntennaPath; j++)
        {
            BleCsRanging_Tone_t data;

            uint8_t pathIndex = permutation & 0x3;

            uint16_t pathOffset = (pathIndex * CS_RANGING_PCT_ARRAY_SIZE);

            // Copy the given step results
            BTCS_modeTwoStepData_t* stepIQData = (BTCS_modeTwoStepData_t*) (stepDataToAdd->IQData + (j * 3));
            data.i = stepIQData->i;
            data.q = stepIQData->q;

            // Quality is ordered by MSB - (A4, A3, A2, A1) - LSB.
            // Therefore take the quality bits using the pathIndex
            data.quality = ((stepDataToAdd->quality) >> pathIndex) & 0x3;

            // Add the given data to DB based on the role
            if (role == CS_ROLE_INITIATOR)
            {
                gCsProcessDb.initPathsData[channelIndex + pathOffset] = data;
            }
            else
            {
                gCsProcessDb.refPathsData[channelIndex + pathOffset] = data;
            }

            // Move the permutation by two bits in order to get the next path for the next loop
            permutation = permutation >> 2;
        }

        // Step post process
        csProcess_ModeTwoStepPostProcess(role, stepChannel);
    }

    return status;
}

/**
 * @fn csProcess_ModeTwoStepPostProcess
 *
 * @brief Handles different global iterators after processing a mode-2 step.
 *
 * @param role           -  Role of the device (initiator or reflector).
 * @param stepChannel    -  Channel used for the processed step.
 *
 * @return None
 *
 * @Warning - Assumes parameters are valid
 */
void csProcess_ModeTwoStepPostProcess(uint8_t role, uint8_t stepChannel)
{
    // Handle different iterators, depends on the source of the steps
    if (role == gCsProcessDb.localRole)
    {
        // When processing local steps - save the channel of this step for processing the remote data later on
        gCsProcessDb.stepsIdxToChnlMap[gCsProcessDb.currentStepLocal] = stepChannel;
        gCsProcessDb.currentStepLocal++;
    }
    else
    {
        // When processing remote steps - increase the Remote Step Iterator
        gCsProcessDb.currentStepRemote++;
    }
}

/**
 * @fn csProcess_StepsPostProcess
 *
 * @brief Posts processing of subevent steps after some steps has been added
 *        for a specific role.
 *        This function Validates mode-0 steps based on the given role, Subevent Done Status,
 *        Procedure Done Status and the current CSProcess database state.
 *
 *        When Subevent Done Status is @ref CS_SUBEVENT_DONE:
 *        1. checks that the mode-0 valid flag for the given role is set,
 *           indicating at least one valid mode-0 step for the last processed subevent.
 *
 *        2. If Procedure Done Status is @ref CS_PROCEDURE_DONE - checks that mode-0 steps were processed
 *           successfully and if so, marks the data flag as ready in the DB state.
 *           Clears the mode-0 state flag for the next subevent.
 *
 * @param role                  -  Role of the device which reported the subevent.
 * @param subeventDoneStatus    -  Subevent Done Status reported parameter.
 * @param procedureDoneStatus   -  Procedure Done Status reported parameter.
 *
 * @return CS_PROCESS_INCORRECT_NUMBER_OF_STEPS - procedureDoneStatus is 0 but the total main mode steps reported is not @ref CS_PROCESS_ALLOWED_NUMBER_OF_STEPS.
 * @return CS_PROCESS_MODE_ZERO_CHECK_FAILED    - Subevent Done status is @ref CS_SUBEVENT_DONE and the mode-0 steps reported by the given role are not valid.
 * @return CS_PROCESS_SUCCESS - Otherwise
 */
csProcessStatus_e csProcess_StepsPostProcess(uint8_t role, uint8_t subeventDoneStatus, uint8_t procedureDoneStatus)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;

    // State flags - Will be set depends on the given role
    uint8_t modeZeroValidFlag;      // State flag for  checking if mode-0 steps has been processed successfully
    uint8_t dataReadyFlag;          // State flag for checking if all of the procedure steps has been processed successfully
    uint8_t expectedMainModeSteps;  // Expected number of main mode steps for the given role

    // Set state flags
    if (role == CS_ROLE_INITIATOR)
    {
        modeZeroValidFlag = (uint8_t) CS_PROCESS_STATE_INITIATOR_MODE_ZERO_VALID;
        dataReadyFlag = (uint8_t) CS_PROCESS_STATE_INITIATOR_DATA_READY;
    }
    else
    {
        modeZeroValidFlag = (uint8_t) CS_PROCESS_STATE_REFLECTOR_MODE_ZERO_VALID;
        dataReadyFlag = (uint8_t) CS_PROCESS_STATE_REFLECTOR_DATA_READY;
    }

    // Get expected number of main mode steps for the given role
    if (role == gCsProcessDb.localRole)
    {
        expectedMainModeSteps = gCsProcessDb.currentStepLocal;
    }
    else
    {
        expectedMainModeSteps = gCsProcessDb.currentStepRemote;
    }

    // If procedure is done, check that the number of steps is exactly as expected
    if (procedureDoneStatus == CS_PROCEDURE_DONE &&
        expectedMainModeSteps != CS_PROCESS_ALLOWED_NUMBER_OF_STEPS)
    {
        status = CS_PROCESS_INCORRECT_NUMBER_OF_STEPS;
    }

    // If subevent is done, validate mode-0 steps
    if (status == CS_PROCESS_SUCCESS && subeventDoneStatus == CS_SUBEVENT_DONE)
    {
        // Check that we processed mode-0 steps successfully
        if ((gCsProcessDb.state & modeZeroValidFlag) == modeZeroValidFlag)
        {
            if (procedureDoneStatus == CS_PROCEDURE_DONE)
            {
                // All results has been processed - mark data as ready
                gCsProcessDb.state |= dataReadyFlag;
            }
            else
            {
                // Clear the mode-0 state flag for the next subevent
                gCsProcessDb.state &= (~modeZeroValidFlag);
            }
        }
        else
        {
            // Subevent mode-0 wasn't valid - consider as a failure
            status = CS_PROCESS_MODE_ZERO_CHECK_FAILED;
        }
    }

    return status;
}

/**
 * @fn csProcess_CheckIfDone
 *
 * @brief  Checks if initiator and reflector data is fully collected.
 *         If both roles data collected - report that the distance is ready to be estimated.
 *
 * @param  None
 *
 * @return CS_PROCESS_DISTANCE_ESTIMATION_PENDING - If the distance is ready to be estimated.
 * @return CS_PROCESS_SUCCESS - otherwise
 */
csProcessStatus_e csProcess_CheckIfDone()
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;

    // If the procedure is active and both initiator and reflector data are ready
    if ((gCsProcessDb.currSession != CS_PROCESS_INVALID_SESSION) &&
        ((gCsProcessDb.state & CS_PROCESS_STATE_INITIATOR_DATA_READY) == CS_PROCESS_STATE_INITIATOR_DATA_READY ) &&
        ((gCsProcessDb.state & CS_PROCESS_STATE_REFLECTOR_DATA_READY) == CS_PROCESS_STATE_REFLECTOR_DATA_READY))
    {
        status = CS_PROCESS_DISTANCE_ESTIMATION_PENDING;
    }

    return status;
}

/**
 * @fn csProcess_CalculateRPL
 *
 * @brief  Calculates the Ranging Power Level (RPL) based on the received RPL values.
 *
 * @param  rpl       Pointer to the array of RPL values.
 * @param  numOfRpl  Number of RPL values in the array.
 *
 * @return The calculated RPL value or CS_INVALID_TX_POWER if no valid RPL is found.
 */
int8_t csProcess_CalculateRPL(int8_t *rpl, uint8_t numOfRpl)
{
    int8_t resultRpl = CS_INVALID_TX_POWER; // Default value if no valid RPL is found

    // Iterate through the RPL values and find the first valid one
    for (uint8_t i = 0; i < numOfRpl; i++)
    {
        if (rpl[i] != CS_INVALID_TX_POWER)
        {
            resultRpl = rpl[i];
            break;
        }
    }

    return resultRpl;
}

float _velocity;

/**
 * @fn csProcess_CalcResults
 *
 * @brief  Calculates ranging results based on the collected data by
 *         sending it to an external api.
 *
 * @param  csResults Pointer to the structure to store the results.
 *
 * @return CS_PROCESS_SUCCESS - results are calculated successfully.
 * @return CS_PROCESS_PROCEDURE_NOT_ACTIVE - If the procedure is not active
 * @return CS_PROCESS_DISTANCE_ESTIMATION_FAILED - If the BleCsRanging API fails to execute.
 */
csProcessStatus_e csProcess_CalcResults(CSProcess_Results_t *csResults)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;
    float deltaTime;                                    // Delta time between current time and previous measurement time
    uint32_t currTime;                                  // Current time
    BleCsRanging_Result_t results = {0};                // Holds the output results from BleCsRanging module
    results.pDebugResult = NULL;                        // By default, set debug info to NULL

#ifdef CS_PROCESS_EXT_RESULTS
    // If debug info is requested
    BleCsRanging_DebugResult_t debugResult = {0};   // Holds the output debug info from BleCsRanging module
    if (NULL != csResults->extendedResults)
    {
        results.pDebugResult = &debugResult;
    }
#endif

    // Check that there is a procedure currently active
    if (gCsProcessDb.currSession == CS_PROCESS_INVALID_SESSION)
    {
        status = CS_PROCESS_PROCEDURE_NOT_ACTIVE;
    }

    if (status == CS_PROCESS_SUCCESS)
    {
        BleCsRanging_Status_e BleCsRangingStatus;
        BleCsRanging_Tone_t* localResults;
        BleCsRanging_Tone_t* remoteResults;
        int8_t localRpl = CS_INVALID_TX_POWER;
        int8_t remoteRpl = CS_INVALID_TX_POWER;

        // Switch the results arrays if needed.
        // Note: In the current version of BleCsRanging, it is assumed that the initiator is the local device.
        //       Since TI devices radio may report a scalar value of TQI, which might be different than non-TI devices -
        //       we need to set the local device results into the Initiator field of the API.
        // Note: The API actually does not consider the role when estimating the distance, therefore it is
        //       safe to switch between the fields in order to handle the issue described above.
        if (gCsProcessDb.localRole == CS_ROLE_INITIATOR)
        {
            localResults = gCsProcessDb.initPathsData;
            remoteResults = gCsProcessDb.refPathsData;
        }
        else
        {
            localResults = gCsProcessDb.refPathsData;
            remoteResults = gCsProcessDb.initPathsData;
        }

        remoteRpl = csProcess_CalculateRPL(gCsProcessDb.remoteRpl, CS_MAX_SUBEVENTS_PER_PROCEDURE);
        localRpl = csProcess_CalculateRPL(gCsProcessDb.localRpl, CS_MAX_SUBEVENTS_PER_PROCEDURE);
        // Call the API function to estimate the distance.
        BleCsRangingStatus = BleCsRanging_estimatePbr(&results,
                                                      localResults,
                                                      remoteResults,
                                                      &localRpl,
                                                      &remoteRpl,
                                                      gCsProcessDb.stepsIdxToChnlMap,
                                                      &(gCsProcessDb.config));

        if (BleCsRangingStatus != BleCsRanging_Status_Success)
        {
            status = CS_PROCESS_DISTANCE_ESTIMATION_FAILED;
        }
        else
        {
            _velocity = results.velocity;
            // Get current time
            currTime = llGetCurrentTime();

            // Get current session DB
            CSProcessSessionData* currSession = &(gCsProcessDb.sessionsDb[gCsProcessDb.currSession]);

            // Initiate filtering for the first distance measurement.
            // This should only be done before the first usage of the filtering
            if (currSession->filteringDb.initDone == FALSE)
            {
                // Init filters DB
                currSession->filteringDb.initDone = TRUE;
                currSession->filteringDb.lastTimeTicks = currTime;
                deltaTime = ((float) currTime) / ((float) RAT_TICKS_IN_1S);

                BleCsRanging_initSlewRateLimiterFilter(&currSession->filteringDb.srlfFilter, 3.0f, gCsProcessDb.config.iirCoeff, BleCsRanging_SlewRateLimiterFilter_MA2);
            }
            else
            {
                // Add delta between current time and previous time difference to the filtering time
                deltaTime = ((float) llTimeAbs(currSession->filteringDb.lastTimeTicks, currTime)) / ((float) RAT_TICKS_IN_1S);

                // Update last time measure to be the current time
                currSession->filteringDb.lastTimeTicks = currTime;
            }

            // Filter the raw results and get the final distance
            if (results.confidence > 0.0f)
            {
                // Filter new estimate
                results.distance = BleCsRanging_computeSlewRateLimiterFilter(&currSession->filteringDb.srlfFilter, deltaTime, results.distance, results.velocity);
            }
            else
            {
                // Bad estimate, repeat previous instead
                results.distance = currSession->filteringDb.srlfFilter.prevValue;
            }

            // Copy the results, while converting floats to 32bits without losing the data after the decimal point
            csResults->distance = (uint32_t) (results.distance * 100);
            csResults->quality = (uint32_t) (results.quality * 100);
            csResults->confidence = (uint32_t) (results.confidence * 100);
        }

#ifdef CS_PROCESS_EXT_RESULTS
        // If the caller requested debug information - fill it (even if BleCsRanging_estimatePbr failed)
        if (NULL != csResults->extendedResults)
        {
            // Add BleCsRanging extended results
            csResults->extendedResults->numMpc = results.numMPC;

            // Copy different paths data
            for (uint8_t i = 0; i < MAX_NUM_ANTPATH; i++)
            {
                csResults->extendedResults->distanceMusic[i]    = (uint32_t) (debugResult.distanceMusic[i] * 100);
                csResults->extendedResults->distanceNN[i]       = (uint32_t) (debugResult.distanceNN[i] * 100);
                csResults->extendedResults->numMpcPaths[i]      = debugResult.numMPC[i];
                csResults->extendedResults->qualityPaths[i]     = (uint32_t) (debugResult.quality[i] * 100);
                csResults->extendedResults->confidencePaths[i]  = (uint32_t) (debugResult.confidence[i] * 100);
            }

            memcpy(csResults->extendedResults->localRpl,
                   gCsProcessDb.localRpl,
                   sizeof(csResults->extendedResults->localRpl));

            memcpy(csResults->extendedResults->remoteRpl,
                   gCsProcessDb.remoteRpl,
                   sizeof(csResults->extendedResults->remoteRpl));

            memcpy(csResults->extendedResults->modeZeroStepsInit,
                   gCsProcessDb.modeZeroStepsInit,
                   sizeof(csResults->extendedResults->modeZeroStepsInit));

            memcpy(csResults->extendedResults->modeZeroStepsRef,
                   gCsProcessDb.modeZeroStepsRef,
                   sizeof(csResults->extendedResults->modeZeroStepsRef));

            memcpy(csResults->extendedResults->permutationIndexLocal,
                   gCsProcessDb.permutationIndexLocal,
                   sizeof(csResults->extendedResults->permutationIndexLocal));

            memcpy(csResults->extendedResults->stepsDataLocal,
                   localResults,
                   sizeof(csResults->extendedResults->stepsDataLocal));

            memcpy(csResults->extendedResults->permutationIndexRemote,
                   gCsProcessDb.permutationIndexRemote,
                   sizeof(csResults->extendedResults->permutationIndexRemote));

            memcpy(csResults->extendedResults->stepsDataRemote,
                   remoteResults,
                   sizeof(csResults->extendedResults->stepsDataRemote));
        }
#endif
    }

    return status;
}

#endif /* CHANNEL_SOUNDING */
