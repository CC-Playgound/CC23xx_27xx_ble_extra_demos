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
#include <ti/drivers/utils/List.h>

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
    CS_PROCESS_STATE_LOCAL_DATA_READY           =   0x02, //!< Finished collecting initiator data
    CS_PROCESS_STATE_REMOTE_DATA_READY          =   0x04, //!< Finished collecting reflector data
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

    /* Sessions data */
    uint16_t currSession;                                                       //!< The current procedure session handle as passed by @ref CSProcess_InitProcedure
    CSProcessSessionData sessionsDb[CS_PROCESS_MAX_SESSIONS];                   //!< Sessions Database

    /* BleCsRanging module related */
    BleCsRanging_CsSubevent_t *pSubeventNodeInProgress;                         //!< In-progress subevent node
    uint32_t inProgressSubeventNodeTotalSize;                                   //!< In-progress subevent node total allocated size
    uint32_t inProgressSubeventStepOffset;                                      //!< Absolute byte offset for next step write (initialized to header size)
    List_List localSubeventList;                                                //!< Linked list of completed local subevent nodes
    List_List remoteSubeventList;                                               //!< Linked list of completed remote subevent nodes
    BleCsRanging_CsConfig_t csConfig;                                           //!< CS procedure config from @ref HCI_LE_CS_CONFIG_COMPLETE
    BleCsRanging_Config_t config;                                               //!< Algorithm configuration passed to BleCsRanging_estimatePbr

    /* BTCS related */
    // TODO: deprecated
    // uint8_t stepsIdxToAntPermutationMap[CS_PROCESS_MAX_STEPS];               //!< Mapping collected steps indices to antenna permutation (not indices)
} CSProcessDb_t;

//*****************************************************************************
//! Prototypes
//*****************************************************************************

/********** Helper functions **********/
uint8_t csProcess_GetStepLengthBTCS(uint8_t mode, uint8_t role);
csProcessStatus_e csProcess_CheckSessionHandle(uint16_t handle);

/********** Internal DB managing functions **********/
void csProcess_ClearProcedureData(void);
void csProcess_SubeventPostProcess(ChannelSounding_resultsSourceMode_e resultsSourceMode, uint8_t subeventDoneStatus, uint8_t procedureDoneStatus);

/********** Subevent node helpers **********/
csProcessStatus_e csProcess_AllocSubeventNode(CSProcess_AddSubeventResultsParams_t *pParams);
void csProcess_LinkSubeventNode(ChannelSounding_resultsSourceMode_e resultsSourceMode);
csProcessStatus_e csProcess_ProcessModeZeroStep(uint8_t*stepData, uint8_t role, uint8_t stepChannel);
csProcessStatus_e csProcess_ProcessModeTwoStep(CS_modeTwoStep_t *stepData, uint8_t role, uint8_t stepChannel);

/********** Subevents processing functions **********/
csProcessStatus_e csProcess_HandleSubeventResults(CSProcess_AddSubeventResultsParams_t *pParams);
csProcessStatus_e csProcess_ProcessSubeventResultsSteps(csSubeventResultsStep_t *subeventResultsSteps,
                                                        uint8_t numStepsReported, uint8_t role, uint32_t *totalBytesProcessed);
#ifdef RANGING_CLIENT
csProcessStatus_e csProcess_ProcessSubeventResultsStepsRAS(Ranging_subEventResultsStep_t *subeventResultsSteps,
                                                           uint8_t numStepsReported, uint8_t role, uint32_t *totalBytesProcessed);
#endif

csProcessStatus_e csProcess_ProcessSubeventResultsStepsBTCS(uint8_t *subeventResultsSteps,
                                                            uint8_t numStepsReported, uint8_t role, uint32_t *totalBytesProcessed);
uint8_t csProcess_GetSubeventCounter(ChannelSounding_resultsSourceMode_e mode);
bool csProcess_IsAllowedToIncrementSubeventCounter(ChannelSounding_resultsSourceMode_e mode);
void csProcess_IncrementSubeventCounter(ChannelSounding_resultsSourceMode_e mode);

/********** Steps processing functions **********/
csProcessStatus_e csProcess_ProcessStep(uint8_t mode, uint8_t *stepData, uint8_t role, uint8_t stepChannel);
//TODO: deprecated
// csProcessStatus_e csProcess_ProcessStepBTCS(uint8_t mode, uint8_t *stepData, uint8_t role, uint8_t stepChannel);
// csProcessStatus_e csProcess_ProcessModeTwoStepBTCS(BTCS_modeTwoStep_t* stepDataToAdd, uint8_t role, uint8_t stepChannel);

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

    // Check parameters
    if ((NULL == pParams) ||
        (pParams->numAntPath < CS_PROCESS_MIN_ANT_PATHS || pParams->numAntPath > CS_PROCESS_MAX_ANT_PATHS) ||
        (pParams->localRole > CS_ROLE_REFLECTOR) || (pParams->mode0Steps > 3))
    {
        status = CS_PROCESS_INVALID_PARAM;
    }

    // Validate that multi-antenna procedures have valid Tsw
    if (status == CS_PROCESS_SUCCESS)
    {
        // If numAntPath requires more than 1 antenna path, validate that tsw is above 0
        if (pParams->numAntPath > 1 && pParams->tSW == CS_T_0US)
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
        // numAntPath already computed by caller from ACI
        gCsProcessDb.config.numAntPath = pParams->numAntPath;

        // Set csConfig params
        gCsProcessDb.csConfig.tIP1                       = pParams->tIP1;
        gCsProcessDb.csConfig.tIP2                       = pParams->tIP2;
        gCsProcessDb.csConfig.tPM                        = pParams->tPM;
        gCsProcessDb.csConfig.tSW                        = pParams->tSW;
        gCsProcessDb.csConfig.csSync                     = pParams->csSync;
        gCsProcessDb.csConfig.rttType                    = pParams->rttType;
        gCsProcessDb.csConfig.tFCS                       = pParams->tFCS;
        gCsProcessDb.csConfig.mode0Steps                 = pParams->mode0Steps;
        gCsProcessDb.csConfig.mainModeType               = pParams->mainModeType;
        gCsProcessDb.csConfig.mainModeRepetition         = pParams->mainModeRepetition;
        gCsProcessDb.csConfig.toneAntennaConfigSelection = pParams->toneAntennaConfigSelection;

        // Save the local role and tx power for the procedure
        gCsProcessDb.localRole = pParams->localRole;

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
    else if (pParams->resultsSourceMode != CS_RESULTS_MODE_LOCAL &&
             (gCsProcessDb.state & CS_PROCESS_STATE_LOCAL_DATA_READY) != CS_PROCESS_STATE_LOCAL_DATA_READY)
    {
        status = CS_PROCESS_LOCAL_RESULTS_NOT_READY;
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
 * @return      Length of the relevant step data
 * @return      0 for one of the following cases:
 *              - Invalid mode has been given.
 *              - mode is 0 and role is invalid.
 */
// TODO: uncomment to support BTCS
// uint8_t csProcess_GetStepLengthBTCS(uint8_t mode, uint8_t role)
// {
//     uint8_t stepDataLen = 0;

//     switch(mode)
//     {
//       case CS_MODE_0:
//       {
//         if (role == CS_ROLE_INITIATOR)
//         {
//             stepDataLen = sizeof(BTCS_modeZeroInitStep_t);
//         }
//         else if(role == CS_ROLE_REFLECTOR)
//         {
//             stepDataLen = sizeof(BTCS_modeZeroReflStep_t);
//         }
//         else
//         {
//             // Consider failure - leave stepDataLen as 0
//         }
//         break;
//       }
//       case CS_MODE_1:
//       {
//         stepDataLen = sizeof(BTCS_modeOneStep_t);
//         break;
//       }
//       case CS_MODE_2:
//       {
//         stepDataLen = sizeof(BTCS_modeTwoStep_t) + gCsProcessDb.config.numAntPath * 3; // 3 is the size of @ref BTCS_modeTwoStepData_t
//         break;
//       }
//       case CS_MODE_3:
//       {
//         stepDataLen = sizeof(BTCS_modeThreeStep_t) + gCsProcessDb.config.numAntPath * 3; // 3 is the size of @ref BTCS_modeTwoStepData_t
//         break;
//       }
//       default:
//       {
//         break;
//       }
//     }

//     return stepDataLen;
// }

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
 * @fn          csProcess_AllocSubeventNode
 *
 * @brief       Dynamically allocates a new subevent node (header + step area) for
 *              the given role and sets in-progress globals.
 *              Called once per subevent on the initial (non-continuation) event.
 *              Data processing will be deferred.
 *
 * @param       pParams - Pointer to subevent results parameters containing all subevent info
 *
 * @return      CS_PROCESS_SUCCESS on success.
 * @return      CS_PROCESS_GENERAL_FAILURE if ICall_malloc fails.
 */
csProcessStatus_e csProcess_AllocSubeventNode(CSProcess_AddSubeventResultsParams_t *pParams)
{
    // Find the size and allocate the subevent node
    if (pParams == NULL || pParams->totalSubeventStepsCount < gCsProcessDb.csConfig.mode0Steps)
    {
        return CS_PROCESS_INVALID_PARAM;
    }

    uint32_t nodeSize = BLECSRANGING_SUBEVENT_ALLOC_MODE0_MODE2(gCsProcessDb.csConfig.mode0Steps,
                                                                pParams->totalSubeventStepsCount - gCsProcessDb.csConfig.mode0Steps,
                                                                pParams->numAntennaPath);
    BleCsRanging_CsSubevent_t *pNode = ICall_malloc(nodeSize);
    if (pNode == NULL)
    {
        return CS_PROCESS_GENERAL_FAILURE;
    }

    // Initialise header fields
    memset(pNode, 0, sizeof(BleCsRanging_CsSubevent_t));
    pNode->frequencyCompensation = pParams->frequencyCompensation;
    pNode->referencePowerLevel   = pParams->referencePowerLevel;
    pNode->numAntennaPaths       = pParams->numAntennaPath;

    // Assign the allocated memory to the in-progress global
    gCsProcessDb.pSubeventNodeInProgress = pNode;
    gCsProcessDb.inProgressSubeventNodeTotalSize = nodeSize;
    gCsProcessDb.inProgressSubeventStepOffset = sizeof(BleCsRanging_CsSubevent_t);

    return CS_PROCESS_SUCCESS;
}

/*******************************************************************************
 * @fn          csProcess_LinkSubeventNode
 *
 * @brief       Links the completed subevent block into the corresponding
 *              local or remote List_List and clears the in-progress variables.
 *
 * @param       resultsSourceMode - Identifier if the subevent node should be added
 *                                  to local or remote list.
 *
 * @return      None
 */
void csProcess_LinkSubeventNode(ChannelSounding_resultsSourceMode_e resultsSourceMode)
{
    // Link the completed node to the appropriate list
    if (gCsProcessDb.pSubeventNodeInProgress != NULL)
    {
        if(resultsSourceMode == CS_RESULTS_MODE_LOCAL)
        {
            List_put(&gCsProcessDb.localSubeventList, (List_Elem*) gCsProcessDb.pSubeventNodeInProgress);
        }
        else
        {
            List_put(&gCsProcessDb.remoteSubeventList, (List_Elem*) gCsProcessDb.pSubeventNodeInProgress);
        }

        // Reset the in-progress globals
        gCsProcessDb.pSubeventNodeInProgress = NULL;
        gCsProcessDb.inProgressSubeventStepOffset = 0;
        gCsProcessDb.inProgressSubeventNodeTotalSize = 0;

        // Increment the subevent counter if needed and allowed to
        if (csProcess_IsAllowedToIncrementSubeventCounter(resultsSourceMode))
        {
            // Save current subevent counter and increment it
            csProcess_IncrementSubeventCounter(resultsSourceMode);
        }
    }
}

/*******************************************************************************
 * @fn          csProcess_FreeSubeventList
 *
 * @brief       Free and drain a subevent list. Dequeues and frees every committed node in pList.
 *              Helper function for @ref csProcess_ClearProcedureData.
 *
 * @param       pList        - Pointer to the List_List to drain
 *
 * @return      None
 */
static void csProcess_FreeSubeventList(List_List *pList)
{
    BleCsRanging_CsSubevent_t *pNode = (BleCsRanging_CsSubevent_t *)List_get(pList);
    while (pNode != NULL)
    {
        ICall_free(pNode);
        pNode = (BleCsRanging_CsSubevent_t *)List_get(pList);
    }
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

    // Reset current procedure session
    gCsProcessDb.currSession = CS_PROCESS_INVALID_SESSION;

    // Reset subevent bookkeeping
    gCsProcessDb.subeventCounterLocal = 0;
    gCsProcessDb.subeventCounterRemote = 0;
    gCsProcessDb.inProgressSubeventNodeTotalSize = 0;
    gCsProcessDb.inProgressSubeventStepOffset = 0;

    // Free the in-progress subevent node
    if(gCsProcessDb.pSubeventNodeInProgress != NULL)
    {
        ICall_free(gCsProcessDb.pSubeventNodeInProgress);
        gCsProcessDb.pSubeventNodeInProgress = NULL;
    }

    // Reset the subevent lists
    csProcess_FreeSubeventList(&gCsProcessDb.localSubeventList);
    csProcess_FreeSubeventList(&gCsProcessDb.remoteSubeventList);

    // Reset CCC related data - BTCS
    // TODO: deprecated
    // memset(gCsProcessDb.stepsIdxToAntPermutationMap, 0, sizeof(gCsProcessDb.stepsIdxToAntPermutationMap));
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
 * @return      CS_PROCESS_TOO_MANY_SUBEVENTS_PROVIDED - Too many subevents provided.
 */
csProcessStatus_e csProcess_HandleSubeventResults(CSProcess_AddSubeventResultsParams_t *pParams)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;
    uint8_t role;           // The role of the device that reported about the subevent

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

    // If all checks passed, proceed with processing the subevent steps
    // Validate that controller-reported numAntennaPath matches configured value
    if (status == CS_PROCESS_SUCCESS)
    {
        if (pParams->numAntennaPath != gCsProcessDb.config.numAntPath)
        {
            // Mismatch between configured ACI and controller-reported antenna paths!
            // This indicates controller bug, configuration error, or hardware issue
            status = CS_PROCESS_INVALID_PARAM;
        }
    }

    // Extract role from source mode
    role = pParams->resultsSourceMode == CS_RESULTS_MODE_LOCAL ? gCsProcessDb.localRole : CS_GET_OPPOSITE_ROLE(gCsProcessDb.localRole);

    if (status == CS_PROCESS_SUCCESS)
    {
        // The first subevent event will allocate a new node for the subevent
        if (gCsProcessDb.pSubeventNodeInProgress == NULL)
        {
            status = csProcess_AllocSubeventNode(pParams);
        }
    }

    if (status == CS_PROCESS_SUCCESS)
    {
        // Add results to the Ranging module
        // Note: Using gCsProcessDb.config.numAntPath (from ACI) instead of pParams->numAntennaPath
        // after validation ensures single source of truth
        switch (pParams->resultsSourceMode)
        {
            case CS_RESULTS_MODE_LOCAL:
            {
                // Process local data
                status = csProcess_ProcessSubeventResultsSteps((csSubeventResultsStep_t*)pParams->data,
                                                                pParams->numStepsReported, role, pParams->totalBytesProcessed);
                break;
            }
            case CS_RESULTS_MODE_PROP:
            {
                // Process remote data from TI L2CAPCOC protocol
                status = csProcess_ProcessSubeventResultsSteps((csSubeventResultsStep_t*)pParams->data,
                                                                pParams->numStepsReported, role, pParams->totalBytesProcessed);
                break;
            }
            case CS_RESULTS_MODE_RAS:
            {
#ifdef RANGING_CLIENT
                // Process remote data provided by RAS profile
                status = csProcess_ProcessSubeventResultsStepsRAS((Ranging_subEventResultsStep_t*)pParams->data,
                                                                    pParams->numStepsReported, role, pParams->totalBytesProcessed);
#else
                status = CS_PROCESS_RESULTS_MODE_NOT_SUPPORTED;
#endif
                break;
            }
            case CS_RESULTS_MODE_BTCS:
            {
                // TODO: deprecated, will return MODE_NOT_SUPPORTED
                // // Process remote data provided by BTCS
                // status = csProcess_ProcessSubeventResultsStepsBTCS(pParams->data,
                //                                                     pParams->numStepsReported, role, pParams->totalBytesProcessed);
                // break;
            }
            default:
            {
                status = CS_PROCESS_RESULTS_MODE_NOT_SUPPORTED;
                break;
            }
        }
    }

    // Check if distance is ready to be calculated and report if it is
    if (status == CS_PROCESS_SUCCESS)
    {
        // Post-process: link the completed subevent, update state flags, increment subevent counter
        csProcess_SubeventPostProcess(pParams->resultsSourceMode, pParams->subeventDoneStatus, pParams->procedureDoneStatus);
        status = csProcess_CheckIfDone();
    }
    else
    {
        // In case of any failure - clear all of the data
        // TODO: Can be removed if handled outside of this module,
        //       In case of FAILURE it is required to ICall_free @ref gCsProcessDb.pSubeventNodeInProgress
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
csProcessStatus_e csProcess_ProcessSubeventResultsSteps(csSubeventResultsStep_t *subeventResultsSteps,
                                                        uint8_t numStepsReported, uint8_t role, uint32_t *totalBytesProcessed)
{
  csProcessStatus_e status = CS_PROCESS_SUCCESS;
  uint8_t* tempResults = (uint8_t *)subeventResultsSteps;
  csSubeventResultsStep_t *srcStep;
  uint8_t expectedStepDataLen = 0;
  uint32_t bytesProcessed = 0;

  for (int i = 0; i < numStepsReported; i++)
  {
    // If any failure occurred during the process - break the loop
    if (status != CS_PROCESS_SUCCESS)
    {
        break;
    }

    // Determine the current step we are working on
    srcStep = (csSubeventResultsStep_t *)tempResults;

    // Get the expected data length for this mode
    expectedStepDataLen = CS_GetStepLength(srcStep->stepMode, role, gCsProcessDb.config.numAntPath);

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
        status = csProcess_ProcessStep(srcStep->stepMode, (uint8_t*) &(srcStep->stepData), role, srcStep->stepChnl);
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
 * @fn    csProcess_GetStepChannelRAS
 *
 * @brief Return the channel related to the current step.
 *        Called by @ref csProcess_ProcessSubeventResultsStepsRAS to retrieve the channel
 *        related to the current step from @ref gCsProcessDb.localSubeventList.
 *
 * @param mode                - Step mode that the channel is required for.
 * @param pSubeventStepOffset - Pointer to an incremental offset inside the subevent node,
 *                              initiated in @ref csProcess_ProcessSubeventResultsStepsRAS,
 *                              incremented here.
 * @param pSubeventNode       - Pointer to the local node related to this remote node.
 *
 * @return Channel for the current step processed in RAS.
 * @return CS_PROCESS_INVALID_CHANNEL if mode is not supported.
 */
static uint8_t csProcess_GetStepChannelRAS(uint8_t mode, uint32_t* pSubeventStepOffset, List_Elem *pSubeventNode)
{
    uint8_t* pStep = NULL;
    uint8_t channel = CS_PROCESS_INVALID_CHANNEL;

    // Point to step header
    pStep = ((uint8_t*)pSubeventNode + sizeof(BleCsRanging_CsSubevent_t) + *pSubeventStepOffset);

    // Parse the struct based on channel stepMode
    switch (mode)
    {
      case CS_MODE_0:
      {
        channel = ((BleCsRanging_StepMode0_t*)pStep)->stepChannel;
        *pSubeventStepOffset += BLECSRANGING_STEP_MODE0_SIZE;
        break;
      }
      case CS_MODE_2:
      // MODE 3 is processed as MODE 2 step @ref csProcess_ProcessStep
      case CS_MODE_3:
      {
        channel = ((BleCsRanging_StepMode2_t*)pStep)->stepChannel;
        *pSubeventStepOffset += BLECSRANGING_STEP_MODE2_SIZE(((BleCsRanging_CsSubevent_t*)pSubeventNode)->numAntennaPaths);
        break;
      }
      case CS_MODE_1:
      default:
      {
        break;
      }
    }

    return channel;
}

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
 * @return CS_PROCESS_LOCAL_RESULTS_NOT_READY - It is required that local results will be processed before remote results.
 */
csProcessStatus_e csProcess_ProcessSubeventResultsStepsRAS(Ranging_subEventResultsStep_t *subeventResultsSteps,
                                                           uint8_t numStepsReported, uint8_t role, uint32_t *totalBytesProcessed)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;
    uint8_t* tempResults = (uint8_t *)subeventResultsSteps;
    Ranging_subEventResultsStep_t *srcStep;
    uint8_t expectedStepDataLen = 0;
    uint32_t bytesProcessed = 0;
    uint32_t subeventStepOffset = 0;
    List_Elem *pLocalNode = List_head(&gCsProcessDb.localSubeventList);
    uint8_t subeventCounter = csProcess_GetSubeventCounter(CS_RESULTS_MODE_RAS);

    // Find the local subevent node related to the remote node (incremented prior to this function call)
    for (uint8_t i = 0; i < subeventCounter; i++)
    {
        if(pLocalNode == NULL)
        {
            status = CS_PROCESS_LOCAL_RESULTS_NOT_READY;
            break;
        }
        else
        {
            pLocalNode = List_next(pLocalNode);
        }
    }

    if (status == CS_PROCESS_SUCCESS &&
        numStepsReported != ((BleCsRanging_CsSubevent_t *)pLocalNode)->numSteps)
    {
        status = CS_PROCESS_INVALID_PARAM;
    }

    for (int i = 0; i < numStepsReported; i++)
    {
      // If any failure occurred during the process - break the loop
      if (status != CS_PROCESS_SUCCESS)
      {
          break;
      }

      // Determine the current step we are working on
      srcStep = (Ranging_subEventResultsStep_t *)tempResults;

      // Calculate the expected step data length
      expectedStepDataLen = CS_GetStepLength(srcStep->stepMode, role, gCsProcessDb.config.numAntPath);

      // Check that the previous function returned a valid length.
      // Ensure step data length equals to the expected one
      if (expectedStepDataLen <= 0)
      {
          status = CS_PROCESS_STEPS_PROCESSING_FAILED;
      }

      // Process the step data depending on which role it is
      if (status == CS_PROCESS_SUCCESS)
      {
          uint8_t stepChannel = csProcess_GetStepChannelRAS(srcStep->stepMode, &subeventStepOffset, pLocalNode);
          status = csProcess_ProcessStep(srcStep->stepMode, (uint8_t *)&(srcStep->stepData), role, stepChannel);
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
// TODO: uncomment to support BTCS
// csProcessStatus_e csProcess_ProcessSubeventResultsStepsBTCS(uint8_t *subeventResultsSteps,
//                                                             uint8_t numStepsReported, uint8_t role, uint32_t *totalBytesProcessed)
// {
//     csProcessStatus_e status = CS_PROCESS_SUCCESS;
//     uint8_t* modeInfoIter = subeventResultsSteps;
//     uint8_t* stepsIter = modeInfoIter +  CS_PROCESS_CEIL_DIVIDE_2(numStepsReported);
//     uint32_t bytesProcessed = 0;

//     for (int i = 0; i < numStepsReported; i++)
//     {
//       BTCS_stepMode_t currStepMode;

//       // If any failure occurred during the process - break the loop
//       if (status != CS_PROCESS_SUCCESS)
//       {
//           break;
//       }

//       // Determine the current step we are working on
//       if (i % 2 == 0)
//       {
//           currStepMode.mode = BTCS_EXTRACT_MODE_FIRST(*modeInfoIter);
//           currStepMode.status = BTCS_EXTRACT_STATUS_FIRST(*modeInfoIter);
//       }
//       else
//       {
//           currStepMode.mode = BTCS_EXTRACT_MODE_SECOND(*modeInfoIter);
//           currStepMode.status = BTCS_EXTRACT_STATUS_SECOND(*modeInfoIter);
//       }

//       // Process the step data depending on which role it is
//       if (status == CS_PROCESS_SUCCESS && currStepMode.status == 0)
//       {
// TODO: deprecated
//           uint8_t stepChannel = gCsProcessDb.stepsIdxToChnlMap[gCsProcessDb.currentStepRemote];
//           status = csProcess_ProcessStepBTCS(currStepMode.mode, stepsIter, role, stepChannel);
//       }

//       if (status == CS_PROCESS_SUCCESS)
//       {
//           // Calculate step length
//           uint8_t stepLength = csProcess_GetStepLengthBTCS(currStepMode.mode, role);

//           // Increment steps iterator
//           stepsIter += stepLength;
//           bytesProcessed += stepLength;

//           // Increment byteProcessed by 1 for step mode processing.
//           // Do it only for even steps indices because each byte of the stepmodes data includes 2 steps (4 bits for a step)
//           if (i % 2 == 0)
//           {
//               bytesProcessed += 1;
//           }

//           // Increment mode iterator only when the loop iterator is odd
//           if (i % 2 == 1)
//           {
//               modeInfoIter += 1;
//           }
//       }
//     }

//     if (status == CS_PROCESS_SUCCESS && (numStepsReported % 2 == 1))
//     {
//         // If the number of reported steps is odd - we didn't process the last mode_step,
//         // therefore we need to compensate for it.
//         bytesProcessed += 1;
//     }

//     // Output the total bytes processed
//     if (totalBytesProcessed != NULL)
//     {
//         *totalBytesProcessed = bytesProcessed;
//     }

//     return status;
// }

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

/**
 * @fn    csProcess_ProcessModeZeroStep
 *
 * @brief Processes a Mode 0 step and adds it to the appropriate subevent node.
 *
 * This function extracts data from the Mode 0 step, populates a BleCsRanging_StepMode0_t
 * struct, and writes it to the current subevent node's step data area.
 *
 * @param stepData       - Pointer to the raw Mode 0 step data. @ref CS_modeZeroInitStep_t or CS_modeZeroRefStep_t.
 * @param role           - The role of the device. @ref CS_ROLE_INITIATOR or CS_ROLE_REFLECTOR.
 * @param stepChannel    - The channel associated with the step.
 *
 * @return CS_PROCESS_SUCCESS if the step was successfully processed and added.
 * @return CS_PROCESS_INVALID_STEP_PARAM if parameters are invalid.
 */
csProcessStatus_e csProcess_ProcessModeZeroStep(uint8_t* stepData, uint8_t role, uint8_t stepChannel)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;

    // Validate parameters
    if ((stepData == NULL) || (role > CS_ROLE_REFLECTOR) || (gCsProcessDb.pSubeventNodeInProgress == NULL) ||
        (stepChannel < 2 || stepChannel > 76 || (stepChannel >= 23 && stepChannel <= 25)))
    {
        status = CS_PROCESS_INVALID_STEP_PARAM;
    }

    if (status == CS_PROCESS_SUCCESS)
    {
        // Calculate the pointer to the step data area within the subevent node
        BleCsRanging_StepMode0_t *pMode0Step =
            (BleCsRanging_StepMode0_t *) ((uint8_t*)gCsProcessDb.pSubeventNodeInProgress + gCsProcessDb.inProgressSubeventStepOffset);

        // Extract generic step data
        pMode0Step->stepMode = CS_MODE_0;
        pMode0Step->stepChannel = stepChannel;
        pMode0Step->pad = 0;

        // Extract role based data
        if(role == CS_ROLE_INITIATOR)
        {
            pMode0Step->packetQuality = ((CS_modeZeroInitStep_t *)stepData)->packetQuality;
            pMode0Step->packetRssi = ((CS_modeZeroInitStep_t *)stepData)->packetRssi;
            pMode0Step->packetAntenna = ((CS_modeZeroInitStep_t *)stepData)->packetAntenna;
            pMode0Step->measuredFreqOffset = ((CS_modeZeroInitStep_t *)stepData)->measuredFreqOffset; // Initiator: measured.
        }
        else
        {
            pMode0Step->packetQuality = ((CS_modeZeroReflStep_t *)stepData)->packetQuality;
            pMode0Step->packetRssi = ((CS_modeZeroReflStep_t *)stepData)->packetRssi;
            pMode0Step->packetAntenna = ((CS_modeZeroReflStep_t *)stepData)->packetAntenna;
            pMode0Step->measuredFreqOffset = 0; // Reflector: 0.
        }

        // Update the byte offset for the next step
        gCsProcessDb.inProgressSubeventStepOffset += BLECSRANGING_STEP_MODE0_SIZE;

        // Increment the steps counter
        gCsProcessDb.pSubeventNodeInProgress->numSteps++;
    }

    return status;
}

/**
 * @fn    csProcess_ProcessModeTwoStep
 *
 * @brief Processes a Mode 2 step and adds it to the appropriate subevent node.
 *
 * This function validates the Mode 2 step data, populates a BleCsRanging_StepMode2_t
 * struct with tone data, and writes it to the current subevent node's step data area.
 *
 * @param stepData       - Pointer to the Mode 2 step data.
 * @param role           - The role of the device (CS_ROLE_INITIATOR or CS_ROLE_REFLECTOR).
 * @param stepChannel    - The channel associated with the step (2-76, excluding 23-25).
 *
 * @return CS_PROCESS_SUCCESS if the step was successfully processed and added.
 * @return CS_PROCESS_INVALID_STEP_PARAM if parameters are invalid.
 */
csProcessStatus_e csProcess_ProcessModeTwoStep(CS_modeTwoStep_t *stepData, uint8_t role, uint8_t stepChannel)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;

    // Validate parameters
    if ((gCsProcessDb.currSession == CS_PROCESS_INVALID_SESSION) ||
        (stepData == NULL) || (role > CS_ROLE_REFLECTOR) ||
        (stepChannel < 2 || stepChannel > 76 || (stepChannel >= 23 && stepChannel <= 25)) ||
        (stepData->antennaPermutationIndex > CS_RANGING_MAX_PERMUTATION_INDEX) ||
        (gCsProcessDb.config.numAntPath == 1 && stepData->antennaPermutationIndex > CS_MAX_PERMUTATION_INDEX_1_ANT) ||
        (gCsProcessDb.config.numAntPath == 2 && stepData->antennaPermutationIndex > CS_MAX_PERMUTATION_INDEX_2_ANT) ||
        (gCsProcessDb.config.numAntPath == 3 && stepData->antennaPermutationIndex > CS_MAX_PERMUTATION_INDEX_3_ANT) ||
        (gCsProcessDb.config.numAntPath == 4 && stepData->antennaPermutationIndex > CS_MAX_PERMUTATION_INDEX_4_ANT))
    {
        status = CS_PROCESS_INVALID_STEP_PARAM;
    }

    if (status == CS_PROCESS_SUCCESS)
    {
        // Calculate the pointer to the step data area within the subevent node
        BleCsRanging_StepMode2_t *pMode2Step =
            (BleCsRanging_StepMode2_t *) ((uint8_t*)gCsProcessDb.pSubeventNodeInProgress + gCsProcessDb.inProgressSubeventStepOffset);

        pMode2Step->stepMode = CS_MODE_2;
        pMode2Step->stepChannel = stepChannel;
        pMode2Step->antennaPermutationIndex = stepData->antennaPermutationIndex;
        pMode2Step->pad = 0;

        // Copy tone data for each antenna path
        for (uint8_t i = 0; i < gCsProcessDb.pSubeventNodeInProgress->numAntennaPaths; i++)
        {
            pMode2Step->tonePCT[i].i = stepData->data[i].i;
            pMode2Step->tonePCT[i].q = stepData->data[i].q;
            pMode2Step->tonePCT[i].quality = stepData->data[i].tqi;
        }

        // Update the byte offset for the next step
        gCsProcessDb.inProgressSubeventStepOffset += BLECSRANGING_STEP_MODE2_SIZE(gCsProcessDb.pSubeventNodeInProgress->numAntennaPaths);

        // Increment the step counter
        gCsProcessDb.pSubeventNodeInProgress->numSteps++;
    }

    return status;
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
 *
 * @return CS_PROCESS_SUCCESS The step has been successfully processed.
 * @return CS_PROCESS_MODE_NOT_SUPPORTED The given mode parameter is not supported or not recognized.
 * @return CS_PROCESS_INVALID_STEP_PARAM - One of the other parameters is invalid.
 */
csProcessStatus_e csProcess_ProcessStep(uint8_t mode, uint8_t *stepData, uint8_t role, uint8_t stepChannel)
{
    csProcessStatus_e status = CS_PROCESS_SUCCESS;

    if( gCsProcessDb.inProgressSubeventNodeTotalSize <= gCsProcessDb.inProgressSubeventStepOffset )
    {
        status = CS_PROCESS_TOO_MANY_STEPS_PROVIDED;
    }

    switch(mode)
    {
      case CS_MODE_0:
      {
        // Add step to the subevent node
        status = csProcess_ProcessModeZeroStep(stepData, role, stepChannel);
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
        // Add step to the subevent node
        status = csProcess_ProcessModeTwoStep(modeTwoStep, role, stepChannel);
        break;
      }
      case CS_MODE_3:
      {
        // Cast step data to Mode-3 step data
        CS_modeThreeStep_t *modeThreeStep = (CS_modeThreeStep_t *)stepData;
        // Add step to the subevent node
        status = csProcess_ProcessModeTwoStep((CS_modeTwoStep_t *)&modeThreeStep->antennaPermutationIndex, role, stepChannel);
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
 *
 * @return CS_PROCESS_SUCCESS The step has been successfully processed.
 * @return CS_PROCESS_MODE_NOT_SUPPORTED The given mode parameter is not supported or not recognized.
 * @return CS_PROCESS_INVALID_STEP_PARAM One of the step parameters is invalid.
 */
// TODO: uncomment to support BTCS
// csProcessStatus_e csProcess_ProcessStepBTCS(uint8_t mode, uint8_t *stepData, uint8_t role, uint8_t stepChannel)
// {
//     csProcessStatus_e status = CS_PROCESS_SUCCESS;

//     switch(mode)
//     {
//       case CS_MODE_0:
//       {
//           csProcess_ProcessModeZeroStep(stepData, role);
//         break;
//       }
//       case CS_MODE_1:
//       {
//         // Mode not supported
//         status = CS_PROCESS_MODE_NOT_SUPPORTED;
//         break;
//       }
//       case CS_MODE_2:
//       {
//         // Cast step data to Mode-2 step data
//         BTCS_modeTwoStep_t *modeTwoStep = (BTCS_modeTwoStep_t *)stepData;

//         // Add step results to the Ranging module
//         status = csProcess_ProcessModeTwoStepBTCS(modeTwoStep, role, stepChannel);
//         break;
//       }
//       case CS_MODE_3:
//       {
//         // Cast step data to Mode-3 step data
//         BTCS_modeThreeStep_t *modeThreeStep = (BTCS_modeThreeStep_t *)stepData;

//         // Add step results to the Ranging module
//         status = csProcess_ProcessModeTwoStepBTCS(&(modeThreeStep->data), role, stepChannel);
//         break;
//       }
//       default:
//       {
//         status = CS_PROCESS_MODE_NOT_SUPPORTED;
//         break;
//       }
//     }

//     return status;
// }

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
 *
 * @return CS_PROCESS_SUCCESS if data is added successfully.
 * @return CS_PROCESS_INVALID_STEP_PARAM one of the step parameters is invalid.
 */
// TODO: uncomment to support BTCS
// csProcessStatus_e csProcess_ProcessModeTwoStepBTCS(BTCS_modeTwoStep_t* stepDataToAdd, uint8_t role, uint8_t stepChannel)
// {
//     csProcessStatus_e status = CS_PROCESS_SUCCESS;

//     // Check Parameters
//     if ((gCsProcessDb.currSession == CS_PROCESS_INVALID_SESSION) ||
//         (stepDataToAdd == NULL) || (role > CS_ROLE_REFLECTOR) ||
//         (stepChannel < 2 || stepChannel > 76 ||  (stepChannel >= 23 && stepChannel <= 25)) || // Channels which are not in use
//         (gCsProcessDb.config.numAntPath > 4))
//     {
//         status = CS_PROCESS_INVALID_STEP_PARAM;
//     }
//     else
//     {
//         // correct the channel index to match the DB array indices
//         uint8_t channelIndex = stepChannel - 2;

//         // Get the permutation from the DB
// TODO: deprecated
//         uint8_t permutation = gCsProcessDb.stepsIdxToAntPermutationMap[gCsProcessDb.currentStepRemote];

//         for (uint8_t j = 0; j < gCsProcessDb.config.numAntPath; j++)
//         {
//             BleCsRanging_Tone_t data;

//             uint8_t pathIndex = permutation & 0x3;

//             uint16_t pathOffset = (pathIndex * CS_RANGING_PCT_ARRAY_SIZE);

//             // Copy the given step results
//             BTCS_modeTwoStepData_t* stepIQData = (BTCS_modeTwoStepData_t*) (stepDataToAdd->IQData + (j * 3));
//             data.i = stepIQData->i;
//             data.q = stepIQData->q;

//             // Quality is ordered by MSB - (A4, A3, A2, A1) - LSB.
//             // Therefore take the quality bits using the pathIndex
//             data.quality = ((stepDataToAdd->quality) >> pathIndex) & 0x3;

//             // Move the permutation by two bits in order to get the next path for the next loop
//             permutation = permutation >> 2;
//         }
//     }

//     return status;
// }

/**
 * @fn csProcess_SubeventPostProcess
 *
 * @brief Post processing of subevent after some or all steps has been added
 *        for a specific role.
 *        This function Validates Subevent Done Status, Procedure Done Status.
 *
 * @param resultsSourceMode     -  Source of the provided results (local/remote).
 * @param subeventDoneStatus    -  Subevent Done Status reported parameter.
 * @param procedureDoneStatus   -  Procedure Done Status reported parameter.
 *
 * @return None.
 */
void csProcess_SubeventPostProcess(ChannelSounding_resultsSourceMode_e resultsSourceMode, uint8_t subeventDoneStatus, uint8_t procedureDoneStatus)
{
    if (subeventDoneStatus == CS_SUBEVENT_DONE)
    {
        // Link the completed subevent node into the list only when the subevent is fully done
        csProcess_LinkSubeventNode(resultsSourceMode);

        // All results has been processed - mark data as ready for the specific role.
        if(procedureDoneStatus == CS_PROCEDURE_DONE)
        {
            if (resultsSourceMode == CS_RESULTS_MODE_LOCAL)
            {
                gCsProcessDb.state |= (uint8_t) CS_PROCESS_STATE_LOCAL_DATA_READY;
            }
            else
            {
                gCsProcessDb.state |= (uint8_t) CS_PROCESS_STATE_REMOTE_DATA_READY;
            }
        }
    }
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
        ((gCsProcessDb.state & CS_PROCESS_STATE_LOCAL_DATA_READY) == CS_PROCESS_STATE_LOCAL_DATA_READY ) &&
        ((gCsProcessDb.state & CS_PROCESS_STATE_REMOTE_DATA_READY) == CS_PROCESS_STATE_REMOTE_DATA_READY))
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

volatile uint32_t _vel_count = 0;
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
    List_List* initiatorSubevents = NULL;               // List to hold the initiator subevents
    List_List* reflectorSubevents = NULL;               // List to hold the reflector subevents

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

        if (gCsProcessDb.localRole == CS_ROLE_INITIATOR)
        {
            initiatorSubevents = &gCsProcessDb.localSubeventList;
            reflectorSubevents = &gCsProcessDb.remoteSubeventList;
        }
        else
        {
            initiatorSubevents = &gCsProcessDb.remoteSubeventList;
            reflectorSubevents = &gCsProcessDb.localSubeventList;
        }

        BleCsRangingStatus = BleCsRanging_estimatePbr(initiatorSubevents,
                                                      reflectorSubevents,
                                                      &gCsProcessDb.csConfig,
                                                      &gCsProcessDb.config,
                                                      &results);

        if (BleCsRangingStatus != BleCsRanging_Status_Success)
        {
            status = CS_PROCESS_DISTANCE_ESTIMATION_FAILED;
        }
        else
        {
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
            csResults->velocity = (uint32_t) (results.velocity * 100);
        }

#ifdef CS_PROCESS_EXT_RESULTS
        // If the caller requested debug information - fill it (even if BleCsRanging_estimatePbr failed)
        if (NULL != csResults->extendedResults)
        {
            CSProcess_ExtendedResults_t* ext = csResults->extendedResults;

            // Scalar fields
            ext->distanceMusic = (uint32_t) (debugResult.distanceMusic * 100);
            ext->distanceNN    = (uint32_t) (debugResult.distanceNN    * 100);
            ext->distanceIFFT  = (uint32_t) (debugResult.distanceIFFT  * 100);
            ext->confidence    = (uint32_t) (debugResult.confidence    * 100);
            ext->numMpc        = debugResult.numMPC;
            ext->dcand         = (uint32_t) (debugResult.dcand         * 100);
            ext->cf            = (uint32_t) (debugResult.cf            * 100);
            ext->dVar          = (uint32_t) (debugResult.d_var         * 100);
            ext->classLabel    = debugResult.class;
            ext->runtimeMs     = (uint32_t) (debugResult.runtime_ms    * 100);
            ext->peakBinIFFT   = debugResult.peakBinIFFT;
            ext->peakCountIFFT = debugResult.peakCountIFFT;
            ext->ifftValid     = debugResult.ifftValid;

            // Per antenna-path arrays
            for (uint8_t i = 0; i < CS_RANGING_MAX_ANT_PATHS; i++)
            {
                ext->qualityPaths[i] = (uint32_t) (debugResult.quality[i]   * 100);
                ext->tqiScore[i]     = (uint32_t) (debugResult.tqi_score[i] * 100);
            }

            // Per-stage runtime profile
            for (uint8_t i = 0; i < 10; i++)
            {
                ext->runtimeProfile[i] = (uint32_t) (debugResult.runtimeProfile[i] * 100);
            }
        }
#endif
    }

    return status;
}

#endif /* CHANNEL_SOUNDING */
