/******************************************************************************

 @file  osal_icall_ble.c

 @brief This file contains function that allows user setup tasks

 Group: WCS, BTS
 Target Device: cc23xx

 ******************************************************************************
 
 Copyright (c) 2013-2025, Texas Instruments Incorporated
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

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "ti/ble/stack_util/osal/osal_snv.h"
#include "ti/ble/stack_util/osal/osal.h"
#include "ti/ble/stack_util/icall/app/icall.h"
#include "ti/ble/stack_util/comdef.h"
#include "ti/ble/stack_util/osal/osal_tasks.h"


/* LL */
#include "ti/ble/controller/ll/ll.h"

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  #include "ti/ble/stack_util/osal/osal_cbtimer.h"
#endif

/* L2CAP */
#include "ti/ble/host/l2cap/l2cap.h"

/* gap */
#include "ti/ble/host/gap/gap.h"

#if defined ( GAP_BOND_MGR )
  #include "ti/ble/host/gapbondmgr/gapbondmgr_internal.h"
#endif

/* GATT */
#include "ti/ble/host/gatt/gatt.h"

/* Application */
#include "ti/ble/controller/hci/hci_tl.h"

#include "ti/ble/host/gatt/gattservapp.h"

#include "ti/ble/host/gapbondmgr/gapbondmgr.h"

#include "ti/ble/app_util/config/ble_user_config.h"
#include "ti_ble_config.h"

#include "ti/ble/stack_util/icall/stack/ble_dispatch_lite.h"

/*********************************************************************
 * FUNCTION PROTOTYPES
 */
extern void AssertHandler(uint8_t assertCause, uint8_t assertSubcause);

/*********************************************************************
 * GLOBAL VARIABLES
 */

// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] =
{
  LL_ProcessEvent,                                                  // task 0
  HCI_ProcessEvent,                                                 // task 1
#if defined ( OSAL_CBTIMER_NUM_TASKS )
  OSAL_CBTIMER_PROCESS_EVENT( osal_CbTimerProcessEvent ),           // task 2
#endif
  L2CAP_ProcessEvent,                                               // task 3
  GAP_ProcessEvent,                                                 // task 4
  SM_ProcessEvent,                                                  // task 5
  GATT_ProcessEvent,                                                // task 6
  GATTServApp_ProcessEvent,                                         // task 7
#if defined ( GAP_BOND_MGR )
  GAPBondMgr_ProcessEvent,                                          // task 8
#endif
  ble_dispatch_liteProcess                                          // task 9
};

const uint8 tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void osalInitTasks( void )
{
  ICall_EntityID entity;
  ICall_SyncHandle syncHandle;
  uint8 taskID = 0;
  uint8 i;

  uint8_t cfg_GATTServApp_att_delayed_req = 0;
  uint8_t cfg_gapBond_gatt_no_service_changed = 0;
  uint8_t cfg_gatt_max_num_prepare_writes = 0;
#if defined ( GAP_BOND_MGR )
  uint8_t cfg_gapBond_gatt_no_client = 0;
#endif

#if defined ( ATT_DELAYED_REQ )
  cfg_GATTServApp_att_delayed_req = 1;
#endif
#if defined ( GATT_NO_SERVICE_CHANGED )
  cfg_gapBond_gatt_no_service_changed = 1;
#endif
#if defined ( GATT_NO_CLIENT )
  cfg_gapBond_gatt_no_client = 1;
#endif
#if defined ( GATT_MAX_PREPARE_WRITES )
  cfg_gatt_max_num_prepare_writes = GATT_MAX_PREPARE_WRITES;
#endif

  /* LL Task */
  LL_Init( taskID++ );

  /* HCI Task */
  HCI_Init( taskID++ );

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  /* Callback Timer Tasks */
  osal_CbTimerInit( taskID );
  taskID += OSAL_CBTIMER_NUM_TASKS;
#endif

  /* L2CAP Task */
  L2CAP_Init( taskID++ );

  /* GAP Task */
  GAP_Init( taskID++ );

  /* SM Task */
  SM_Init( taskID++ );

  /* GATT Task */
  GATT_Init( taskID++ );

  /* GATT Server App Task */
  GATTServApp_Init( taskID++, cfg_GATTServApp_att_delayed_req, cfg_gapBond_gatt_no_service_changed, cfg_gatt_max_num_prepare_writes );

#if defined ( GAP_BOND_MGR )
  /* Bond Manager Task */
  GAPBondMgr_Init( taskID++, GAP_BONDINGS_MAX, GAP_CHAR_CFG_MAX, cfg_gapBond_gatt_no_client, cfg_gapBond_gatt_no_service_changed);
#endif

  ble_dispatch_liteInit(taskID++);

#if defined ( NOTIFY_PARAM_UPDATE_RJCT )
    HCI_ParamUpdateRjctEvtRegister();
#endif

  // ICall enrollment
  /* Enroll the service that this stack represents */
  ICall_enrollService(ICALL_SERVICE_CLASS_BLE, NULL, &entity, &syncHandle);

  /* Register all other OSAL tasks to use the registered dispatcher entity
   * ID as the source of dispatcher messages, even though the other OSAL
   * tasks didn't register themselves to receive messages from application.
   */
  for (i = 0; i < taskID; i++)
  {
    osal_enroll_senderid(i, entity);
  }
}

/**
 * Main entry function for the stack image
 */
int stack_main( void *arg )
{
  /* User reconfiguration of BLE Controller and Host variables */
  setBleUserConfig( (icall_userCfg_t *)arg );

  /* Establish OSAL for a stack service that requires accompanying
   * messaging service */
  if (ICall_enrollService(ICALL_SERVICE_CLASS_BLE_MSG,
                          (ICall_ServiceFunc) osal_service_entry,
                          &osal_entity,
                          &osal_syncHandle) != ICALL_ERRNO_SUCCESS)
  {
    /* abort */
    ICall_abort();
  }

  // Disable interrupts
  halIntState_t state;
  HAL_ENTER_CRITICAL_SECTION(state);

  osal_set_icall_hook(icall_liteMsgParser);

#ifndef NO_OSAL_SNV
  // Initialize NV System
  osal_snv_init( );
#endif //NO_OSAL_SNV

  // Initialize the operating system
  osal_init_system();

  // Allow interrupts
  HAL_EXIT_CRITICAL_SECTION(state);

  osal_start_system(); // No Return from here

  return 0;  // Shouldn't get here.
}

/*********************************************************************
*********************************************************************/
