/******************************************************************************

@file  app_main.c

@brief This file contains the application main functionality

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
#include <app_main.h>

// The CHANNEL_SOUNDING define added to prevent warnings, because the file uses
// Channel sounding logic only
#ifdef CHANNEL_SOUNDING
#include "app_car_node.h"
#endif

#ifdef APP_EXTERNAL_CONTROL
#include <app_extctrl_common.h>
#include <app_extctrl_connection.h>
#include <app_extctrl_pairing.h>
#include <app_extctrl_gatt.h>
#include <app_extctrl_ranging_client.h>
#include <app_extctrl_ranging_server.h>
#ifdef CAR_ACCESS_SERVER
#include <app_extctrl_ca_server.h>
#endif
#ifdef CONNECTION_MONITOR
#include <app_extctrl_cm.h>
#endif
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
#include <app_extctrl_peripheral.h>
#ifdef CONNECTION_HANDOVER
#include <app_extctrl_handover.h>
#endif // CONNECTIONG_HANDOVER
#endif // defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
#include <app_extctrl_central.h>
#ifdef CHANNEL_SOUNDING
#include <app_extctrl_cs.h>
#endif // CHANNEL_SOUNDING
#endif //  HOST_CONFIG & ( CENTRAL_CFG )
#endif // APP_EXTERNAL_CONTROL

//*****************************************************************************
//! Defines
//*****************************************************************************

#ifdef APP_EXTERNAL_CONTROL

// NPI message buffer size
#ifdef CS_PROCESS_EXT_RESULTS
#define NPI_MSG_BUFF_SIZE       4096    // For CS Process extended results, we need a bigger buffer to pass messages to the UART
#else
#define NPI_MSG_BUFF_SIZE       1500    // Default value
#endif

#define NPI_TASK_STACK_SIZE     1024
#define UART_BAUD_RATE          460800
#endif

//*****************************************************************************
//! Globals
//*****************************************************************************

// Parameters that should be given as input to the BLEAppUtil_init function
BLEAppUtil_GeneralParams_t appMainParams =
{
    .taskPriority = 2,
    .taskStackSize = 4096,
    .profileRole = (BLEAppUtil_Profile_Roles_e)(HOST_CONFIG),
    .addressMode = DEFAULT_ADDRESS_MODE,
    .deviceNameAtt = attDeviceName,
    .pDeviceRandomAddress = pRandomAddress,
};

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )
BLEAppUtil_PeriCentParams_t appMainPeriCentParams =
{
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
 .connParamUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION,
#endif //#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )

#ifdef GAP_BOND_MGR
 .gapBondParams = &gapBondParams
#endif //GAP_BOND_MGR
};
#else //observer || broadcaster
BLEAppUtil_PeriCentParams_t appMainPeriCentParams;
#endif //#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      criticalErrorHandler
 *
 * @brief   Application task entry point
 *
 * @return  none
 */
void criticalErrorHandler(int32 errorCode , void* pInfo)
{
//    trace();
//
//#ifdef DEBUG_ERR_HANDLE
//
//    while (1);
//#else
//    SystemReset();
//#endif

}

/*********************************************************************
 * @fn      App_extCtrlInit
 *
 * @brief   This function will initialize the dispatcher and the application's
 *          external control.
 *
 * @return  none
 */
static bStatus_t App_extCtrlInit(void)
{
  bStatus_t status = SUCCESS;

#ifdef APP_EXTERNAL_CONTROL
  status = Dispatcher_start(NPI_TASK_STACK_SIZE, NPI_MSG_BUFF_SIZE, UART_BAUD_RATE);

  status |= CommonExtCtrl_start();

  status |= ConnectionExtCtrl_start();

  status |= PairingExtCtrl_start();

  status |= AppGATTExtCtrl_start();

#if defined(RANGING_CLIENT_EXTCTRL_APP) && defined(RANGING_CLIENT)
  status |= AppRREQExtCtrl_start();
#endif

#if defined(RANGING_SERVER_EXTCTRL_APP) && defined(RANGING_SERVER)
  status |= AppRRSPExtCtrl_start();
#endif

#ifdef CAR_ACCESS_SERVER
  status |= CAServerExtCtrl_start();
#endif
#ifdef CONNECTION_MONITOR
  status |= ConnectionMonitorExtCtrl_start();
#endif
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
  status |= PeripheralExtCtrl_start();

#ifdef CONNECTION_HANDOVER
  status |= HandoverExtCtrl_start();
#endif // CONNECTION_HANDOVER
#endif // HOST_CONFIG & ( PERIPHERAL_CFG )

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
  status |= CentralExtCtrl_start();
#endif // HOST_CONFIG & ( CENTRAL_CFG )

#ifdef CHANNEL_SOUNDING
  status |= CSExtCtrl_start(&CarNode_registerEvtHandler);
#endif // CHANNEL_SOUNDING

#endif // APP_EXTERNAL_CONTROL

  return status;
}

/*********************************************************************
 * @fn      App_StackInitDone
 *
 * @brief   This function will be called when the BLE stack init is
 *          done.
 *          It should call the applications modules start functions.
 *
 * @return  none
 */
void App_StackInitDoneHandler(gapDeviceInitDoneEvent_t *deviceInitDoneData)
{
    bStatus_t status = SUCCESS;

//    status = App_extCtrlInit();
//    if (status != SUCCESS)
//    {
//        // TODO: Call Error Handler
//    }

    // Initialize the common component
    status = Common_start();

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
    // Any device that accepts the establishment of a link using
    // any of the connection establishment procedures referred to
    // as being in the Peripheral role.
    // A device operating in the Peripheral role will be in the
    // Peripheral role in the Link Layer Connection state.
    status = Peripheral_start();
    if (status != SUCCESS)
    {
        // TODO: Call Error Handler
    }

#ifdef CONNECTION_HANDOVER
    status = Handover_start();
    if (status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif // CONNECTION_HANDOVER

#endif // ( HOST_CONFIG & ( PERIPHERAL_CFG ) )

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
    // A device that supports the Central role initiates the establishment
    // of an active physical link. A device operating in the Central role will
    // be in the Central role in the Link Layer Connection state.
    // A device operating in the Central role is referred to as a Central.
    status = Central_start();
    if (status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif


#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )
    status = Connection_start();
    if (status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
    status = Pairing_start();
    if (status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
    status = AppGATT_start();
    if (status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#ifdef CAR_ACCESS_SERVER
    status = CAServer_start();
    if (status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif

#if defined (BLE_V41_FEATURES) && (BLE_V41_FEATURES & L2CAP_COC_CFG)
    status = L2CAPCOC_start();
    if (status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif // BLE_V41_FEATURES & L2CAP_COC_CFG

#ifdef CHANNEL_SOUNDING
    status = ChannelSounding_start();
    if (status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif // CHANNEL_SOUNDING

#ifdef CONNECTION_MONITOR
    ConnectionMonitor_start();
#endif // CONNECTION_MONITOR

#endif // ( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )

// The CHANNEL_SOUNDING define added to prevent warnings, because the file uses
// Channel sounding logic only
#ifdef CHANNEL_SOUNDING
    status = CarNode_start();
    if( status != SUCCESS )
    {
        // TODO: Call Error Handler
    }
#endif //CHANNEL_SOUNDING

}

/*********************************************************************
 * @fn      appMain
 *
 * @brief   Application main function
 *
 * @return  none
 */
void appMain(void)
{
    // Call the BLEAppUtil module init function
    BLEAppUtil_init(&criticalErrorHandler, &App_StackInitDoneHandler,
                    &appMainParams, &appMainPeriCentParams);
}
