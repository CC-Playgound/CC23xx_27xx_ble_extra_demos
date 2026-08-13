/******************************************************************************

@file  app_extctrl_ranging_server.h

@brief This file provides the interface for external control commands to the
       ranging server: enable, send CS event, and send CS event continue.

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

#ifndef APP_EXTCTRL_RANGING_SERVER_H
#define APP_EXTCTRL_RANGING_SERVER_H

#ifdef __cplusplus
extern "C"
{
#endif


/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>

/*********************************************************************
 * TYPEDEFS
 */

// Enable command structure
typedef struct __attribute__((packed))
{
    uint16_t connHandle;
} AppExtCtrlRRSPEnableCmd_t;

// Send CS event (raw data) command structure
typedef struct __attribute__((packed))
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
} AppExtCtrlRRSPSendCsEvtCmd_t;

// Send CS event continue (raw data) command structure
typedef struct __attribute__((packed))
{
  uint8_t  csEvtOpcode;          //!< CS Event Code @ref csEventOpcodes_e
  uint16_t connHandle;           //!< Connection handle
  uint8_t  configID;             //!< Configuration ID
  uint8_t  procedureDoneStatus;  //!< Procedure done status @ref CS_Procedure_Done_Status
  uint8_t  subeventDoneStatus;   //!< Subevent done status
  uint8_t  abortReason;          //!< Abort reason @ref CS_Abort_Reason
  uint8_t  numAntennaPath;       //!< Number of antenna paths
  uint8_t  numStepsReported;     //!< Number of steps reported
  uint16_t dataLen;              //!< Data length
  uint8_t  data[];               //!< Data
} AppExtCtrlRRSPSendCsEvtContCmd_t;

/*********************************************************************
 * API FUNCTIONS
 */

/**
 * @brief Initialize and register the ranging server external control handler.
 *
 * @return SUCCESS/FAILURE
 */
uint8_t AppRRSPExtCtrl_start(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_EXTCTRL_RANGING_SERVER_H */
