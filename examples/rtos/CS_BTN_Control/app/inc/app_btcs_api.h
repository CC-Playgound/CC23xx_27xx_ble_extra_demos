/******************************************************************************

@file  app_btcs_api.h

@brief This file contains the CCC BTCS API and its structures.

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

#ifndef APP_BTCS_H
#define APP_BTCS_H

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

// BTCS message types
#define BTCS_RANGING_SERVICE_MESSAGE_TYPE   0x07

// BTCS Ranging Service Message Identifiers
#define BTCS_RANGING_PROCEDURE_RESULTS_START     0x01
#define BTCS_RANGING_PROCEDURE_RESULTS_CONTINUE  0x02

/*********************************************************************
 * MACROS
 */

// BTCS Macros to extract Procedure and Subevent status
#define BTCS_EXTRACT_PROCDONE_STATUS(status)        ((status) & 0x0F)
#define BTCS_EXTRACT_SUBEVENTDONE_STATUS(status)    ((status) & 0xF0)

// BTCS Macros to extract mode and status for first and second steps from 1 byte
#define BTCS_EXTRACT_MODE_FIRST(val)        ((val) & 0x3)
#define BTCS_EXTRACT_MODE_SECOND(val)       ((val >> 4) & 0x3)
#define BTCS_EXTRACT_STATUS_FIRST(val)      (((val) & 0x8) >> 3)
#define BTCS_EXTRACT_STATUS_SECOND(val)     (((val >> 4) & 0x8) >> 3)

/*********************************************************************
 * TYPEDEFS
 */

PACKED_TYPEDEF_STRUCT
{
    uint8_t msgType:6;  //!< Message type, one of: @ref BTCS_RANGING_PROCEDURE_RESULTS_START and @ref BTCS_RANGING_PROCEDURE_RESULTS_CONTINUE
    uint8_t rfu:2;
} DK_MsgHdr;

PACKED_TYPEDEF_STRUCT
{
    DK_MsgHdr msgHdr;       //!< Message type
    uint8_t payloadHdr;     //!< Message identifier
    uint16_t msgLen;        //!< Length of the entire message, including this data
    uint8_t payloadData[];  //!< DK message payload, one of the following: @ref BTCS_ProcHdr_t and @ref BTCS_ProcContHdr_t
} DK_Message_t;

PACKED_TYPEDEF_STRUCT
{
    uint8_t seqNum;     //!< BTCS procedure sequence number derived from the least significant byte of the CS procedure counter since BTCS establishment
    uint8_t configId;   //!< CS Config ID
    uint8_t moduleId;   //!< Vehicle Bluetooth Module
    uint8_t data[];     //!< Subevent header of type @ref BTCS_SubeventHdrInit_t or @ref BTCS_SubeventHdrRefl_t
} BTCS_ProcHdr_t;

PACKED_TYPEDEF_STRUCT
{
    uint8_t seqNum;     //!< BTCS procedure sequence number derived from the least significant byte of the CS procedure counter since BTCS establishment
    uint8_t data[];     //!< Subevent header of type @ref BTCS_SubeventContHdr_t
} BTCS_ProcContHdr_t;

PACKED_TYPEDEF_STRUCT
{
    uint16_t startAclConnectionEvent;   //!< The ACL connection event count for the anchor point of this subevent
    uint8_t status;                     //!< Procedure and subevent status. Bits 0-3: procedure, Bits 4-7: subevent
    uint8_t abortReason;                //!< CS Abort reason
    int16_t frequencyCompensation;      //!< Frequency compensation (carrier frequency offset [CFO]) value in units of 0.01 ppm
    uint8_t PBR;                        //!< Phase-Based Ranging Format. 0: 12 bit I/Q. 1-255: RFU
    int8_t referencePowerLevel;         //!< Reference power level for scaling I/Q magnitudes
    uint8_t totalSubeventSteps;         //!< Number of steps that will be reported as a part of this BTCS procedure subevent
    uint8_t numStepsReported;           //!< The number of steps of the subevent reported in the CSSubeventData following this header
    uint8_t data[];                     //!< Subevent data for modes 0 - 3
} BTCS_SubeventHdrInit_t;

PACKED_TYPEDEF_STRUCT
{
    uint16_t startAclConnectionEvent;   //!< The ACL connection event count for the anchor point of this subevent
    uint8_t status;                     //!< Procedure and subevent status. Bits 0-3: procedure, Bits 4-7: subevent
    uint8_t abortReason;                //!< CS Abort reason
    uint8_t PBR;                        //!< Phase-Based Ranging Format. 0: 12 bit I/Q. 1-255: RFU
    int8_t referencePowerLevel;         //!< Reference power level for scaling I/Q magnitudes
    uint8_t totalSubeventSteps;         //!< Number of steps that will be reported as a part of this BTCS procedure subevent
    uint8_t numStepsReported;           //!< The number of steps of the subevent reported in the CSSubeventData following this header
    uint8_t data[];                     //!< Subevent data for modes 0 - 3
} BTCS_SubeventHdrRefl_t;

PACKED_TYPEDEF_STRUCT
{
    uint16_t startAclConnectionEvent;   //!< The ACL connection event count for the anchor point of this subevent
    uint8_t numStepsReported;           //!< The number of steps of the subevent reported in the CSSubeventData following this header
    uint8_t data[];                     //!< Subevent data for modes 0 - 3
} BTCS_SubeventContHdr_t;

typedef CS_modeZeroInitStep_t BTCS_modeZeroInitStep_t;
typedef CS_modeZeroReflStep_t BTCS_modeZeroReflStep_t;
typedef CS_modeOneStep_t BTCS_modeOneStep_t;

PACKED_TYPEDEF_STRUCT
{
  uint32_t i:12;    //!< I sample
  uint32_t q:12;    //!< Q sample
} BTCS_modeTwoStepData_t;

PACKED_TYPEDEF_STRUCT
{
  uint8_t quality;      //!< quality per antenna path (2 bits for each path, starting from lsb, first antenna path)

  /*
   * array contains I and Q step data for all paths.
   * Each 24 bits contains I,Q data for a specific path.
   * The order of each I,Q sample corresponds to the antenna permutation index
   */
  uint8_t IQData[];
} BTCS_modeTwoStep_t;

PACKED_TYPEDEF_STRUCT
{
  uint8_t packetQuality;    //!< Packet quality
  uint8_t packetNadm;       //!< Attack likelihood
  uint8_t packetRssi;       //!< Packet RSSI
  uint16_t timeDiff;        //!< Time difference in 0.5 ns units between arrival and departure of CS packets
  uint8_t antennaUsed;      //!< Antenna used by the sender for CS_SYNC packet
  BTCS_modeTwoStep_t data;
} BTCS_modeThreeStep_t;

typedef struct
{
  uint8_t mode:2;   //!< Main mode of the step. One of the following: @ref CS_MODE_0 @ref CS_MODE_1 @ref CS_MODE_2 @ref CS_MODE_3
  uint8_t rfu:1;    //!< RFU bit
  uint8_t status:1; //!< 0 - step OK, 1 - step aborted and excluded in data
} BTCS_stepMode_t;

/*********************************************************************
 * FUNCTIONS
 */

#ifdef __cplusplus
}
#endif

#endif /* APP_BTCS_H */
