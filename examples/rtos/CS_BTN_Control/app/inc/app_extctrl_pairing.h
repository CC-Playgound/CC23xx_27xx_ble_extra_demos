/******************************************************************************

@file  app_extctrl_pairing.h

@brief This file parse and process the messages comes form the external control
 dispatcher module, and build the events from the app_pairing.c application and
 send it to the external control dispatcher module back.

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

#ifndef APP_APP_EXTCTRL_PAIRING_H
#define APP_APP_EXTCTRL_PAIRING_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include <app_extctrl_common.h>
#include <app_pairing_api.h>

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
// Read Bond Command
typedef struct __attribute__((packed))
{
  uint8_t mode;                      //!< The read mode, @ref GAPBondMgr_Read_Mode
  uint8_t pIdentifier[B_ADDR_LEN];   //!< Peer address of the bond to read in GAPBOND_READ_BY_ADDR mode
                                     //!< or index of the bond to read in GAPBOND_READ_BY_IDX mode
  GAP_Peer_Addr_Types_t addrType;    //!< The address type of the bond to read if GAPBOND_READ_BY_ADDR mode
} AppExtCtrlPairingReadBondCmd_t;

// Structure of NV data for the connected device's characteristic configuration
typedef struct __attribute__((packed))
{
  uint16_t attrHandle;  // attribute handle
  uint8_t  value;       // attribute value for this device
} AppExtCtrlPairingCharCfg_t;

// Structure of NV data information
typedef struct __attribute__((packed))
{
  // Basic bond record
  uint8_t peerAddr[B_ADDR_LEN];                         //!< Peer's address
  GAP_Peer_Addr_Types_t peerAddrType;                   //!< Peer's address type
  uint8_t stateFlags;                                   //!< State flags of bond
  // LTK used by this device during pairing
  uint8_t localLTK[KEYLEN];                             //!< Long Term Key (LTK)
  uint16_t localDiv;                                    //!< LTK eDiv
  uint8_t localRand[B_RANDOM_NUM_SIZE];                 //!< LTK random number
  uint8_t localKeySize;                                 //!< LTK key size
  // LTK used by the peer device during pairing
  uint8_t peerLTK[KEYLEN];                              //!< Long Term Key (LTK)
  uint16_t peerDiv;                                     //!< LTK eDiv
  uint8_t peerRand[B_RANDOM_NUM_SIZE];                  //!< LTK random number
  uint8_t peerKeySize;                                  //!< LTK key size
  uint8_t pIRK[KEYLEN];                                 //!< IRK used by the peer device during pairing
  uint8_t pSRK[KEYLEN];                                 //!< SRK used by the peer device during pairing
  uint32_t signCount;                                   //!< Sign counter used by the peer device during pairing
  AppExtCtrlPairingCharCfg_t charCfg[GAP_CHAR_CFG_MAX]; //!< GATT characteristic configuration
} AppExtCtrlPairingBondInfo_t;

// Write Bond Command
typedef struct __attribute__((packed))
{
  AppExtCtrlPairingBondInfo_t bondInfo;
} AppExtCtrlPairingWriteBondCmd_t;

// Set OOP Enable Command
typedef struct __attribute__((packed))
{
  uint8_t enable;                         //!< Enable or disable OOB
} AppExtCtrlPairingSetOobEnableCmd_t;

// Set OOB Data Command
typedef struct __attribute__((packed))
{
  uint8_t oobConfirm[KEYLEN];             //!< OOB data
  uint8_t oobRand[KEYLEN];                //!< OOB data
} AppExtCtrlPairingSetRemoteOobDataCmd_t;

typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;
  uint8_t  confirm[KEYLEN];              //calculated/received confirm value
  uint8_t  rand[KEYLEN];                 //calculated/received random number
} AppExtCtrlPairingGetLocalOobDataEvent_t;

// Max Bonds Event
typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;
  uint8_t maxNumCharCfg;                        //!< The max num of char cfg entries
} AppExtCtrlPairingMaxNumCharCfgEvent_t;

// Read Bonds Event
typedef struct __attribute__((packed))
{
  AppExtCtrlEvent_e event;
  AppExtCtrlPairingBondInfo_t bondInfo;
} AppExtCtrlPairingReadBondEvent_t;

/*********************************************************************
 * Enumerators
 */

/*********************************************************************
 * Structures
 */

/*********************************************************************
 * @fn      PairingExtCtrl_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific message handler of the pairing module
 *          to the external control dispatcher, and register the call back
 *          event handler function to the pairing application.
 *
 * @return  SUCCESS/FAILURE
 */
bStatus_t PairingExtCtrl_start(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_APP_EXTCTRL_PAIRING_H */
