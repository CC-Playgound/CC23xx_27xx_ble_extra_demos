/******************************************************************************

@file  app_pairing_api.h

@brief This file contains the Pairing APIs and structures.


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

#ifndef APP_PAIRING_API_H
#define APP_PAIRING_API_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include <app_extctrl_common.h>

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
typedef struct
{
  uint8_t mode;                      //!< The read mode, @ref GAPBondMgr_Read_Mode
  uint8_t pIdentifier[B_ADDR_LEN];   //!< Peer address of the bond to read in GAPBOND_READ_BY_ADDR mode
                                     //!< or index of the bond to read in GAPBOND_READ_BY_IDX mode
  GAP_Peer_Addr_Types_t addrType;    //!< The address type of the bond to read if GAPBOND_READ_BY_ADDR mode
} AppPairing_readBond_t;

/// @brief Aggregation of values relevant for reading a bond from NV
typedef struct
{
    gapBondRec_t pBondRec;                      // pBondRec - basic bond record
    gapBondLTK_t localLtk;                      // localLTK - LTK used by this device during pairing
    gapBondLTK_t devLtk;                        // devLTK - LTK used by the peer device during pairing
    uint8_t pIRK[KEYLEN];                       // pIRK - IRK used by the peer device during pairing
    uint8_t pSRK[KEYLEN];                       // pSRK - SRK used by the peer device during pairing
    uint32_t signCount;                         // signCounter - Sign counter used by the peer device during pairing
    gapBondCharCfg_t *charCfg;                  // charCfg - GATT characteristic configuration
} AppPairing_BondNvRecord_t;

// Write Bond Command
typedef struct
{
  gapBondRec_t bondRec;                         //!< Basic bond record
  gapBondLTK_t *pLocalLtk;                      //!< LTK used by this device during pairing
  gapBondLTK_t *pDevLtk;                        //!< LTK used by the peer device during pairing
  uint8_t *pIRK;                                //!< IRK used by the peer device during pairing
  uint8_t *pSRK;                                //!< SRK used by the peer device during pairing
  uint32_t signCount;                           //!< Sign counter used by the peer device during pairing
  gapBondCharCfg_t charCfg[GAP_CHAR_CFG_MAX];   //!< GATT characteristic configuration
} AppPairing_writeBond_t;

/*********************************************************************
 * Enumerators
 */

/*********************************************************************
 * Structures
 */

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      Pairing_readBond
 *
 * @brief   The purpose of this function is to read a bond information.
 *
 * @param  pReadData - The data that indicates which bond to read
 * @output pBondRecordInfo - The pointer to return the bond record information into
 *
 * @return  SUCCESS if all bond information was read successfully
 *          bleGAPNotFound if there is no bond record
 *          bleInvalidRange if the bond index is out of range or the mode is invalid
 *          bleGAPBondItemNotFound if an item is not found in nv
 */
uint8_t Pairing_readBond(AppPairing_readBond_t *pReadData, AppPairing_BondNvRecord_t *pBondRecordInfo);

/*********************************************************************
 * @fn      Pairing_writeBond
 *
 * @brief   The purpose of this function is to write a bond information
 *
 * @param pWriteBond - The data information to write as a bond record
 *
 * @return SUCCESS if bond was writen successfully or updated if exists
 *         bleNoResources if there are no empty slots
 */
uint8_t Pairing_writeBond(AppPairing_writeBond_t *pWriteBond);

/*********************************************************************
 * @fn      Pairing_registerEvtHandler
 *
 * @brief   This function is called to register the external event handler
 *          function.
 *
 * @return  None
 */
void Pairing_registerEvtHandler(ExtCtrl_eventHandler_t fEventHandler);

#ifdef __cplusplus
}
#endif

#endif /* APP_PAIRING_API_H */
