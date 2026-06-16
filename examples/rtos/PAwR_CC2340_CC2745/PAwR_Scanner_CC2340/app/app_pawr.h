/******************************************************************************

 @file  app_pawr.h

 @brief This file contains the Periodic Advertising with Responses (PAwR)
        module for BLE applications.

 The app_pawr module provides bidirectional communication using BLE Periodic
 Advertising with Responses. It supports two roles:

 PAwR Advertiser:
 - Creates periodic advertising with subevents
 - Provides subevent data to responders
 - Receives and processes responses from responders

 PAwR Responder:
 - Discovers and synchronizes to a PAwR advertiser
 - Receives subevent data from the advertiser
 - Sends response data in designated response slots

 Usage:
 1. Create a configuration structure with role and parameters
 2. Implement application callbacks for data exchange
 3. Call AppPawr_init() with the configuration
 4. For Responder: Use AppPawr_sendResponse() to send responses
 5. For Responder: Use AppPawr_setSubeventSync() to update subevent subscription

 This module uses both BLEAppUtil wrapper APIs and direct GAP API calls
 for PAwR-specific functionality.

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

#ifndef APP_PAWR_H
#define APP_PAWR_H

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>

#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "ti/ble/stack_util/bcomdef.h"

/*****************************************************************************
 * Defines
 *****************************************************************************/

/* Maximum number of subevents supported by the module */
#define APP_PAWR_MAX_SUBEVENTS                          (128)

/* Maximum response data length in bytes */
#define APP_PAWR_MAX_RESPONSE_DATA_LEN                  (251)

/* Maximum subevent data length in bytes */
#define APP_PAWR_MAX_SUBEVENT_DATA_LEN                  (251)

/* Response data status values (from BLE Core Spec) */
#define APP_PAWR_DATA_STATUS_COMPLETE                   (0x00)
#define APP_PAWR_DATA_STATUS_INCOMPLETE_MORE            (0x01)
#define APP_PAWR_DATA_STATUS_TRUNCATED                  (0x02)
#define APP_PAWR_DATA_STATUS_FAILED_TO_RECEIVE          (0xFF)

/* Default values that can be overridden by configuration */

/* Default periodic advertising interval minimum (400 * 1.25ms = 500ms) */
#ifndef APP_PAWR_DEFAULT_PERIODIC_ADV_INTERVAL_MIN
#define APP_PAWR_DEFAULT_PERIODIC_ADV_INTERVAL_MIN      (400)
#endif

/* Default periodic advertising interval maximum (400 * 1.25ms = 500ms) */
#ifndef APP_PAWR_DEFAULT_PERIODIC_ADV_INTERVAL_MAX
#define APP_PAWR_DEFAULT_PERIODIC_ADV_INTERVAL_MAX      (400)
#endif

/* Default sync timeout (200 * 10ms = 2s) */
#ifndef APP_PAWR_DEFAULT_SYNC_TIMEOUT
#define APP_PAWR_DEFAULT_SYNC_TIMEOUT                   (200)
#endif

/* Default skip value (no events skipped) */
#ifndef APP_PAWR_DEFAULT_SKIP
#define APP_PAWR_DEFAULT_SKIP                           (0)
#endif

/* Default advertising SID for PAwR */
#ifndef APP_PAWR_DEFAULT_ADV_SID
#define APP_PAWR_DEFAULT_ADV_SID                        (5)
#endif

/* Default number of subevents */
#ifndef APP_PAWR_DEFAULT_NUM_SUBEVENTS
#define APP_PAWR_DEFAULT_NUM_SUBEVENTS                  (1)
#endif

/* Default subevent interval (20 * 1.25ms = 25ms) */
#ifndef APP_PAWR_DEFAULT_SUBEVENT_INTERVAL
#define APP_PAWR_DEFAULT_SUBEVENT_INTERVAL              (20)
#endif

/* Default response slot delay (16 * 1.25ms = 20ms) */
/* Increased to give responder more time to process data and send response */
#ifndef APP_PAWR_DEFAULT_RESPONSE_SLOT_DELAY
#define APP_PAWR_DEFAULT_RESPONSE_SLOT_DELAY            (16)
#endif

/* Default response slot spacing (8 * 0.125ms = 1ms) */
#ifndef APP_PAWR_DEFAULT_RESPONSE_SLOT_SPACING
#define APP_PAWR_DEFAULT_RESPONSE_SLOT_SPACING          (8)
#endif

/* Default number of response slots per subevent */
#ifndef APP_PAWR_DEFAULT_NUM_RESPONSE_SLOTS
#define APP_PAWR_DEFAULT_NUM_RESPONSE_SLOTS             (4)
#endif

/* Default number of response slots per subevent */
#ifndef APP_PAWR_EXTENDED_ADV_PROP
#define APP_PAWR_EXTENDED_ADV_PROP                      (0)
#endif

/* Subevent data buffer count - automatically sized based on configured subevents */
/* Uses the configured default with a minimum of 4 for flexibility */
/* This conserves RAM by only allocating what's needed (not the full 128 max) */
#ifndef APP_PAWR_SUBEVENT_BUFFER_COUNT
#define APP_PAWR_SUBEVENT_BUFFER_COUNT                  ((APP_PAWR_DEFAULT_NUM_SUBEVENTS) < 4 ? 4 : (APP_PAWR_DEFAULT_NUM_SUBEVENTS))
#endif

/* BLE specification constants */
#define APP_PAWR_MAX_SID                                (15)   /* Maximum advertising SID value (0-15) */
#define APP_PAWR_MAX_ADDR_TYPE                          (3)    /* Maximum address type (0=Public, 1=Random, 2=Public ID, 3=Random ID) */
#define APP_PAWR_MAX_RESPONSES_PER_EVENT                (32)   /* Maximum responses we can handle per event */
#define APP_PAWR_SUBEVENT_BUFFER_SIZE                   (1024) /* Size of subevent data buffer in bytes */
#define APP_PAWR_SUBEVENT_HEADER_SIZE                   (4)    /* Size of subevent header: num, start, count, len */
#define APP_PAWR_RESPONSE_HEADER_SIZE                   (6)    /* Size of response header: txPwr, rssi, cte, slot, status, len */

/*****************************************************************************
 * Types
 *****************************************************************************/

/**
 * @brief PAwR Responder State Machine States
 *
 * Defines the state of the PAwR Responder during advertiser discovery and sync.
 */
typedef enum
{
    APP_PAWR_RESPONDER_STATE_IDLE,        ///< Initial state, not scanning or synced
    APP_PAWR_RESPONDER_STATE_SCANNING,    ///< Scanning for advertiser by SID
    APP_PAWR_RESPONDER_STATE_SYNCING,     ///< Advertiser found, creating periodic sync
    APP_PAWR_RESPONDER_STATE_SYNCED,      ///< Periodic sync established
    APP_PAWR_RESPONDER_STATE_SUBSCRIBED   ///< Subevent synchronization configured
} AppPawr_ResponderState_t;

/**
 * @brief Subevent data structure
 *
 * Contains data for a single subevent to be transmitted by the advertiser.
 * Application fills this structure via the subevent data callback.
 */
typedef struct
{
    uint8_t subeventNum;       ///< Subevent number (0x00-0x7F)
    uint8_t rspSlotStart;      ///< First response slot for this subevent
    uint8_t rspSlotCount;      ///< Number of response slots
    uint8_t dataLen;           ///< Length of subevent data (0-251)
    uint8_t *pData;            ///< Pointer to subevent data buffer (app-owned)
} AppPawr_SubeventData_t;

/**
 * @brief Response information structure
 *
 * Contains information about a single response received by the advertiser.
 */
typedef struct
{
    int8_t  txPower;           ///< TX power of response (dBm)
    int8_t  rssi;              ///< RSSI of response (dBm)
    uint8_t cteType;           ///< CTE type (0x00=AoA, 0x01=AoD 1us, 0x02=AoD 2us, 0xFF=No CTE)
    uint8_t responseSlot;      ///< Response slot index
    uint8_t dataStatus;        ///< Data status (0x00=complete, 0x01=incomplete more, 0x02=truncated, 0xFF=failed to receive)
    uint8_t dataLen;           ///< Length of response data (0-251)
    uint8_t *pData;            ///< Pointer to response data (stack-owned, copy if needed)
} AppPawr_ResponseInfo_t;

/**
 * @brief Advertiser callback to provide subevent data
 *
 * Called by the module when the controller requests subevent data.
 * Application must fill the pSubeventData array with data for the
 * requested subevents.
 *
 * @param advHandle       - Advertising set handle
 * @param subeventStart   - First subevent number being requested
 * @param subeventCount   - Number of consecutive subevents being requested
 * @param pSubeventData   - Output array to fill with subevent data
 *
 * @return None
 */
typedef void (*AppPawr_SubeventDataCb_t)(
    uint8_t advHandle,
    uint8_t subeventStart,
    uint8_t subeventCount,
    AppPawr_SubeventData_t *pSubeventData
);

/**
 * @brief Advertiser callback for received responses
 *
 * Called by the module when responses are received from responders.
 *
 * @param advHandle      - Advertising set handle
 * @param subevent       - Subevent number that was responded to
 * @param numResponses   - Number of responses received
 * @param pResponses     - Array of response information structures
 *
 * @return None
 */
typedef void (*AppPawr_ResponseReceivedCb_t)(
    uint8_t advHandle,
    uint8_t subevent,
    uint8_t numResponses,
    AppPawr_ResponseInfo_t *pResponses
);

/**
 * @brief Responder callback for received subevent data
 *
 * Called by the module when subevent data is received from the advertiser.
 *
 * @param syncHandle  - Sync handle
 * @param subevent    - Subevent number
 * @param dataLen     - Length of received data
 * @param pData       - Pointer to received data (stack-owned, copy if needed)
 *
 * @return None
 */
typedef void (*AppPawr_SubeventReceivedCb_t)(
    uint16_t syncHandle,
    uint8_t subevent,
    uint8_t dataLen,
    uint8_t *pData
);

/**
 * @brief Responder callback to provide response data
 *
 * Called by the module to check if the application wants to send a response
 * for a given subevent and slot.
 *
 * @param syncHandle         - Sync handle
 * @param requestSubevent    - Subevent being responded to
 * @param responseSubevent   - Subevent containing the response slot
 * @param responseSlot       - Response slot index
 * @param pDataLen           - Output: Length of response data (0-251)
 * @param pData              - Output: Response data buffer (app provides, max 251 bytes)
 *
 * @return 1 if response should be sent, 0 to skip this slot
 */
typedef uint8_t (*AppPawr_ResponseDataCb_t)(
    uint16_t syncHandle,
    uint8_t requestSubevent,
    uint8_t responseSubevent,
    uint8_t responseSlot,
    uint8_t *pDataLen,
    uint8_t *pData
);

/**
 * @brief PAwR Configuration for Advertiser
 */
typedef struct
{
    uint8_t advSID;                                 ///< Advertising SID (0-15)
    uint16_t periodicAdvIntervalMin;                ///< Min periodic adv interval (N * 1.25ms)
    uint16_t periodicAdvIntervalMax;                ///< Max periodic adv interval (N * 1.25ms)
    uint16_t periodicAdvProp;                       ///< Periodic adv properties (bit 6: include TxPower)

    /* PAwR-specific parameters */
    uint8_t numSubevents;                           ///< Number of subevents (0x01-0x80)
    uint8_t subeventInterval;                       ///< Interval between subevents (N * 1.25ms)
    uint8_t responseSlotDelay;                      ///< Delay to first response slot (N * 1.25ms)
    uint8_t responseSlotSpacing;                    ///< Spacing between response slots (N * 0.125ms)
    uint8_t numOfResponseSlots;                     ///< Number of response slots per subevent

    /* Application callbacks */
    AppPawr_SubeventDataCb_t subeventDataCb;        ///< Callback to get subevent data (required)
    AppPawr_ResponseReceivedCb_t responseCb;        ///< Callback for received responses (optional)
} AppPawr_AdvertiserConfig_t;

/**
 * @brief PAwR Configuration for Responder
 */
typedef struct
{

    uint8_t advSID;                                 ///< Advertising SID to sync with (0-15) - PRIMARY FILTER
    uint16_t skip;                                  ///< Max events to skip (0x0000 to 0x01F3)
    uint16_t syncTimeout;                           ///< Sync timeout in 10ms units (0x000A to 0x4000)
    uint8_t options;                                ///< Sync options

    /* Subevent subscription */
    uint8_t numSubevents;                           ///< Number of subevents to sync (1-128)
    uint8_t *subEvents;                             ///< Array of subevent indices to sync

    /* Response slot assignment */
    uint8_t assignedResponseSlot;                   ///< Response slot this device should use (0-255)
    uint8_t deviceId;                               ///< Device identifier (0-255)

    /* Application callbacks */
    AppPawr_SubeventReceivedCb_t subeventCb;        ///< Callback for received subevent data (required)
    AppPawr_ResponseDataCb_t responseDataCb;        ///< Callback to get response data (optional)
} AppPawr_ResponderConfig_t;


#ifdef __cplusplus
}
#endif

#endif /* APP_PAWR_H */
