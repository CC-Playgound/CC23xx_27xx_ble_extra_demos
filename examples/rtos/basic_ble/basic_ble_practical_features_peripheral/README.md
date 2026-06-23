
Purpose / Scope
===============

This purpose of this example is to demonstrate implementation of frequently used BLE features. The project is based on examples from SimpleLink F3 SDK. User is considered to be already familiar with the SDK, for more information of SimpleLink SDK please refer to the SDK documentations.

Demo features
=============

This project demonstrates following features:

- Send parameter update request from a clock timeout callback
- Disabling CSA#2
- Register connection event callback to get RSSI

All features are under pre-defined symbols. Users can enable/disable the features from CCS project properties -> Build -> Tools -> Arm Compiler -> Predefined Symbols. The corresponding pre-defined symbols of the features are;

- ``PARAM_UPDATE_REQ_AFTER_CLOCK_TIMEOUT``
- ``DISABLE_CSA2``
- ``CONNECTION_EVENT_CALLBACK``

Prerequisites
=============

#### Hardware Requirements

Before running the demo, the user will need the following components:

- 1x [LP-EM-CC2340R5 Launchpad](https://www.ti.com/tool/LP-EM-CC2340R5)
- 1x [LP-XDS110 Debugger](https://www.ti.com/tool/LP-XDS110ET)
- 1x USB type C to USB A cable (included with LP-XDS110ET debugger)

#### Firmware Requirements

- [SimpleLink Low Power F3 SDK (9.11.00.18)](https://www.ti.com/tool/download/SIMPLELINK-LOWPOWER-F3-SDK)

#### Mobile Application Requirements

- [SimpleLink Connect Application on iOS](https://apps.apple.com/app/simplelink-connect/id6445892658)
- [SimpleLink Connect Application on Android](https://play.google.com/store/apps/details?id=com.ti.connectivity.simplelinkconnect)
- [SimpleLink Connect Application source code](https://www.ti.com/tool/SIMPLELINK-CONNECT-SW-MOBILE-APP)


Send parameter update request from a clock timeout callback
===========================================================

The code under pre-defined symbol ``PARAM_UPDATE_REQ_AFTER_CLOCK_TIMEOUT`` demonstrates 2 frequently used features:

- Create a clock to execute periodic tasks
- Send a parameter update request from the peripheral

The demo combines the 2 features to send a parameter update request from a clock timeout callback. This is a common use case to allow the peripheral device to request for a larger connection interval after the service discovery for the sake of power consumption.

A simple flow chart of the code:

- Create and start a clock after a BLE connection is established
- Send a parameter update request from the clock timeout callback

All the code required for this demo is inside ``app_connection.c``.

## Create a clock instance

SimpleLink F3 SDK(9.11 and above) provides APIS to easily creating a one-time or periodic clock to execute a delayed or periodic task. API declarations are inside ``<SDK_INSTALL_DIR>\source\ti\ble\app_util\framework\bleapputil_timers.h`` and definitions are inside ``<SDK_INSTALL_DIR>\source\ti\ble\app_util\framework\src\bleapputil_timers.c``.

In this demo, ``BLEAppUtil_startTimer`` is called in ``Connection_ConnEventHandler`` -> ``case BLEAPPUTIL_LINK_ESTABLISHED_EVENT`` to create a one-time clock with 6000ms timeout value. The function pointer of ``sendParamUpdateReq`` is passed as an input parameter to allow the clock callback function to pass the actual handler function to the BLE stack message queue.

```
void Connection_ConnEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch(event)
    {
        case BLEAPPUTIL_LINK_ESTABLISHED_EVENT:
        {
            ...
#ifdef PARAM_UPDATE_REQ_AFTER_CLOCK_TIMEOUT
            // Start a clock to send parameter update request
            paramUpdateReqClockHandle = BLEAppUtil_startTimer((void *)paramUpdateReqTimeoutCB,  // clock timeout callback
                                                              PARAM_UPDATE_REQ_TIMEOUT, // clock timeout value in milliseconds
                                                              false,                    // periodic clock or not
                                                              sendParamUpdateReq);      // data pointer passed to callback
#endif // #ifdef PARAM_UPDATE_REQ_AFTER_CLOCK_TIMEOUT
            ...
```

In ``Connection_ConnEventHandler`` -> ``case BLEAPPUTIL_LINK_TERMINATED_EVENT``, ``BLEAppUtil_abortTimer`` is called to abort the clock in case of a connection is terminated. This avoids sending a parameter update request when a connection is terminated before the clock timeout is reached.

## Using BLEAppUtil_invokeFunction for BLE APIs

2 steps are implemented to send the parameter update request from the clock timeout callback:

- Call ``BLEAppUtil_invokeFunctionNoData`` to pass the actual handler function to the BLE stack message queue
- Execute the handler function to send parameter update request from BLE task context

The reason for this implementation is that all BLE APIs should be called from an icall-enabled task context, rather than a non-icall task or a callback function. Detail of the limitation can be found in the [BLE5-Stack User's Guide](https://software-dl.ti.com/simplelink/esd/simplelink_lowpower_f3_sdk/9.11.00.18/exports/docs/ble5stack/ble_user_guide/html/ble-stack-5.x/the-application-cc23xx.html#icall).

```
void paramUpdateReqTimeoutCB(BLEAppUtil_timerHandle timerHandle, BLEAppUtil_timerTermReason_e reason, void *pData)
{
    // Only send param update req if clock timeout
    // If clock aborted, means connection terminated within the clock timeout
    if(reason == BLEAPPUTIL_TIMER_TIMEOUT)
    {
        // Switch to BLE task context to execute since 
        BLEAppUtil_invokeFunctionNoData(pData);
    }
}
```

``sendParamUpdateReq`` is the actual handler function of the clock timeout. It calls ``BLEAppUtil_paramUpdateReq`` to send the parameter update request with user defined connection parameters. For simplicity the current implementation only handles 1 connection. If multiple connections exist, user should create extra clock instance for each connection to allow parallel execution of parameter update procedures.
```
void sendParamUpdateReq(void)
{
    bStatus_t status;
    gapUpdateLinkParamReq_t pParamUpdateReq =
    {
        .connectionHandle = linkDB_NumActive() - 1, // The latest connection
        .intervalMin = 400,
        .intervalMax = 800,
        .connLatency = 0,
        .connTimeout = 600
    };

    // Send a connection param update request
    status = BLEAppUtil_paramUpdateReq(&pParamUpdateReq);

    // Print the status of the param update call
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: ParamUpdateReq = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);
}
```

Disabling CSA#2
===============

The code under pre-defined symbol ``DISABLE_CSA2`` demonstrates how to disable a certain LE feature. This is needed for compatibility with older BLE stack versions or user specified applications.

The [BLE5-Stack User's Guide](https://software-dl.ti.com/simplelink/esd/simplelink_lowpower_f3_sdk/9.11.00.18/exports/docs/ble5stack/ble_user_guide/html/ble-stack-5.x/channel-selection-algorithm-number-two.html#channel-selection-algorithm-2) already provides a guide to disable CSA#2. However since the involved HCI events can be confusing for users, this demo showcases more detailed steps to handle the HCI events. The steps also apply to other LE feature set bits, moreover can be extended to handle other HCI commands.

The detailed HCI commands and events description can be found in the [Host Controller Interface (HCI)](https://software-dl.ti.com/simplelink/esd/simplelink_lowpower_f3_sdk/9.11.00.18/exports/docs/ble5stack/ble_user_guide/html/ble-stack-5.x/hci-cc23xx.html#host-controller-interface-hci) section in BLE5-Stack User's Guide.

The code required for this demo is inside ``app_connection.c`` and ``app_peripheral.c``. To change a feature set bit, firstly use ``HCI_LE_ReadLocalSupportedFeaturesCmd`` to read the currently supported local LE features:

```
bStatus_t Connection_start()
{
    ...
#ifdef DISABLE_CSA2
    // Get local supported LE features
    status = HCI_LE_ReadLocalSupportedFeaturesCmd();
    if(status != SUCCESS)
    {
        return(status);
    }
#endif // #ifdef DISABLE_CSA2

    return status;
}
```

All HCI commands are async commands which have corresponding event to inform the application for the status of execution. Refer to ``hci.h`` or the API document of ``HCI_LE_ReadLocalSupportedFeaturesCmd`` to acquire the corresponding event of this API:

```
/**
 * Read the LE locally supported features.
 *
 * @par Corresponding Events
 * @ref hciEvt_CmdComplete_t with cmdOpcode
 *      @ref HCI_LE_READ_LOCAL_SUPPORTED_FEATURES
 *
 * @return @ref HCI_SUCCESS
 */
extern hciStatus_t HCI_LE_ReadLocalSupportedFeaturesCmd( void );
```

From the API doc we learn the corresponding event of ``HCI_LE_ReadLocalSupportedFeaturesCmd`` is ``hciEvt_CmdComplete_t``. Therefore this event needs to be added to the event mask of ``connectionHciGAPHandler``, so the callback will be invoked when this event is returned from the BLE stack:

```
BLEAppUtil_EventHandler_t connectionHciGAPHandler =
{
    .handlerType    = BLEAPPUTIL_HCI_GAP_TYPE,
    .pEventHandler  = Connection_HciGAPEventHandler,
    .eventMask      = BLEAPPUTIL_HCI_COMMAND_STATUS_EVENT_CODE |
#ifdef DISABLE_CSA2
                      BLEAPPUTIL_HCI_COMMAND_COMPLETE_EVENT_CODE |
                      BLEAPPUTIL_HCI_VE_EVENT_CODE |
#endif // #ifdef DISABLE_CSA2
                      BLEAPPUTIL_HCI_LE_EVENT_CODE
};
```

Above event mask is used by ``BLEAppUtil_registerEventHandler`` to register application callback for masked HCI events. After the registration the ``BLEAPPUTIL_HCI_COMMAND_COMPLETE_EVENT_CODE`` event will be invoked when the ``HCI_LE_ReadLocalSupportedFeaturesCmd`` is executed by the BLE stack. Upon receiving of this event, ``HCI_EXT_SetLocalSupportedFeaturesCmd`` will be called to set the modified feature set.

```
void Connection_HciGAPEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{

    switch (event)
    {
#ifdef DISABLE_CSA2
        case BLEAPPUTIL_HCI_COMMAND_COMPLETE_EVENT_CODE:
        {
            hciEvt_CmdComplete_t *pCmdCompleteMsg = (hciEvt_CmdComplete_t *)pMsgData;
            switch ( pCmdCompleteMsg->cmdOpcode )
            {
              case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
              {
                  uint8_t featSet[8];
                  // get current feature set from received event (bits 1-9 of
                  // the returned data
                  memcpy(featSet, &pCmdCompleteMsg->pReturnParam[1], 8);
                  // clear the LL_FEATURE_CHAN_ALGO_2 feature bit
                  CLR_FEATURE_FLAG(featSet[1], LL_FEATURE_CHAN_ALGO_2);
                  // Update controller with modified features
                  HCI_EXT_SetLocalSupportedFeaturesCmd(featSet);

                  break;
              }

              default:
              {
                  break;
              }
              break;
            }
        }
        ...
```

Please note ``HCI_EXT_SetLocalSupportedFeaturesCmd`` is another HCI command, same precedure can be used to learn its corresponding event and cmdOpcode. A similar step is implemented to register for ``BLEAPPUTIL_HCI_VE_EVENT_CODE``, and corresponding code is added to ``Connection_HciGAPEventHandler`` in order to handle the event:

```
void Connection_HciGAPEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{

    switch (event)
    {
        ...
        case BLEAPPUTIL_HCI_VE_EVENT_CODE:
        {
            hciEvt_VSCmdComplete_t *pVSCmdCompleteMsg = (hciEvt_VSCmdComplete_t *)pMsgData;
            switch ( pVSCmdCompleteMsg->cmdOpcode )
            {
              case HCI_EXT_SET_LOCAL_SUPPORTED_FEATURES:
              {
                  // It is safer to move BLEAppUtil_advStart to here if feature set is modified
                  // to avoid connection is established before feature set is changed.
                  BLEAppUtil_advStart(peripheralAdvHandle_1, &advSetStartParamsSet_1);
                  MenuModule_printf(APP_MENU_CONN_EVENT, 0, "Feature set change - CSA#2 disabled");
                  break;
              }

              default:
              {
                  break;
              }
              break;
            }
        }
        ...
```

Last but not least, it is suggested to move the call of ``BLEAppUtil_advStart`` to the event handler to avoid the case of establishing a connection before the change of feature set. In a rare case the device will connect to a peer device before the feature set change is finished, this will misleading the peer device to use a wrong feature set which may lead to a connection failure or other unexpected behaviors.

Register connection event callback to get RSSI
==============================================

SimpleLink F3 SDK supports application layer callback for the connection events. The benefit of connection event callback can be found in [BLE5-Stack User's Guide](https://software-dl.ti.com/simplelink/esd/simplelink_lowpower_f3_sdk/9.11.00.18/exports/docs/ble5stack/ble_user_guide/html/ble-stack-5.x/gap-cc23xx.html#connection-event-callback). The user's guide also provides steps to register for the callback but some key steps are missing. This demo showcases the complete steps to register the connection event callback, and use it to get the RSSI from the connection as an example to users.

All the code required for this demo is inside ``app_connection.c``. Firstly, create an event handler struct which will be used to register the application layer callback of connection event:

```
// Event handler for connection event callback
BLEAppUtil_EventHandler_t connectionConnEvtHandler =
{
    .handlerType    = BLEAPPUTIL_CONN_NOTI_TYPE,
    .pEventHandler  = Connection_ConnectionEventHandler,
    .eventMask      = BLEAPPUTIL_CONN_NOTI_CONN_EVENT_ALL
};
```

In ``Connection_start``, call ``BLEAppUtil_registerEventHandler`` to register a callback with the newly created ``connectionConnEvtHandler``:

```
bStatus_t Connection_start()
{
    ...
#ifdef CONNECTION_EVENT_CALLBACK
    status = BLEAppUtil_registerEventHandler(&connectionConnEvtHandler);
    if(status != SUCCESS)
    {
        return(status);
    }
#endif // #ifdef CONNECTION_EVENT_CALLBACK
    ...

    return status;
}
```

In above struct, ``Connection_ConnectionEventHandler`` is the function name of the application callback, and the event mask is set to ``BLEAPPUTIL_CONN_NOTI_CONN_EVENT_ALL`` which means all connection event. This means ``Connection_ConnectionEventHandler`` will be called upon each BLE connection event.

We still need to call ``BLEAppUtil_registerConnNotifHandler`` in ``Connection_ConnEventHandler`` -> ``case BLEAPPUTIL_LINK_ESTABLISHED_EVENT`` to enable the connection event callback for the stack. After this call the application layer callback will be invoked at each connection event:

```
void Connection_ConnEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch(event)
    {
        case BLEAPPUTIL_LINK_ESTABLISHED_EVENT:
        {
            ...
#ifdef CONNECTION_EVENT_CALLBACK
            // Register connection event callback for all connection events
            BLEAppUtil_registerConnNotifHandler(gapEstMsg->connectionHandle, GAP_CB_CONN_EVENT_ALL);
#endif // #ifdef CONNECTION_EVENT_CALLBACK
            ...
```

Next, a definition of ``Connection_ConnectionEventHandler`` needs to be created to handle the connection event callback:

```
void Connection_ConnectionEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch(event)
    {
        case BLEAPPUTIL_CONN_NOTI_CONN_EVENT_ALL:
        {
            Gap_ConnEventRpt_t *connEvtReportMsg = (Gap_ConnEventRpt_t *)pMsgData;

            /*! Print the number of current connections */
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "CONN_EVT_CB: status 0x%x, handle 0x%x, lastRssi %d",
                              connEvtReportMsg->status, connEvtReportMsg->handle, connEvtReportMsg->lastRssi);

            break;
        }

        default:
        {
            break;
        }
    }
}
```

A message data of ``Gap_ConnEventRpt_t`` struct type will be passed into the callback, this struct contains information about current connection which could be used to implement application logic such as deciding the connection signal quality or RSSI reading:

```
/**
 * Report describing connection event Returned via a @ref pfnGapConnEvtCB_t.
 */
typedef struct
{
  GAP_ConnEvtStat_t     status;   //!< status of connection event
  uint16_t              handle;   //!< connection handle
  uint8_t               channel;  //!< BLE RF channel index (0-39)
  GAP_ConnEvtPhy_t      phy;      //!< PHY of connection event
  int8_t                lastRssi; //!< RSSI of last packet received
  /// Number of packets received for this connection event
  uint16_t              packets;
  /// Total number of CRC errors for the entire connection
  uint16_t              errors;
  /// Type of next BLE task
  GAP_ConnEvtTaskType_t nextTaskType;
  /// Time to next BLE task (in us). 0xFFFFFFFF if there is no next task.
  uint32_t              nextTaskTime;
  uint16_t              eventCounter; // event Counter
  uint32_t              timeStamp;    // timestamp (anchor point)
  GAP_CB_Event_e        eventType;    // event type of the connection report.
} Gap_ConnEventRpt_t;
```

In this demo the connection event callback is used to print the RSSI of the connection. The RSSI can help decide the link quality or the distance from the peer device, which is useful in localization applications.
