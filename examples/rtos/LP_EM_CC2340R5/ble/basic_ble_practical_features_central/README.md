
basic_ble_practical_features_central
====================================

This purpose of this example is to demonstrate implementation of frequently used BLE features. The project is based on examples from SimpleLink F3 SDK. User is considered to be already familiar with the SDK, for more information of SimpleLink SDK please refer to the SDK documentations.

Demo features
=============

This project demonstrates following features as a BLE central device:

- Enable continuous scan and use ADV report callback to filter scan results
- Service discovery

All features are under pre-defined symbols. Users can enable/disable the features from CCS project properties -> Build -> Tools -> Arm Compiler -> Predefined Symbols. The corresponding pre-defined symbols of the features are;

- ``CONTINUOUS_SCAN``
- ``GATT_SERVICE_DISCOVERY``

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


Continuous scan and ADV report callback
=======================================

The code under pre-defined symbol ``CONTINUOUS_SCAN`` demonstrates 2 frequently used features:

- Continuous scan
- Use ADV report callback to filter scan results

Basic_ble example from SimpleLink F3 SDK use a limited scan duration by default. Sometimes central device wants to start a continuous scan until a target peripehral device is found, in that case we can use continuous scan and use ADV report event callback to perform application layer filter for the scan results. This demo use a targeted peer address as the application layer filter, the central device will establish a connection when the peer device with target address is found.

All code changes required for this demo is inside ``app_central.c``.

## Start a continuous scan

Call ``BLEAppUtil_scanStart`` at the end of ``Central_start`` to start a scan. Use the scan parameters as below:

- Scan period = 0
- Scan duration = 0
- Max number of report = 0

Use 0 for scan period and scan duration will start a continuous scan. Set max number of report to 0 will revoke generation of advertising report list because we don't want a limit of the scan result in a continuous scan.

```
bStatus_t Central_start()
{
    ...
#ifdef CONTINUOUS_SCAN
    // Start continuous scan
    const BLEAppUtil_ScanStart_t centralScanStartParams =
    {
        /*! Zero for continuously scanning */
        .scanPeriod     = 0, /* Units of 1.28sec */

        /*! Scan Duration shall be greater than to scan interval,*/
        /*! Zero continuously scanning. */
        .scanDuration   = 0, /* Units of 10ms */

        /*! If non-zero, the list of advertising reports will be */
        /*! generated and come with @ref GAP_EVT_SCAN_DISABLED.  */
        .maxNumReport   = 0
    };
    status = BLEAppUtil_scanStart(&centralScanStartParams);
    // Print the status of the scan
    MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: ScanStart = "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                      status);
#endif // #ifndef CONTINUOUS_SCAN

    // Return status value
    return(status);
}
```

The duplicate filter needs to be disabled in ``centralScanInitParams`` for continuous scan, because duplicate filter is not supported in this scan mode.
```
const BLEAppUtil_ScanInit_t centralScanInitParams =
{
    ...
    /*! Opt SCAN_FLT_DUP_ENABLE | SCAN_FLT_DUP_DISABLE | SCAN_FLT_DUP_RESET */
    // Duplicate filter is not supported in continuous scan
    .fltDup                     = SCAN_FLT_DUP_DISABLE
};
```

## Register for ADV report event callback

Add ``BLEAPPUTIL_ADV_REPORT`` to the scan event handler ``centralScanHandler`` will enable the ADV report event callback. Each time an advertisement is scanned this event will be invoked after enabling this event mask.

```
BLEAppUtil_EventHandler_t centralScanHandler =
{
    ...
    .eventMask      = BLEAPPUTIL_SCAN_ENABLED |
#ifdef CONTINUOUS_SCAN
                      BLEAPPUTIL_ADV_REPORT |
#endif // #ifdef CONTINUOUS_SCAN
                      BLEAPPUTIL_SCAN_DISABLED
};
```

In ``Central_ScanEventHandler`` code is added under ``case BLEAPPUTIL_ADV_REPORT`` to handle the ADV reports. In this demo the application looks for a peer address ending with xx:xx:xx:xx:A2:D9 and establish a connection once it is found. User can modify the address to find a different device, or implement custom logic to find a target device.

```
void Central_ScanEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    BLEAppUtil_ScanEventData_t *scanMsg = (BLEAppUtil_ScanEventData_t *)pMsgData;

    switch (event)
    {
#ifdef CONTINUOUS_SCAN
        case BLEAPPUTIL_ADV_REPORT:
        {
            BLEAppUtil_GapScan_Evt_AdvRpt_t* adv_report = &scanMsg->pBuf->pAdvReport;
            uint8_t* p_mac = adv_report->addr;

            // This is just a demo searching for a certain address
            if (p_mac[0] == 0xD9 && p_mac[1] == 0xA2)
            {
                BLEAppUtil_scanStop();
                BLEAppUtil_ConnectParams_t connParams =
                {
                    .peerAddrType = adv_report->addrType,
                    .phys = DEFAULT_INIT_PHY,
                    .timeout = 5000
                };
                memcpy(connParams.pPeerAddress, p_mac, B_ADDR_LEN);
                uint32_t status = BLEAppUtil_connect(&connParams);
                MenuModule_printf(APP_MENU_SCAN_EVENT, 0, "BLEAPPUTIL_ADV_REPORT: %02x-%02x-%02x-%02x-%02x-%02x", 
                                  p_mac[5], p_mac[4], p_mac[3], p_mac[2], p_mac[1], p_mac[0]);
                MenuModule_printf(APP_MENU_CONN_EVENT, 0, "start connect: %d", status);
            }

            break;
        }
#endif // #ifdef CONTINUOUS_SCAN
```

Service discovery
=================

The code under pre-defined symbol ``GATT_SERVICE_DISCOVERY`` demonstrates how to perform a GATT service discovery from a central device. Service discovery is often used to find the services and characteristics resided on a peripheral device, so the central device knows the characteristic handles for GATT message exchange.

This demo implements 3 steps to complete the service discovery process:

- Request for a larger MTU
- Discover the service handle with a known UUID
- Discover all characteristics under the found service

The code required for this demo is inside ``app_connection.c`` and ``app_central.c``.

A larger MTU is requested from the central device before the service discovery. This is because the packets during service discovery may exceeds the default 27 byte MTU. The demo requests for the largest allowed MTU based on the ``MAX_PDU_SIZE`` setting in SysConfig. The largest allowed MTU is ``MAX_PDU_SIZE`` - 4 because of the L2CAP header size. Refer to [BLE5-Stack User's Guide](https://software-dl.ti.com/simplelink/esd/simplelink_lowpower_f3_sdk/9.11.00.18/exports/docs/ble5stack/ble_user_guide/html/ble-stack-5.x/gatt-cc23xx.html#maximum-transmission-unit-mtu) for more detail about MTU.

```
void Connection_ConnEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch(event)
    {
        case BLEAPPUTIL_LINK_ESTABLISHED_EVENT:
        {
            ...

#ifdef GATT_SERVICE_DISCOVERY
            bStatus_t status;
            attExchangeMTUReq_t req;
            // Request for the largest possible MTU size
            req.clientRxMTU = MAX_PDU_SIZE - L2CAP_HDR_SIZE;
            status = GATT_ExchangeMTU(gapEstMsg->connectionHandle, &req, BLEAppUtil_getSelfEntity());
            // Print the status of the GATT_ExchangeMTU call
            MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: Change max ATT_MTU = "
                              MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                              status);
#endif // #ifdef GATT_SERVICE_DISCOVERY

            break;
        }
```

The ``ATT_MTU_UPDATED_EVENT`` event will be invoked in ``GATT_EventHandler`` after a successful MTU exchange. This is where a service discovery is initiated. ``GATT_DiscPrimaryServiceByUUID`` is called with UUID = 0xFFF0 to find the service with this UUID. This UUID is defined for the Simple Profile in basic_ble example. An ATT_FIND_BY_TYPE_VALUE_REQ packet will be sent from the central side to obtain the handles of attributes that have a 16-bit UUID attribute type and attribute value.

```
/**
static void GATT_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  gattMsgEvent_t *gattMsg = ( gattMsgEvent_t * )pMsgData;
  switch ( gattMsg->method )
  {
      ...

      case ATT_MTU_UPDATED_EVENT:
      {
          MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "GATT status: ATT MTU update to %d",
                            gattMsg->msg.mtuEvt.MTU);
#ifdef GATT_SERVICE_DISCOVERY
          /* SimpleLink Service **/
          uint8_t uuid[2] = {0xF0, 0xFF};
          /** GATT Service Discovery **/
          bStatus_t bstatus = GATT_DiscPrimaryServiceByUUID(linkDB_NumActive() - 1, // only support 1 connection
                                                            uuid,
                                                            2,
                                                            BLEAppUtil_getSelfEntity());
          MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: GATT_DiscPrimaryServiceByUUID = "
                            MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                            bstatus);
#endif // #ifdef GATT_SERVICE_DISCOVERY
      }
      break;
```

Upon receiving the response(ATT_FIND_BY_TYPE_VALUE_RSP) from peripheral device,  ``ATT_FIND_BY_TYPE_VALUE_RSP`` event will be invoked in ``GATT_EventHandler``. To receive this event in application layer, we need to register the ``BLEAPPUTIL_ATT_FIND_BY_TYPE_VALUE_RSP`` event in advance inside ``dataGATTHandler``. As shown in below code snippet we also register for ``BLEAPPUTIL_ATT_READ_BY_TYPE_RSP`` since it is used for subsequent characteristic discovery.

```
BLEAppUtil_EventHandler_t dataGATTHandler =
{
    .handlerType    = BLEAPPUTIL_GATT_TYPE,
    .pEventHandler  = GATT_EventHandler,
    .eventMask      = BLEAPPUTIL_ATT_FLOW_CTRL_VIOLATED_EVENT |
#ifdef GATT_SERVICE_DISCOVERY
                      BLEAPPUTIL_ATT_FIND_BY_TYPE_VALUE_RSP |
                      BLEAPPUTIL_ATT_READ_BY_TYPE_RSP |
#endif // #ifdef GATT_SERVICE_DISCOVERY
                      BLEAPPUTIL_ATT_MTU_UPDATED_EVENT
};
```

The ``case ATT_FIND_BY_TYPE_VALUE_RSP`` event will be invoked twice during a service discovery. First when the target UUID is found the central device will receive a ATT_FIND_BY_TYPE_VALUE_RSP with ``att.numInfo`` equal to found attribute, then the central will send another ATT_FIND_BY_TYPE_VALUE_REQ to see if there is other qualified attributes, if no more is found, a ATT_FIND_BY_TYPE_VALUE_RSP will be received with ``att.numInfo`` = 0 which means the end of the service discovery. We will record the attribute handle in the first one and start discoverying all characteristics in the second one by calling ``GATT_DiscAllChars``.

```
static void GATT_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  gattMsgEvent_t *gattMsg = ( gattMsgEvent_t * )pMsgData;
  switch ( gattMsg->method )
  {
    ...

#ifdef GATT_SERVICE_DISCOVERY
    case ATT_FIND_BY_TYPE_VALUE_RSP:
    {
        static uint16_t lastStart;
        static uint16_t lastEnd;
        static uint8_t gattServiceFound = 0;
        attFindByTypeValueRsp_t att = (attFindByTypeValueRsp_t)gattMsg->msg.findByTypeValueRsp;
        // This event will be invoked twice, one with service found, the other with Attribute not found
        // Set gattServiceFound flag to 1 in first one, start characteristic discovery in second one
        // att.numInfo > 0 means target service is found
        if (att.numInfo > 0)
        {
            for (uint8_t i = 0; i < att.numInfo; i++)
            {
                uint16_t attHandle = BUILD_UINT16(att.pHandlesInfo[i * 4], att.pHandlesInfo[i * 4 + 1]);
                uint16_t endHandle = BUILD_UINT16(att.pHandlesInfo[i * 4 + 2], att.pHandlesInfo[i * 4 + 3]);
                lastStart = attHandle;
                lastEnd = endHandle;
                // This event will be invoked twice, one with service found, the other with Attribute not found
                // Set gattServiceFound flag to 1 in first one, start characteristic discovery in second one
                gattServiceFound = 1;
                MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "GATT service found: handle 0x%x", attHandle);
            }
        }
        // att.numInfo = 0 mean attribute not found, that's the end of service discovery
        else
        {
            // Target service is found in previous event
            if(gattServiceFound == 1)
            {
                bStatus_t bstatus = GATT_DiscAllChars(gattMsg->connHandle,
                                                      lastStart,
                                                      lastEnd,
                                                      BLEAppUtil_getSelfEntity());
                MenuModule_printf(APP_MENU_GENERAL_STATUS_LINE, 0, "Call Status: GATT_DiscAllChars = "
                                  MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_RED "%d" MENU_MODULE_COLOR_RESET,
                                  bstatus);
            }
            // Target service not found
            else
            {
                MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "GATT service not found");
            }
        }
    }
    break;

    ...
```

The subsequent event for ``GATT_DiscAllChars`` is ``ATT_READ_BY_TYPE_RSP``, central device will receive multiple events if there are more than one characteristic in the attribute handle range. In this demo we print the characteristic handle, UUID and properties in the event callback.

```
static void GATT_EventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
  gattMsgEvent_t *gattMsg = ( gattMsgEvent_t * )pMsgData;
  switch ( gattMsg->method )
  {
    ...
    
    case ATT_READ_BY_TYPE_RSP:
    {
        attReadByTypeRsp_t att = (attReadByTypeRsp_t)gattMsg->msg.readByTypeRsp;

        MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE + display_index, 0, "ATT_READ_BY_TYPE_RSP len: %d numPairs: %d", 
                          att.len, att.numPairs);
        display_index++;

        for (uint8_t i = 0; i < att.numPairs; i++)
        {
            /* This is the Attribute Handle */
            uint16_t attHandle = BUILD_UINT16(att.pDataList[i * att.len + 0], att.pDataList[i * att.len + 1]);
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE + display_index, 0,"ATT Handle: 0x%x", attHandle);
            display_index++;

            /* This is the index to the beginning of the value data AFTER the attribute handle */
            uint8_t index = i * att.len + 2;

            /* The characteristic value data */
            uint8_t data1 = att.pDataList[index];     // Bit field of characteristic properties

            uint8_t data2 = att.pDataList[index + 1]; // Second Byte of Characteristic Value Handle
            uint8_t data3 = att.pDataList[index + 2]; // First Byte of Characteristic Value Handle

            /* If we were using 128-bit Bluetooth UUIDs, then we would have 16 of these instead of 2. */
            uint8_t data4 = att.pDataList[index + 3]; // Second byte of UUID
            uint8_t data5 = att.pDataList[index + 4]; // First byte of UUID

            /* Since we are in little endian, we have to reverse the order to make the values make sense. */
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE + display_index, 0,"UUID: 0x%x%x Val Handle: 0x%x%x Properties: 0x%x", 
                              data5, data4, data3, data2, data1);
            display_index++;
        }
    }
    break;

    ...
```
