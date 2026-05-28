# Zigbee Hub — ESP32 + CC2652P7 Project

## Overview

A custom Zigbee hub built on commodity embedded hardware, combining the ESP32-N16R2 as the application host and the CC2652P7 as the dedicated Zigbee radio coprocessor. The hub exposes a structured framework API that allows upper-layer applications to manage Zigbee networks, devices, and events without directly handling protocol internals.

---

## Hardware Design

| Component | Role |
|---|---|
| ESP32-N16R2 | ZNP Host — runs the Zigbee framework, application logic, and network management |
| CC2652P7 | ZNP Coprocessor — handles all Zigbee radio operations in Coordinator mode |

The two chips communicate over **UART**, using the Z-Stack Monitor and Test (MT) protocol (ZNP interface). The CC2652P7 operates purely as a radio coprocessor; all higher-level logic lives on the ESP32.

---

## Software Architecture

### Communication Layer

- ESP32 ↔ CC2652P7 via **UART (ZNP protocol)**
- Handles framing, command dispatch, and response correlation
- Abstracts hardware differences from the framework layer above

---

### Zigbee Framework (ESP32)

The core of the project. Structured as an event-driven state machine that manages the full lifecycle of the Zigbee Coordinator and the network it controls.

#### State Machine — Network Lifecycle

Manages transitions across the following states:

```
UNINITIALIZED
    └─► ZNP_INIT          — Reset and probe the CC2652P7 coprocessor
        └─► NETWORK_CHECK  — Detect existing network or decide to form a new one
            ├─► FORMING    — Create a new Zigbee network (PAN ID, channel, key)
            ├─► RECONFIGURING — Apply changed network parameters
            └─► READY      — Network is up; normal operation
                └─► FW_UPDATE — OTA / firmware update flow for the coprocessor
```

#### Responsibilities of the Framework

- **ZNP coprocessor lifecycle** — reset, initialization, re-sync on error
- **Network management** — form, join, configure, and maintain the Zigbee network
- **Inbound data processing** — parse and route incoming ZNP messages (attribute reports, commands, status events)
- **Command processing** — accept commands from the upper layer and serialize them to ZNP requests
- **Device manager** — maintain a registry of joined devices, their endpoints, clusters, and last-known state
- **Notification/subscription bus** — deliver asynchronous events to registered subscribers

---

### API Surface

The framework exposes two primary interaction surfaces to upper-layer applications:

#### Subscribe API

Register callbacks to receive asynchronous notifications from the framework:

```c
// Example subscriber registration
zb_subscribe(ZB_EVENT_DEVICE_JOINED,   on_device_joined);
zb_subscribe(ZB_EVENT_ATTR_REPORT,     on_attribute_report);
zb_subscribe(ZB_EVENT_DEVICE_LEFT,     on_device_left);
zb_subscribe(ZB_EVENT_NETWORK_READY,   on_network_ready);
```

Event categories include:
- Network lifecycle events (ready, lost, reconfigured)
- Device join / leave / announce
- Attribute reports and command responses
- Coprocessor status and error events

#### Command API

Send commands down to the framework, which translates them to ZNP requests:

```c
// Example command calls
zb_cmd_permit_join(60);                        // open network for 60 s
zb_cmd_send_zcl(dev_id, ep, cluster, cmd, payload, len);
zb_cmd_read_attr(dev_id, ep, cluster, attr_id);
zb_cmd_change_channel(15);
zb_cmd_firmware_update(fw_blob, fw_len);
```

---

### Device Manager

Maintains a runtime registry of all devices in the Zigbee network:

- **Join / leave tracking** — updates registry on network events
- **Endpoint & cluster map** — stores each device's supported endpoints and ZCL clusters
- **State cache** — last-known attribute values per device
- **Notification fanout** — on any state change, notifies all registered subscribers with a structured event payload

---

## Firmware Update Flow

A dedicated state in the framework state machine handles coprocessor firmware updates:

1. Upper layer calls `zb_cmd_firmware_update(blob, len)`
2. Framework transitions to `FW_UPDATE` state — pauses normal operation
3. Firmware is streamed to CC2652P7 over UART using the bootloader protocol
4. On success, framework re-initializes the coprocessor and returns to `READY`
5. On failure, framework attempts rollback and surfaces error via subscriber notification

---

## Key Design Principles

- **Separation of concerns** — radio protocol on CC2652P7, all logic on ESP32
- **Event-driven** — no polling; all data flow is interrupt/callback-driven
- **Resilient lifecycle** — the state machine recovers from coprocessor resets and network disruptions automatically
- **Extensible API** — upper-layer code (MQTT bridge, REST API, local automations) depends only on the subscribe/command interface, not on ZNP internals

---

## Potential Upper-Layer Integrations

- **MQTT bridge** — publish device events to a broker; subscribe to command topics
- **Matter bridge** — expose Zigbee devices as Matter endpoints over Wi-Fi
- **Local automation engine** — rule-based triggers using the subscribe API
- **REST / WebSocket API** — for integration with home automation dashboards

---

## Open Questions / Next Steps

- [ ] Define UART baud rate and flow control scheme for ZNP communication
- [ ] Decide on persistent storage format for device registry (NVS, LittleFS, etc.)
- [ ] Establish error recovery policy for ZNP desyncs mid-operation
- [ ] Define firmware update packaging format and integrity verification
- [ ] Evaluate FreeRTOS task partitioning for UART driver, state machine, and API layers
- [ ] Determine over-the-air update strategy for the ESP32 itself (OTA via Wi-Fi)