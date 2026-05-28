# Zigbee Hub (ESP32 + CC2652P7) — Functional Specification Document (FSD)

**Version:** 1.4  
**Date:** 2026-05-29  
**Status:** Draft  
**Changed (v1.1):** Storage layer replaced — NVS → LittleFS (JSON config + nanopb device files)  
**Changed (v1.2):** Device Abstraction Layer added — typed Capability API hides ZCL internals from upper layer  
**Changed (v1.3):** Physical user-button input (GPIO34) with multi-press detection (single / double / long)  
**Changed (v1.4):** Network status/config command API (FR-11); extended ZHA cluster set — IAS zone, pressure, occupancy, metering, electrical measurement, analog/binary I/O (FR-12)

---

## 1. System Overview

### 1.1 Purpose

The Zigbee Hub is a custom embedded Zigbee coordinator built on two commodity chips: an
ESP32-N16R2 as the application host and a CC2652P7 as the dedicated Zigbee radio
coprocessor. The hub exposes a structured C framework API — a Subscribe interface for
asynchronous events and a Command interface for outbound actions — so that upper-layer
applications (MQTT bridge, Matter bridge, local automation engine, REST/WebSocket API)
can interact with a Zigbee network without handling protocol internals.

### 1.2 Problem Statement

Generic Zigbee solutions either lock users into closed ecosystems (commercial hubs) or
require deep protocol knowledge to operate (raw Z-Stack integration). This project
provides a clean abstraction layer: the CC2652P7 owns all radio operations via the
Z-Stack Monitor and Test (ZNP) protocol, and the ESP32 presents a stable, event-driven
API surface that separates radio concerns from application logic.

### 1.3 Users / Stakeholders

| Role | Interaction |
|------|-------------|
| Embedded developer | Implements upper-layer apps on top of the framework Subscribe/Command API |
| Home-automation integrator | Connects the hub to MQTT, Home Assistant, or Matter ecosystems |
| End user | Indirectly benefits through reliable device control and event reporting |
| Firmware engineer | Maintains and updates ESP32 and CC2652P7 firmware |

### 1.4 Goals

- Reliable bidirectional communication between ESP32 and CC2652P7 via UART/ZNP.
- Event-driven Zigbee Coordinator lifecycle with automatic error recovery.
- Clean, stable API surface insulating upper-layer code from ZNP internals.
- Persistent device registry (joined devices, endpoints, clusters, state cache).
- Safe firmware update path for the CC2652P7 coprocessor.

### 1.5 Non-Goals

- The framework does not implement any specific upper-layer integration (MQTT, Matter,
  REST) — these are application-layer concerns outside Phase 1–3 scope.
- The hub does not act as a Zigbee Router or End Device; Coordinator mode only.
- No pairing UI or provisioning flow beyond the existing network-formation logic.

### 1.6 High-Level System Flow

```
Upper-Layer App
      │  zb_subscribe(event, cb)
      │  zb_cmd_*(...)
      ▼
┌─────────────────────────────────────────┐
│          Zigbee Framework (ESP32)        │
│  ┌──────────────┐  ┌──────────────────┐  │
│  │ State Machine│  │  Device Manager  │  │
│  └──────┬───────┘  └──────────────────┘  │
│         │  ZNP Commands / Events          │
│  ┌──────▼───────────────────────────┐    │
│  │      ZNP Communication Layer     │    │
│  │   (framing · dispatch · corr.)   │    │
│  └──────────────────┬───────────────┘    │
└─────────────────────┼─────────────────────┘
                       │ UART (ZNP MT protocol)
                  ┌────▼────┐
                  │CC2652P7 │  Zigbee Radio
                  │ (ZNP)   │  Coordinator
                  └─────────┘
                       │ 2.4 GHz RF
               Zigbee End Devices / Routers
```

---

## 2. System Architecture

### 2.1 Logical Architecture

The system is divided into five logical layers:

| Layer | Responsibility |
|-------|---------------|
| **ZNP Transport** | UART framing, synchronous command dispatch (SREQ/SRSP), async event reception (AREQ), re-sync on error |
| **Framework Core** | Event-driven state machine managing coprocessor and network lifecycle |
| **Device Manager** | Runtime registry of joined devices, endpoint/cluster maps, state cache; owns cluster schema table |
| **Device Abstraction Layer** | Maps ZCL internals (endpoints, clusters, attributes) to opaque `zb_cap_id_t` handles and typed cluster functions; upper layer never sees raw ZCL |
| **API Surface** | Capability API (`zb_cap_*`) + Subscribe API (`zb_cap_subscribe`) consumed by upper-layer apps |

Data flow (inbound path):
```
CC2652P7 UART AREQ → ZNP parser → state machine dispatch → device manager update
  → abstraction layer: ZCL attr report → zb_cap_event_t (ieee + cap_id + typed value)
  → subscriber callbacks (upper layer sees no ZCL)
```

Data flow (outbound path):
```
upper-layer app  zb_cap_onoff_set(ieee, cap_id, true)
  → abstraction layer resolves cap_id → ep + cluster + attr
  → command queue → ZNP transport SREQ → CC2652P7
```

### 2.2 Hardware / Platform Architecture

| Component | Part | Role |
|-----------|------|------|
| Application host | ESP32-N16R2 | Runs FreeRTOS, framework, device manager, API |
| Zigbee radio coprocessor | CC2652P7 | Runs Z-Stack firmware in Coordinator mode |
| Interconnect | UART | ZNP MT protocol at configurable baud rate |
| Flash (ESP32) | 16 MB | OTA partitions + LittleFS `storage` partition (config + device registry) |
| Flash (CC2652P7) | Internal | Z-Stack firmware; updated via UART bootloader |

**Connectivity:**

| Interface | Usage |
|-----------|-------|
| UART (ESP32 ↔ CC2652P7) | ZNP protocol, coprocessor firmware update |
| Wi-Fi (ESP32) | ESP32 OTA, MQTT/REST upper layers (assumed, Phase 4) |
| 2.4 GHz RF (CC2652P7) | Zigbee coordinator radio |

**Power:**
- ESP32-N16R2 and CC2652P7 operate at 3.3 V (assumed, regulated from USB or external supply).
- CC2652P7 reset line is driven from an ESP32 GPIO to allow hard reset during init and firmware update.
- A single momentary push-button is wired to ESP32 GPIO34 (input-only pin, external pull-up to 3.3 V; pressed = LOW) for user actions such as opening the network for join.

### 2.3 Software Architecture

#### 2.3.1 FreeRTOS Task Decomposition (assumed)

| Task | Priority | Responsibility |
|------|----------|---------------|
| `znp_uart_rx_task` | High | Receive bytes from UART ISR ring buffer, assemble and validate ZNP frames |
| `znp_dispatch_task` | High | Route assembled frames to SRSP correlator or AREQ event queue |
| `framework_task` | Medium | Run state machine, process AREQ events, drive transitions |
| `device_mgr_task` | Medium | Process device join/leave/attribute events, update registry |
| `api_notify_task` | Medium | Fan out subscriber callbacks from event queue |
| `cmd_queue_task` | Medium | Dequeue upper-layer commands, serialize to SREQ, await SRSP |

#### 2.3.2 Boot Sequence

```
1. ESP32 boots (FreeRTOS scheduler starts)
2. NVS flash initialized (system use only — Wi-Fi, PHY calibration)
3. zb_storage_init(): mount LittleFS partition "storage" at /zb; create /zb/devices/
4. ZNP transport layer initialized (UART, ring buffers, queues)
5. Framework state machine enters UNINITIALIZED
6. Hard-reset CC2652P7 via GPIO
7. → ZNP_INIT: probe coprocessor, verify Z-Stack version
8. → NETWORK_CHECK: load /zb/config.json; query CC2652P7 NV to detect existing network
9a. Network found and config matches → startup coordinator → READY
9b. No network → FORMING (write config to CC2652P7 NV, start coordinator, save config.json)
9c. Config changed → RECONFIGURING (clear CC2652P7 NV, re-form, update config.json)
10. READY: zb_storage_device_load_all() restores device registry; subscriber callbacks enabled
```

#### 2.3.3 Persistence / Storage

| Data | Storage | Path / Format |
|------|---------|--------------|
| Network configuration (PAN ID, channel, key, baud) | LittleFS | `/zb/config.json` — cJSON key-value object |
| Device registry (one file per device) | LittleFS | `/zb/devices/<16-hex-ieee>.pb` — nanopb-encoded `ZbDeviceRecord` |
| CC2652P7 firmware blob | Supplied externally at update time | Binary (Z-Stack image) |
| ESP-IDF system data (Wi-Fi, PHY cal.) | ESP32 NVS (system partition, 24 KB) | Key-value; not used by application code |

LittleFS is mounted at `/zb` from the `storage` partition (spiffs subtype, ~11 MB on
16 MB flash). The `zb_storage` component manages mount, directory creation, JSON
encode/decode, and nanopb encode/decode. The `joltwallet/littlefs` and `nanopb/nanopb`
ESP-IDF managed components are declared in `components/zb_storage/idf_component.yml`.

#### 2.3.4 Update Model

- **ESP32 firmware:** OTA via Wi-Fi, dual A/B partition scheme (assumed, Phase 4).
- **CC2652P7 firmware:** Streamed over UART using the CC2652P7 ROM bootloader protocol,
  triggered by `zb_cmd_firmware_update()`. Full update flow managed by the `FW_UPDATE`
  state in the framework state machine.

---

## 3. Implementation Phases

### 3.1 Phase 1 — ZNP Communication Foundation

**Scope:**
- UART driver: ring-buffer ISR receive, configurable baud rate, optional hardware flow control.
- ZNP framing: SOF detection, length decode, FCS verification, frame assembly/disassembly.
- Synchronous command path: SREQ → SRSP with per-command timeout and error return.
- Asynchronous event path: AREQ frames enqueued for framework consumption.
- CC2652P7 reset sequence: GPIO-driven hard reset, reset-indication (SYS_RESET_IND) detection.
- ZNP re-sync: detect frame desync (unexpected bytes, FCS errors), drain and re-sync.

**Deliverables:**
- `znp_transport.c/.h` — transport layer module.
- `znp_mt_protocol.c/.h` — MT frame definitions, command IDs, payload helpers.
- Integration test: send `ZB_APP_REGISTER_REQUEST` SREQ, receive SRSP.
- Loopback / offline unit tests for framing logic.

**Exit Criteria:**
- SREQ sends and SRSP received for at least one known command (`SYS_PING`).
- AREQ (`SYS_RESET_IND`) correctly received and enqueued after a hard reset.
- FCS error injection triggers re-sync without hang.
- Frame assembly/disassembly unit tests pass (100 % coverage of framing logic).

**Dependencies:**
- CC2652P7 programmed with valid Z-Stack Coordinator firmware.
- ESP-IDF UART driver available.

---

### 3.2 Phase 2 — Zigbee Framework Core (State Machine + Network)

**Scope:**
- Full state machine implementation: UNINITIALIZED, ZNP_INIT, NETWORK_CHECK, FORMING,
  RECONFIGURING, READY, FW_UPDATE.
- ZNP_INIT: coprocessor version check, startup synchronization.
- NETWORK_CHECK: query NV for existing network state; branch to FORMING or READY.
- FORMING: configure PAN ID, channel, network key; start Coordinator via `ZDO_STARTUP_FROM_APP`.
- RECONFIGURING: triggered by changed network parameters; clear NV, re-form.
- READY: steady-state operation; AREQ events dispatched to device manager and subscribers.
- Automatic recovery: on unexpected `SYS_RESET_IND` in READY, transition back through
  ZNP_INIT without upper-layer disruption.

**Deliverables:**
- `zb_framework.c/.h` — state machine, lifecycle management.
- `zb_network.c/.h` — network formation, coordinator start, permit-join.
- Integration test: full boot to READY on physical hardware.
- Integration test: network forms with a test end device joining.

**Exit Criteria:**
- State machine reaches READY state in under 10 seconds from cold boot.
- Network formation succeeds with correct PAN ID and channel.
- A Zigbee end device can join the network.
- Coprocessor hard reset during READY triggers automatic re-initialization to READY within 15 seconds.
- All state transitions logged at DEBUG level.

**Dependencies:**
- Phase 1 complete.
- `zb_storage` component available; LittleFS partition present in `partitions.csv`.

---

### 3.3 Phase 3 — Device Manager & API Layer

**Scope:**
- Device Manager: join/leave tracking, endpoint discovery, cluster map storage.
- State cache: maintain last-known ZCL attribute values per device/endpoint/cluster.
- Device registry persistence: on join, call `zb_storage_device_save()`; on leave, call `zb_storage_device_delete()`; on READY entry, call `zb_storage_device_load_all()` to restore registry from LittleFS protobuf files.
- **Device Abstraction Layer:** after endpoint/cluster discovery on join, device manager builds a `zb_cap_id_t` list for the device and registers matched cluster schemas; upper-layer code uses only `ieee_addr` + `zb_cap_id_t`, never raw endpoints or clusters.
- Subscribe API: `zb_subscribe()` / `zb_unsubscribe()` with event-type filtering.
- Command API: `zb_cmd_permit_join()`, `zb_cmd_send_zcl()`, `zb_cmd_read_attr()`,
  `zb_cmd_change_channel()`.
- Notification fanout: deliver structured `zb_event_t` payloads to all registered callbacks.
- Command queue: buffer commands during non-READY states; drain on entering READY.

**Deliverables:**
- `zb_device_mgr.c/.h` — device registry, state cache, cluster schema table, capability list per device.
- `zb_cap.c/.h` — Device Abstraction Layer: `zb_cap_id_t`, `zb_cap_event_t`, typed cluster functions, schema registration.
- `zb_subscribe.c/.h` — event bus, subscriber table.
- Built-in schemas for: OnOff (0x0006), LevelControl (0x0008), ColorControl (0x0300), TemperatureMeasurement (0x0402), RelativeHumidity (0x0405), IlluminanceMeasurement (0x0400).
- Upper-layer example: test app using only `ieee_addr` + `zb_cap_id_t`; no raw ZCL.
- Registry persistence test: registry survives ESP32 reboot; caps re-enumerated from stored endpoints.

**Exit Criteria:**
- Device join event delivered to subscriber within 500 ms of join.
- ZCL attribute report delivered to subscriber within 200 ms of AREQ receipt.
- Device registry (3 devices minimum) persists across power cycle.
- `zb_cmd_permit_join(60)` opens network; devices can join within the 60 s window.
- `zb_cmd_read_attr()` returns attribute value via subscriber callback.
- Command queue holds commands issued in FORMING state; all execute after reaching READY.
- `zb_dev_get_caps()` returns non-empty list for a joined bulb with OnOff + LevelControl clusters.
- Upper-layer test app emits no direct ZCL cluster IDs or endpoint numbers — only `ieee_addr` + `zb_cap_id_t`.

**Dependencies:**
- Phase 2 complete.
- Persistent storage driver available.

---

### 3.4 Phase 4 — Firmware Update & First Upper-Layer Integration

**Scope:**
- CC2652P7 firmware update: `FW_UPDATE` state machine, UART bootloader streaming,
  integrity check, success re-init, failure rollback.
- ESP32 OTA via Wi-Fi: dual A/B partition update triggered via HTTP endpoint (assumed).
- MQTT bridge: first reference upper-layer integration — publish device events to a
  configurable broker, subscribe to command topics.
- Firmware version reporting: both ESP32 and CC2652P7 versions queryable via API.

**Deliverables:**
- `zb_fw_update.c/.h` — CC2652P7 firmware update logic.
- `mqtt_bridge.c/.h` — reference MQTT upper-layer (optional, may be separate repo).
- End-to-end test: CC2652P7 firmware updated successfully and coordinator resumes.
- Rollback test: corrupted firmware blob → rollback → coordinator still operational.

**Exit Criteria:**
- Valid CC2652P7 firmware blob streamed, installed, and coordinator re-initialized.
- Invalid or truncated blob rejected; hub recovers to READY without intervention.
- ESP32 OTA update completes via Wi-Fi without losing device registry.
- MQTT bridge publishes at least one device join event and attribute report to broker.

**Dependencies:**
- Phase 3 complete.
- Wi-Fi credentials provisioned (assumed pre-configured in NVS system partition for Phase 4).

---

## 4. Functional Requirements

### 4.1 Functional Requirements

#### FR-1: ZNP Communication Layer

- **FR-1.1** [Must]: The ZNP transport shall initialize the UART peripheral with a
  configurable baud rate (default: 115200 bps).
- **FR-1.2** [Must]: The transport shall frame outbound MT commands (SOF, length,
  command type, command ID, payload, FCS) per the Z-Stack Monitor and Test protocol.
- **FR-1.3** [Must]: The transport shall assemble inbound MT frames from the UART byte
  stream, verify FCS, and reject malformed frames.
- **FR-1.4** [Must]: The transport shall correlate SRSP frames with the outstanding SREQ
  using command type and command ID, with a configurable per-command timeout
  (default: 3000 ms).
- **FR-1.5** [Must]: The transport shall enqueue AREQ frames on a dedicated queue for
  asynchronous consumption by the framework.
- **FR-1.6** [Must]: The transport shall drive the CC2652P7 reset GPIO to perform a hard
  reset and detect the resulting `SYS_RESET_IND` AREQ within 3 seconds.
- **FR-1.7** [Must]: On detection of consecutive FCS errors or UART overrun, the
  transport shall drain the receive buffer and attempt to re-sync (SOF hunt).
- **FR-1.8** [Should]: The transport shall expose an error counter (FCS errors, timeouts,
  overruns) queryable by the framework for diagnostic logging.

#### FR-2: Framework State Machine

- **FR-2.1** [Must]: The framework state machine shall transition from UNINITIALIZED to
  ZNP_INIT on first call to `zb_framework_start()`.
- **FR-2.2** [Must]: In ZNP_INIT, the framework shall reset the coprocessor, send
  `SYS_PING`, and verify the response before proceeding.
- **FR-2.3** [Must]: In NETWORK_CHECK, the framework shall query NV items on the
  coprocessor to determine whether a valid network configuration exists.
- **FR-2.4** [Must]: If no network exists, the framework shall enter FORMING, configure
  PAN ID, channel, and network key, then call `ZDO_STARTUP_FROM_APP` to start the
  coordinator.
- **FR-2.5** [Must]: On successful coordinator start, the framework shall transition to
  READY and emit a `ZB_EVENT_NETWORK_READY` notification.
- **FR-2.6** [Must]: If stored network parameters differ from current NV, the framework
  shall enter RECONFIGURING, clear the existing network NV, and re-form the network.
- **FR-2.7** [Must]: In READY state, the framework shall receive and dispatch all AREQ
  events to the device manager and subscriber notification bus.
- **FR-2.8** [Must]: On receiving an unexpected `SYS_RESET_IND` in READY state, the
  framework shall automatically re-enter ZNP_INIT and restore the network without
  upper-layer intervention.
- **FR-2.9** [Should]: All state transitions shall be logged at DEBUG level with
  the source state, target state, and trigger event.
- **FR-2.10** [Must]: The framework shall expose the current state as a queryable value.

#### FR-3: Network Management

- **FR-3.1** [Must]: The framework shall configure the coordinator with a user-supplied
  PAN ID, persisted in `/zb/config.json` after first formation.
- **FR-3.2** [Must]: The framework shall configure the coordinator operating channel
  (2.4 GHz, channels 11–26), persisted in `/zb/config.json`.
- **FR-3.3** [Must]: The framework shall provision the network security key on coordinator
  initialization.
- **FR-3.4** [Must]: `zb_cmd_permit_join(duration_s)` shall instruct the coordinator to
  accept new device joins for the specified duration (0 = close, 0xFF = open indefinitely).
- **FR-3.5** [Should]: `zb_cmd_change_channel(channel)` shall request a channel change;
  the framework shall enter RECONFIGURING and re-form on the new channel.

#### FR-4: Device Manager

- **FR-4.1** [Must]: The device manager shall add a device record to the registry upon
  receiving a `ZDO_TC_DEV_IND` or `ZDO_END_DEVICE_ANNCE_IND` AREQ.
- **FR-4.2** [Must]: The device manager shall remove the device record on receiving a
  `ZDO_LEAVE_IND` AREQ.
- **FR-4.3** [Must]: The device manager shall store each device's IEEE address, network
  address, supported endpoints, and supported ZCL clusters per endpoint.
- **FR-4.4** [Must]: The device manager shall update the state cache for a device on
  every received ZCL attribute report.
- **FR-4.5** [Must]: Each device record shall be serialized as a nanopb-encoded
  `ZbDeviceRecord` protobuf file (`/zb/devices/<16-hex-ieee>.pb`) and restored from
  LittleFS on every READY state entry via `zb_storage_device_load_all()`.
- **FR-4.6** [Should]: The device manager shall expose a query function
  `zb_dev_get(ieee_addr)` returning a pointer to the device record or NULL.
- **FR-4.7** [May]: The device manager shall support iterating all registered devices
  via `zb_dev_foreach(callback, ctx)`.

#### FR-5: Subscribe API

- **FR-5.1** [Must]: The framework shall provide `zb_subscribe(event_type, callback)`
  to register a callback for a specific event type.
- **FR-5.2** [Must]: The framework shall provide `zb_unsubscribe(event_type, callback)`
  to deregister a previously registered callback.
- **FR-5.3** [Must]: The notification bus shall deliver `ZB_EVENT_NETWORK_READY` and
  `ZB_EVENT_NETWORK_LOST` events on state machine transitions.
- **FR-5.4** [Must]: The notification bus shall deliver `ZB_EVENT_DEVICE_JOINED` and
  `ZB_EVENT_DEVICE_LEFT` events on device registry changes.
- **FR-5.5** [Must]: The notification bus shall deliver `ZB_EVENT_ATTR_REPORT` events
  carrying device ID, endpoint, cluster ID, attribute ID, type, and value.
- **FR-5.6** [Must]: The notification bus shall deliver `ZB_EVENT_ZNP_ERROR` events
  on coprocessor communication failures.
- **FR-5.7** [Should]: Multiple subscribers shall be supported per event type
  (fan-out delivery).
- **FR-5.8** [Should]: Subscriber callbacks shall be invoked from a dedicated notify
  task, not from the UART ISR or ZNP dispatch path.

#### FR-6: Command API

- **FR-6.1** [Must]: `zb_cmd_permit_join(duration_s)` shall send `ZDO_MGMT_PERMIT_JOIN_REQ`
  to the coordinator.
- **FR-6.2** [Must]: `zb_cmd_send_zcl(dev_id, ep, cluster, cmd, payload, len)` shall
  construct and send a ZCL command frame to the target device endpoint.
- **FR-6.3** [Must]: `zb_cmd_read_attr(dev_id, ep, cluster, attr_id)` shall send a ZCL
  Read Attribute Request; the response shall be delivered via `ZB_EVENT_ATTR_REPORT`.
- **FR-6.4** [Should]: `zb_cmd_change_channel(channel)` shall trigger a RECONFIGURING
  transition in the state machine.
- **FR-6.5** [Must]: `zb_cmd_firmware_update(fw_blob, fw_len)` shall initiate the
  CC2652P7 firmware update flow.
- **FR-6.6** [Should]: Commands issued when the framework is not in READY state shall
  be queued and executed in order upon the next READY transition.
- **FR-6.7** [Must]: All Command API functions shall return an error code if the
  framework is in FW_UPDATE state (commands not queued during update).

#### FR-7: CC2652P7 Firmware Update

- **FR-7.1** [Must]: On `zb_cmd_firmware_update()` call, the framework shall transition
  to FW_UPDATE state, pause all normal ZNP traffic, and disable subscriber delivery
  except for `ZB_EVENT_FW_UPDATE_*` events.
- **FR-7.2** [Must]: The firmware update module shall reset the CC2652P7 into bootloader
  mode via GPIO and stream the firmware image over UART using the BSL (Boot Serial
  Loader) protocol.
- **FR-7.3** [Must]: Before streaming, the firmware module shall verify the supplied
  firmware blob is non-empty and passes a length / header sanity check.
- **FR-7.4** [Must]: On successful flash and verification, the framework shall re-initialize
  the coprocessor (re-enter ZNP_INIT) and restore normal operation.
- **FR-7.5** [Must]: On flash failure or verification failure, the framework shall emit
  `ZB_EVENT_FW_UPDATE_FAILED`, attempt to restart the coprocessor in normal mode, and
  return to READY if the existing firmware is still functional.
- **FR-7.6** [Should]: The firmware update module shall report progress in percentage
  increments via `ZB_EVENT_FW_UPDATE_PROGRESS` subscriber events.

#### FR-8: Storage Layer (`zb_storage`)

- **FR-8.1** [Must]: The system shall initialize the LittleFS filesystem on the `storage`
  partition (mount point `/zb`) in `zb_framework_init()` before any other framework
  operation.
- **FR-8.2** [Must]: `zb_storage_config_load()` shall read and parse `/zb/config.json`
  and return `ESP_ERR_NOT_FOUND` on first boot (no file present); the caller shall
  use compile-time Kconfig defaults in that case.
- **FR-8.3** [Must]: `zb_storage_config_save()` shall atomically overwrite
  `/zb/config.json` with the current network parameters (PAN ID, channel, network key
  as 32-char hex string, UART baud rate).
- **FR-8.4** [Must]: `zb_storage_device_save()` shall encode a `ZbDeviceRecord` to
  nanopb binary format and write it to `/zb/devices/<16-hex-ieee>.pb`, creating or
  overwriting the file.
- **FR-8.5** [Must]: `zb_storage_device_load()` shall read and nanopb-decode a single
  device file identified by IEEE address.
- **FR-8.6** [Must]: `zb_storage_device_delete()` shall remove the device file for the
  given IEEE address; returns `ESP_ERR_NOT_FOUND` if the file does not exist (not an
  error condition for the caller).
- **FR-8.7** [Must]: `zb_storage_device_load_all()` shall iterate all `*.pb` files in
  `/zb/devices/`, decode each, and invoke the supplied callback once per device.
- **FR-8.8** [Should]: If the LittleFS partition fails to mount, the framework shall
  format the partition (controlled by `CONFIG_ZB_STORAGE_FORMAT_ON_FAIL`) and continue
  with empty configuration rather than halting.
- **FR-8.9** [Should]: The storage layer shall log the number of devices restored at
  bootup at INFO level.

#### FR-9: Device Abstraction Layer

- **FR-9.1** [Must]: The system shall define an opaque capability handle type
  `zb_cap_id_t` (32-bit) encoding an endpoint and cluster ID:
  `ZB_CAP_ID(ep, cluster) = (uint32_t)(ep) << 16 | (cluster)`.
  Upper-layer code shall identify device capabilities using only `uint64_t ieee_addr`
  and `zb_cap_id_t`; raw endpoint and cluster values shall not appear in the upper-layer API.
- **FR-9.2** [Must]: The device manager shall maintain a **cluster schema table** — a
  static registry mapping ZCL cluster IDs to named schema entries, each containing a
  human-readable name and typed `set` / `get` function pointers
  (`zb_cap_set_fn` / `zb_cap_get_fn`).
- **FR-9.3** [Must]: On device join and endpoint/cluster discovery, the device manager
  shall iterate all discovered clusters, look up matching schemas in the table, and
  attach the resolved function pointers to the device's capability list.
- **FR-9.4** [Must]: `zb_dev_get_caps(ieee_addr, cap_list, count)` shall return the
  list of `zb_cap_id_t` values supported by a device, allowing upper-layer code to
  enumerate capabilities without knowing Zigbee structure.
- **FR-9.5** [Must]: Inbound ZCL attribute reports shall be translated by the
  abstraction layer into `zb_cap_event_t` before delivery to subscribers.
  `zb_cap_event_t` shall carry `ieee_addr`, `cap_id`, and a typed `value` union
  (e.g. `bool on_off`, `uint8_t level`, `int16_t temperature_hundredths`) resolved
  by the cluster schema — no raw attribute IDs or ZCL data types visible to callers.
- **FR-9.6** [Must]: Typed cluster-specific send functions shall be provided for the
  initially supported cluster set (see §6.4). Each function takes `(uint64_t ieee,
  zb_cap_id_t cap, <typed params>)` and returns `esp_err_t`; internally it resolves
  `cap_id` to endpoint + cluster and calls the ZNP command path.
- **FR-9.7** [Must]: `zb_cap_subscribe(ieee, cap_id, cb, ctx)` shall register a
  callback for events on a specific device capability (or `ZB_CAP_ANY` to receive all
  events from a device). The callback receives only `zb_cap_event_t *`.
- **FR-9.8** [Should]: If a received ZCL attribute report maps to no known cluster
  schema, the abstraction layer shall emit a raw `ZB_EVENT_ZNP_RAW_ATTR` event
  containing the `zb_cap_id_t` and raw bytes, rather than silently dropping it.
- **FR-9.9** [Should]: New cluster schemas shall be registrable at runtime via
  `zb_schema_register(const zb_cluster_schema_t *schema)`, allowing application code
  to extend the built-in schema table without modifying framework source.

#### FR-10: Physical User Button (`zb_button`)

- **FR-10.1** [Must]: A single momentary push-button connected to a configurable
  ESP32 GPIO (default GPIO34, input-only, external pull-up, active-low) shall be
  sampled by the firmware as a user input device.
- **FR-10.2** [Must]: The button driver shall debounce mechanical contact bounce
  with a software filter (default 20 ms stable window).
- **FR-10.3** [Must]: The button driver shall distinguish at least the following
  press patterns and emit a corresponding event for each:
    - **Single press** — one press-release with hold time shorter than the long-press
      threshold; emitted after the multi-press timeout expires without a second press.
    - **Double press** — two press-releases separated by less than the multi-press
      timeout (default 400 ms).
    - **Long press** — hold time exceeds the long-press threshold (default 1500 ms);
      emitted once when the threshold is crossed, while the button is still held.
- **FR-10.4** [Should]: Triple-press, quad-press, and very-long-press patterns shall
  be representable by the event type enum so they can be added later without an
  API break.
- **FR-10.5** [Must]: Button events shall be delivered through the existing
  `zb_subscribe` event bus as `ZB_EVENT_BUTTON` with the pattern carried in
  `e->data.button.event` (a `zb_button_event_kind_t` enum value).
- **FR-10.6** [Must]: A built-in default handler shall react to a **single press**
  by enqueueing `zb_cmd_permit_join(CONFIG_ZB_BUTTON_PERMIT_JOIN_SECONDS)` (default
  60 s). The handler shall not be invoked when the framework is not in READY state.
- **FR-10.7** [Should]: GPIO pin, debounce window, long-press threshold,
  multi-press timeout, and active-level polarity shall all be configurable via
  Kconfig.
- **FR-10.8** [Should]: The button driver shall use a single FreeRTOS task with a
  short polling interval (default 10 ms) rather than per-edge interrupts, to keep
  the debounce / multi-press timing self-contained and avoid ISR-from-task races.

#### FR-11: Network Status & Configuration Commands

- **FR-11.1** [Must]: The framework shall provide `zb_network_info_get(zb_network_info_t *out)`
  returning a snapshot of the current network state: derived `zb_network_status_t`, raw
  `zb_state_t`, active PAN ID, active channel, permit-join TTL (seconds remaining), and
  count of devices in the in-memory registry.
- **FR-11.2** [Must]: `zb_network_info_get()` shall be callable from any framework state,
  including before READY. When the framework has not yet reached READY, `pan_id` and
  `channel` shall reflect the last values loaded from `/zb/config.json` (or Kconfig
  defaults if no config file exists); `device_count` shall reflect the current in-memory
  registry size; `permit_join_ttl` shall be 0.
- **FR-11.3** [Must]: `zb_network_status_t` shall map the internal `zb_state_t` to a
  caller-facing status as follows:
    - `ZB_NET_STATUS_OFFLINE` — UNINITIALIZED or ZNP_INIT (coprocessor not yet responding).
    - `ZB_NET_STATUS_FORMING` — NETWORK_CHECK, FORMING, or RECONFIGURING.
    - `ZB_NET_STATUS_READY` — READY, no permit-join window open.
    - `ZB_NET_STATUS_PERMIT_JOIN` — READY, permit-join window open (`permit_join_ttl > 0`).
    - `ZB_NET_STATUS_FW_UPDATE` — FW_UPDATE, normal traffic suspended.
- **FR-11.4** [Must]: The framework shall provide `zb_network_param_set(const zb_net_config_t *cfg)`
  to update PAN ID, channel, network key, and UART baud rate. Parameters shall be persisted
  to `/zb/config.json`. If called while in READY state and PAN ID or channel differs from
  the active network, the framework shall trigger a RECONFIGURING transition.
- **FR-11.5** [Should]: `zb_network_info_get()` shall be thread-safe with no ZNP
  round-trip; it shall read module-static variables updated only by the framework task.
- **FR-11.6** [Should]: The framework shall maintain a `permit_join_ttl` counter,
  decremented by a 1-second FreeRTOS timer while in READY state, so that
  `zb_network_info_get()` can report the remaining join window without querying the
  coprocessor.

#### FR-12: Extended ZHA Device Cluster Support

- **FR-12.1** [Must]: The built-in cluster schema table shall be extended to support the
  following ZHA clusters in addition to the initial six defined in FR-9.2:

    | Cluster ID | Name | Settable | Primary `value` field in `zb_cap_event_t` |
    |-----------|------|----------|------------------------------------------|
    | 0x0403 | PressureMeasurement | No | `int16_t pressure_hpa` (hPa) |
    | 0x0406 | OccupancySensing | No | `bool occupancy` |
    | 0x0500 | IASZone | No | `zb_ias_zone_t ias_zone` |
    | 0x0702 | Metering | No | `zb_metering_t metering` |
    | 0x0B04 | ElectricalMeasurement | No | `zb_electrical_t electrical` |
    | 0x000C | AnalogInput | No | `float analog_input` (PresentValue) |
    | 0x000F | BinaryInput | No | `bool binary_input` |
    | 0x0010 | BinaryOutput | Yes (`bool`) | `bool binary_output` |

- **FR-12.2** [Must]: The IAS Zone schema (0x0500) shall decode three ZCL attributes into
  `zb_ias_zone_t`: `ZoneState` (0x0000, uint8), `ZoneType` (0x0001, uint16), and
  `ZoneStatus` (0x0002, uint16 bitmap). Zone type values include: `0x000D` motion detector,
  `0x0015` contact switch, `0x0028` fire/smoke sensor, `0x002A` water/flood sensor,
  `0x002B` CO sensor, `0x002C` personal emergency, `0x0225` vibration sensor.
  ZoneStatus bitmap: bit 0 = Alarm1, bit 1 = Alarm2, bit 2 = Tamper, bit 3 = LowBattery,
  bit 4 = SupervisionReports, bit 5 = RestoreReports, bit 6 = Trouble, bit 7 = AC/mains.
- **FR-12.3** [Must]: The IAS Zone schema shall handle `IAS_ZONE_STATUS_CHANGE_NOTIFICATION`
  (cluster-specific command 0x00) as an unsolicited report: decode the zone status bitmap
  from the command payload and deliver a `ZB_EVENT_CAP_REPORT` with `value.ias_zone`
  updated, without requiring a read-attribute poll from the upper layer.
- **FR-12.4** [Must]: The Metering schema (0x0702) shall decode `CurrentSummationDelivered`
  (0x0000, uint48), `CurrentSummationReceived` (0x0001, uint48), and `InstantaneousDemand`
  (0x0400, int24) into `zb_metering_t`. Raw summation values are in device-native units;
  callers apply `Multiplier` (0x0301) ÷ `Divisor` (0x0302) for engineering-unit conversion.
- **FR-12.5** [Must]: The Electrical Measurement schema (0x0B04) shall decode `RMSVoltage`
  (0x0505), `RMSCurrent` (0x0508), `ActivePower` (0x050B), and `PowerFactor` (0x0510) into
  `zb_electrical_t`. Default ZHA scaling: RMSVoltage in 0.1 V units; RMSCurrent in mA;
  ActivePower in W; PowerFactor as signed percent.
- **FR-12.6** [Must]: The BinaryOutput schema (0x0010) shall provide a typed `set` function
  `zb_cap_binary_output_set(uint64_t ieee, zb_cap_id_t cap, bool value)` that issues a ZCL
  Write Attributes command for `PresentValue` (0x0055).
- **FR-12.7** [Should]: When a Metering device's `Multiplier` and `Divisor` are not yet
  cached in the device record, the schema's `get` function shall issue additional
  read-attribute requests for both scaling attributes immediately after the primary
  summation read, cache the results in the cluster attribute store, and use them for
  all subsequent conversions without further ZNP round-trips.
- **FR-12.8** [Should]: All new cluster schemas shall be registered at framework init time
  via the same `zb_schema_register()` mechanism used by the initial six, preserving
  runtime extensibility (FR-9.9).

### 4.2 Non-Functional Requirements

- **NFR-1.1** [Must]: SREQ→SRSP round-trip latency for standard commands shall not
  exceed 100 ms under normal operating conditions.
- **NFR-1.2** [Must]: The framework shall recover from an unexpected ZNP desync
  (coprocessor reset) and return to READY state within 15 seconds.
- **NFR-1.3** [Must]: The device manager shall support a registry of at least 100
  simultaneously joined devices without performance degradation.
- **NFR-2.1** [Must]: All inbound data processing shall be interrupt or callback-driven;
  no blocking polling loops are permitted on ZNP receive paths.
- **NFR-2.2** [Must]: Subscriber callbacks shall be invoked within 200 ms of the
  corresponding AREQ being received by the transport layer.
- **NFR-3.1** [Must]: All state machine transitions shall be logged at DEBUG level.
- **NFR-3.2** [Should]: ZNP UART baud rate shall be configurable via Kconfig default and
  persisted in `/zb/config.json`, supporting at minimum 115200 bps and 230400 bps.
- **NFR-6.1** [Should]: LittleFS write operations (config save, device save) shall
  complete within 200 ms under normal flash conditions.
- **NFR-6.2** [Should]: The `/zb/devices/` directory shall support at least 100 device
  `.pb` files and `zb_storage_device_load_all()` shall complete within 5 seconds for
  100 devices at boot.
- **NFR-4.1** [Must]: The device registry shall survive an ESP32 power cycle; all
  previously joined devices shall be known on next boot.
- **NFR-4.2** [Must]: A failed CC2652P7 firmware update shall not leave the hub
  inoperative; the coprocessor shall remain bootable on its existing firmware.
- **NFR-5.1** [Should]: The framework shall expose a version string for both the ESP32
  firmware and the CC2652P7 Z-Stack firmware version (retrieved during ZNP_INIT).
- **NFR-5.2** [May]: Heap usage shall remain stable over 72 hours of continuous
  operation with no unbounded growth.

### 4.3 Constraints

- The CC2652P7 must be pre-programmed with a Z-Stack 3.x Coordinator firmware image
  supporting ZNP (MT serial interface) before first use.
- UART is the only supported physical link between ESP32 and CC2652P7; SPI or USB CDC
  are out of scope.
- The ESP32 must control at least one GPIO connected to the CC2652P7 RESET pin.
- The CC2652P7 BSL must be accessible: the BSL_INVOKE pin (or equivalent) must be wired
  from the ESP32.
- All code runs under ESP-IDF (FreeRTOS); Arduino framework is out of scope.

---

## 5. Risks, Assumptions & Dependencies

### 5.1 Risk Register

| ID | Risk | Likelihood | Impact | Mitigation |
|----|------|-----------|--------|------------|
| R-01 | ZNP UART desync during high Zigbee traffic causes command timeouts | Medium | High | Implement SOF-hunt re-sync, exponential back-off on retries, AREQ queue depth monitoring |
| R-02 | CC2652P7 BSL entry fails silently, leaving hub unable to update firmware | Medium | High | Verify BSL entry via GPIO state echo; detect BSL prompt byte before streaming |
| R-03 | Device registry size grows unbounded on networks with high churn | Low | Medium | Cap registry at 100 entries (NFR-1.3); LRU eviction for overflows |
| R-04 | LittleFS write failure (power loss mid-write) corrupts config.json or a device .pb file | Low | High | LittleFS uses copy-on-write; partial write leaves previous version intact; validate on read and fall back to defaults |
| R-09 | LittleFS partition full after extended operation with high device churn | Low | Medium | Cap registry at 100 devices (NFR-1.3); `zb_storage_device_delete()` called on every leave event; monitor filesystem usage at boot |
| R-05 | Zigbee channel 26 restricted in some jurisdictions (reduced TX power) | Low | Medium | Default to channel 15; document regional constraints |
| R-06 | Concurrent SREQ from command queue and framework during READY causes collision | Medium | High | Single-writer mutex on ZNP SREQ path; command queue serializes all outbound SREQs |
| R-07 | CC2652P7 firmware update interrupted by power loss mid-flash | Low | Critical | BSL protocol uses sector-granular writes; partial write → coprocessor falls back to BSL mode on next boot; user can retry |
| R-08 | Upper-layer subscriber callbacks block the notify task | Medium | Medium | Enforce callback timeout; deliver from dedicated task with bounded stack |

### 5.2 Assumptions

- The ESP32-N16R2 and CC2652P7 share a 3.3 V power rail with adequate decoupling (assumed).
- The CC2652P7 is pre-loaded with Z-Stack 3.x NCP/ZNP coordinator firmware (assumed).
- UART flow control (RTS/CTS) is available; hardware flow control is recommended at baud
  rates above 115200 bps (assumed optional in Phase 1, evaluated in Phase 2).
- PAN ID, channel, and network key are configured by the upper-layer application before
  calling `zb_framework_start()` (assumed, not provisioned via a UI flow).
- NVS flash is initialized by the caller (`nvs_flash_init()`) before `zb_framework_init()`;
  application config no longer uses NVS (LittleFS only) (assumed, caller responsibility).
- Wi-Fi connectivity for ESP32 OTA is provisioned separately (assumed, Phase 4 only).
- `idf.py update-dependencies` is run once before the first build to fetch
  `joltwallet/littlefs` and `nanopb/nanopb` from the ESP-IDF component registry (assumed).
- The project is built with ESP-IDF v5.x (assumed).

### 5.3 External Dependencies

| Dependency | Version | Purpose |
|-----------|---------|---------|
| ESP-IDF | v5.x | FreeRTOS, UART driver, NVS (system), OTA, Wi-Fi, cJSON (`json` component) |
| joltwallet/littlefs | ≥ 1.14.8 | LittleFS VFS driver for ESP32 (`idf_component.yml`) |
| nanopb/nanopb | ≥ 0.4.7 | Protobuf encode/decode for device records (`idf_component.yml`) |
| Z-Stack 3.x NCP firmware | TI release | CC2652P7 Coordinator/ZNP image |
| CC2652P7 BSL ROM | Built-in | UART bootloader for firmware update |

### 5.4 Open Questions (from project brief)

| # | Question | Impact | Target Phase |
|---|---------|--------|-------------|
| OQ-1 | UART baud rate and flow control scheme for ZNP | FR-1.1, NFR-3.2 | Phase 1 |
| ~~OQ-2~~ | ~~Persistent storage format for device registry~~ | **Resolved:** LittleFS + nanopb protobuf (one `.pb` file per device); JSON for network config | Closed |
| OQ-3 | Error recovery policy for ZNP desyncs mid-operation | FR-1.7, NFR-1.2 | Phase 2 |
| OQ-4 | Firmware update packaging format and integrity verification mechanism | FR-7.3 | Phase 4 |
| OQ-5 | FreeRTOS task priorities and stack sizes | NFR-2.1 | Phase 1 |
| OQ-6 | ESP32 OTA update strategy (Wi-Fi endpoint, trigger mechanism) | Phase 4 | Phase 4 |

---

## 6. Interface Specifications

### 6.1 External Interfaces

#### 6.1.1 ZNP UART Physical Link

| Parameter | Value |
|-----------|-------|
| Default baud rate | 115200 bps (configurable; TBD OQ-1) |
| Data bits | 8 |
| Stop bits | 1 |
| Parity | None |
| Flow control | RTS/CTS (assumed optional Phase 1; enable in Phase 2) |
| Frame start byte (SOF) | 0xFE |

#### 6.1.2 ZNP MT Frame Format

```
┌──────┬────────┬──────────┬──────────┬────────────────┬─────┐
│ SOF  │ Length │ Cmd Type │ Cmd ID   │ Payload        │ FCS │
│ 0xFE │ 1 byte │ 1 byte   │ 1 byte   │ 0–250 bytes    │ 1 B │
└──────┴────────┴──────────┴──────────┴────────────────┴─────┘
```

- **SOF**: always 0xFE.
- **Length**: byte count of Payload only.
- **Cmd Type**: high nibble = MT subsystem; low nibble = type (SREQ=0x2x, SRSP=0x6x, AREQ=0x4x).
- **FCS**: XOR of Length, Cmd Type, Cmd ID, and all Payload bytes.

#### 6.1.3 CC2652P7 BSL (Bootloader Serial Loader) Interface

| Parameter | Value |
|-----------|-------|
| Entry method | BSL_INVOKE GPIO low during reset |
| Protocol | TI Unified BSL over UART |
| Baud rate | 115200 bps (BSL default) |
| Frame format | TI Unified BSL packet (ACK-based) |
| Operations used | `COMMAND_DOWNLOAD`, `COMMAND_SEND_DATA`, `COMMAND_CRC32`, `COMMAND_RESET` |

### 6.2 Internal Interfaces

#### 6.2.1 ZNP Transport → Framework (AREQ Event Queue)

```c
typedef struct {
    uint8_t cmd_type;
    uint8_t cmd_id;
    uint8_t payload[ZNP_MAX_PAYLOAD];
    uint8_t payload_len;
} znp_areq_t;

// Posted to xAreqQueue by znp_dispatch_task
// Consumed by framework_task
```

#### 6.2.2 Framework → Device Manager (Internal Event)

```c
typedef enum {
    FW_INT_DEVICE_JOINED,
    FW_INT_DEVICE_LEFT,
    FW_INT_ATTR_REPORT,
    FW_INT_NETWORK_READY,
    FW_INT_NETWORK_LOST,
} fw_internal_event_type_t;

typedef struct {
    fw_internal_event_type_t type;
    union { /* per-event payloads */ } data;
} fw_internal_event_t;
```

#### 6.2.3 Command Queue (Upper Layer → Framework)

```c
typedef struct {
    zb_cmd_type_t type;
    union {
        struct { uint8_t duration_s; }                   permit_join;
        struct { uint16_t dev_id; uint8_t ep; uint16_t cluster;
                 uint8_t cmd; uint8_t *payload; uint8_t len; } send_zcl;
        struct { uint16_t dev_id; uint8_t ep; uint16_t cluster;
                 uint16_t attr_id; }                     read_attr;
        struct { uint8_t channel; }                      change_channel;
        struct { const uint8_t *blob; size_t len; }      fw_update;
    } params;
    SemaphoreHandle_t done_sem;   /* optional: caller blocks until complete */
    esp_err_t         result;
} zb_command_t;
```

### 6.3 Data Models / Schemas

#### 6.3.0 Network Configuration (`zb_net_config_t` / `config.json`)

**C struct** (defined in `zb_storage.h`):
```c
typedef struct {
    uint16_t pan_id;        // 16-bit PAN ID
    uint8_t  channel;       // Zigbee channel 11-26
    uint8_t  nwk_key[16];  // AES-128 network key
    uint32_t uart_baud;     // ZNP UART baud rate
} zb_net_config_t;
```

**JSON representation** (`/zb/config.json`):
```json
{
  "pan_id":    6699,
  "channel":   15,
  "nwk_key":   "01030507090b0d0f00020406080a0c0d",
  "uart_baud": 115200
}
```

- `nwk_key` is stored as a 32-character lowercase hex string (16 bytes × 2 hex digits).
- File is created on first successful network formation; subsequent boots read it to
  determine if the CC2652P7 NV matches and skip re-formation.

#### 6.3.1 Device Record (`zb_device_t` / `ZbDeviceRecord`)

```c
typedef struct {
    uint64_t ieee_addr;               // 64-bit IEEE address (EUI-64)
    uint16_t network_addr;            // 16-bit NWK address (may change on rejoin)
    uint8_t  num_endpoints;
    zb_endpoint_t endpoints[ZB_MAX_ENDPOINTS_PER_DEV]; // (default 8)
    uint32_t last_seen_ms;            // ESP32 tick time of last communication
} zb_device_t;

typedef struct {
    uint8_t  ep_id;
    uint16_t profile_id;
    uint16_t device_id;
    uint8_t  num_clusters;
    zb_cluster_cache_t clusters[ZB_MAX_CLUSTERS_PER_EP]; // (default 16)
} zb_endpoint_t;

typedef struct {
    uint16_t cluster_id;
    uint8_t  num_attrs;
    zb_attr_cache_t attrs[ZB_MAX_ATTRS_PER_CLUSTER]; // (default 8)
} zb_cluster_cache_t;

typedef struct {
    uint16_t attr_id;
    uint8_t  data_type;
    uint8_t  value[8];    // raw ZCL attribute value, up to 8 bytes
    uint32_t updated_ms;
} zb_attr_cache_t;
```

**Protobuf schema** (`components/zb_storage/proto/zb_device.proto`):
```protobuf
syntax = "proto3";

message ZbAttrCache {
    uint32 attr_id    = 1;  // ZCL attribute ID
    uint32 data_type  = 2;  // ZCL data type byte
    bytes  value      = 3;  // raw value (nanopb max_size:8)
    uint32 updated_ms = 4;
}
message ZbClusterCache {
    uint32 cluster_id          = 1;
    repeated ZbAttrCache attrs = 2;  // max_count:8
}
message ZbEndpoint {
    uint32 ep_id                     = 1;
    uint32 profile_id                = 2;
    uint32 device_id                 = 3;
    repeated ZbClusterCache clusters = 4;  // max_count:16
}
message ZbDeviceRecord {
    uint64 ieee_addr              = 1;
    uint32 network_addr           = 2;
    repeated ZbEndpoint endpoints = 3;  // max_count:8
    uint32 last_seen_ms           = 4;
}
```

- Static allocation bounds are declared in `proto/zb_device.options`; all structs are
  stack/static allocated (no heap fragmentation on encode/decode).
- Each device is stored as `/zb/devices/<16-hex-ieee>.pb` — e.g.,
  `/zb/devices/00124b001234abcd.pb`.

#### 6.3.1b Capability Handle (`zb_cap_id_t`) and Schema

```c
/* Opaque capability handle — encodes endpoint + cluster */
typedef uint32_t zb_cap_id_t;
#define ZB_CAP_ID(ep, cluster)  ((uint32_t)(ep) << 16 | (uint16_t)(cluster))
#define ZB_CAP_EP(cap)          ((uint8_t)((cap) >> 16))
#define ZB_CAP_CLUSTER(cap)     ((uint16_t)((cap) & 0xFFFF))
#define ZB_CAP_ANY              ((zb_cap_id_t)0xFFFFFFFF)  /* wildcard for subscribe */

/* Typed set / get function signatures */
typedef esp_err_t (*zb_cap_set_fn)(uint64_t ieee, zb_cap_id_t cap,
                                    const void *value, size_t len);
typedef esp_err_t (*zb_cap_get_fn)(uint64_t ieee, zb_cap_id_t cap);

/* Cluster schema entry — one row per supported ZCL cluster */
typedef struct {
    uint16_t       cluster_id;
    const char    *name;         /* e.g. "OnOff", "LevelControl" */
    zb_cap_set_fn  set;          /* NULL if cluster has no settable state */
    zb_cap_get_fn  get;          /* NULL if cluster has no readable state */
} zb_cluster_schema_t;
```

**Built-in schema table** (extensible via `zb_schema_register()`; initial 6 + FR-12 additions):

| Cluster ID | Name | Typed set params | Typed get / notes |
|-----------|------|-----------------|-------------------|
| 0x0006 | OnOff | `bool on` | read OnOff attr |
| 0x0008 | LevelControl | `uint8_t level` | read CurrentLevel attr |
| 0x0300 | ColorControl | `uint16_t x, uint16_t y` | read CurrentX/Y attrs |
| 0x0402 | TemperatureMeasurement | — | read MeasuredValue attr |
| 0x0405 | RelativeHumidity | — | read MeasuredValue attr |
| 0x0400 | IlluminanceMeasurement | — | read MeasuredValue attr |
| 0x0403 | PressureMeasurement | — | read MeasuredValue attr (hPa) |
| 0x0406 | OccupancySensing | — | read Occupancy bitmap attr |
| 0x0500 | IASZone | — | read ZoneState/Type/Status; unsolicited STATUS_CHANGE_NOTIFICATION |
| 0x0702 | Metering | — | read Summation + InstantaneousDemand; auto-fetch Multiplier/Divisor |
| 0x0B04 | ElectricalMeasurement | — | read RMSVoltage, RMSCurrent, ActivePower, PowerFactor |
| 0x000C | AnalogInput | — | read PresentValue (float) |
| 0x000F | BinaryInput | — | read PresentValue (bool) |
| 0x0010 | BinaryOutput | `bool value` | read/write PresentValue via ZCL Write Attributes |

#### 6.3.1c Capability Event (`zb_cap_event_t`)

```c
typedef struct {
    uint64_t    ieee_addr;   /* device identifier */
    zb_cap_id_t cap_id;      /* ZB_CAP_ID(ep, cluster) */
    union {
        bool     on_off;                  /* cluster 0x0006 */
        uint8_t  level;                   /* cluster 0x0008  (0–254) */
        struct { uint16_t x; uint16_t y; } color_xy;  /* cluster 0x0300 */
        int16_t  temperature_hundredths;  /* cluster 0x0402  (°C × 100) */
        int16_t  pressure_hpa;            /* cluster 0x0403  (hPa) */
        uint16_t humidity_hundredths;     /* cluster 0x0405  (% × 100) */
        uint32_t illuminance_lux;         /* cluster 0x0400 */
        bool     occupancy;               /* cluster 0x0406 */
        zb_ias_zone_t   ias_zone;         /* cluster 0x0500 — see §6.3.1e */
        zb_metering_t   metering;         /* cluster 0x0702 — see §6.3.1e */
        zb_electrical_t electrical;       /* cluster 0x0B04 — see §6.3.1e */
        float    analog_input;            /* cluster 0x000C  (PresentValue) */
        bool     binary_input;            /* cluster 0x000F */
        bool     binary_output;           /* cluster 0x0010 */
        struct {                          /* fallback for unknown clusters */
            uint16_t attr_id;
            uint8_t  data_type;
            uint8_t  raw[8];
        } raw;
    } value;
} zb_cap_event_t;

typedef void (*zb_cap_event_cb_t)(const zb_cap_event_t *event, void *ctx);
```

#### 6.3.1d Network Info (`zb_network_info_t`) — FR-11

```c
typedef enum {
    ZB_NET_STATUS_OFFLINE = 0,   /* UNINITIALIZED or ZNP_INIT — coprocessor not responding */
    ZB_NET_STATUS_FORMING,       /* NETWORK_CHECK / FORMING / RECONFIGURING */
    ZB_NET_STATUS_READY,         /* READY — coordinator operational, no join window open */
    ZB_NET_STATUS_PERMIT_JOIN,   /* READY — join window open (permit_join_ttl > 0) */
    ZB_NET_STATUS_FW_UPDATE,     /* FW_UPDATE — normal traffic suspended */
} zb_network_status_t;

typedef struct {
    zb_network_status_t status;          /* high-level derived status (FR-11.3) */
    zb_state_t          fw_state;        /* raw framework state machine value */
    uint16_t            pan_id;          /* active PAN ID; 0 if not yet configured */
    uint8_t             channel;         /* active channel; 0 if not yet configured */
    uint8_t             permit_join_ttl; /* seconds remaining in join window; 0 = closed */
    uint16_t            device_count;    /* devices in the in-memory registry */
} zb_network_info_t;
```

Populated by `zb_network_info_get()`. Valid from any framework state; fields not yet
meaningful are set to zero.

#### 6.3.1e Extended ZHA Value Types — FR-12

Supporting structs used in the `zb_cap_event_t.value` union for the extended cluster set:

```c
/* IAS Zone cluster (0x0500) — FR-12.2 */
typedef struct {
    uint8_t  zone_state;   /* 0=not enrolled, 1=enrolled */
    uint16_t zone_type;    /* device class: 0x000D=motion, 0x0015=contact,
                            *   0x0028=fire/smoke, 0x002A=water/flood,
                            *   0x002B=CO, 0x002C=personal emergency,
                            *   0x0225=vibration */
    uint16_t zone_status;  /* bitmap: bit0=Alarm1, bit1=Alarm2, bit2=Tamper,
                            *         bit3=LowBattery, bit4=SupervisionReports,
                            *         bit5=RestoreReports, bit6=Trouble,
                            *         bit7=AC/mains */
} zb_ias_zone_t;

/* Metering cluster (0x0702) — FR-12.4 */
typedef struct {
    uint64_t summation_delivered;  /* total energy/volume delivered (ZCL uint48, raw) */
    uint64_t summation_received;   /* total energy/volume received  (ZCL uint48, raw) */
    int32_t  instantaneous_demand; /* current power or flow (ZCL int24, raw) */
} zb_metering_t;
/* Apply Multiplier (attr 0x0301) / Divisor (attr 0x0302) for engineering units. */

/* Electrical Measurement cluster (0x0B04) — FR-12.5 */
typedef struct {
    uint16_t rms_voltage;  /* AC voltage in 0.1 V steps  (e.g. 2300 = 230.0 V) */
    uint16_t rms_current;  /* AC current in mA            (e.g. 1500 = 1.500 A) */
    int16_t  active_power; /* active power in W */
    int8_t   power_factor; /* power factor in %  (signed: −100 to +100) */
} zb_electrical_t;
```

#### 6.3.2 Subscriber Event (`zb_event_t`)

```c
typedef enum {
    ZB_EVENT_NETWORK_READY,
    ZB_EVENT_NETWORK_LOST,
    ZB_EVENT_DEVICE_JOINED,
    ZB_EVENT_DEVICE_LEFT,
    ZB_EVENT_DEVICE_ANNOUNCE,
    ZB_EVENT_ATTR_REPORT,
    ZB_EVENT_ZNP_ERROR,
    ZB_EVENT_FW_UPDATE_PROGRESS,
    ZB_EVENT_FW_UPDATE_DONE,
    ZB_EVENT_FW_UPDATE_FAILED,
} zb_event_type_t;

typedef struct {
    zb_event_type_t type;
    union {
        struct { uint64_t ieee_addr; uint16_t nwk_addr; }   device;
        struct { uint64_t ieee_addr; uint8_t ep;
                 uint16_t cluster; uint16_t attr_id;
                 uint8_t data_type; uint8_t value[8]; }     attr_report;
        struct { uint8_t percent_done; }                    fw_progress;
        struct { esp_err_t code; const char *msg; }         error;
    } data;
} zb_event_t;
```

### 6.4 Commands / Opcodes

#### Subscribe API

```c
esp_err_t zb_subscribe(zb_event_type_t event, zb_event_cb_t callback);
esp_err_t zb_unsubscribe(zb_event_type_t event, zb_event_cb_t callback);
```

#### Command API

```c
esp_err_t zb_cmd_permit_join(uint8_t duration_s);
esp_err_t zb_cmd_send_zcl(uint16_t dev_id, uint8_t ep, uint16_t cluster,
                           uint8_t cmd, const uint8_t *payload, uint8_t len);
esp_err_t zb_cmd_read_attr(uint16_t dev_id, uint8_t ep,
                            uint16_t cluster, uint16_t attr_id);
esp_err_t zb_cmd_change_channel(uint8_t channel);
esp_err_t zb_cmd_firmware_update(const uint8_t *fw_blob, size_t fw_len);
```

#### Framework Lifecycle

```c
esp_err_t zb_framework_init(const zb_config_t *cfg);
esp_err_t zb_framework_start(void);
zb_state_t zb_framework_get_state(void);
esp_err_t zb_framework_get_versions(zb_versions_t *out);
```

#### Capability API (`zb_cap.h`) — upper-layer interface

```c
/* Subscribe to events for a specific capability (or ZB_CAP_ANY for all) */
esp_err_t zb_cap_subscribe(uint64_t ieee, zb_cap_id_t cap,
                            zb_cap_event_cb_t cb, void *ctx);
esp_err_t zb_cap_unsubscribe(uint64_t ieee, zb_cap_id_t cap,
                              zb_cap_event_cb_t cb);

/* Enumerate capabilities of a joined device */
esp_err_t zb_dev_get_caps(uint64_t ieee, zb_cap_id_t *cap_list,
                           uint8_t *count);  /* count: in/out */

/* Generic set/get (dispatches through schema function table) */
esp_err_t zb_cap_set(uint64_t ieee, zb_cap_id_t cap,
                     const void *value, size_t len);
esp_err_t zb_cap_get(uint64_t ieee, zb_cap_id_t cap);

/* Typed convenience functions — initial cluster set */
esp_err_t zb_cap_onoff_set(uint64_t ieee, zb_cap_id_t cap, bool on);
esp_err_t zb_cap_onoff_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_level_set(uint64_t ieee, zb_cap_id_t cap, uint8_t level);
esp_err_t zb_cap_level_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_color_xy_set(uint64_t ieee, zb_cap_id_t cap,
                               uint16_t x, uint16_t y);
esp_err_t zb_cap_temp_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_humidity_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_illuminance_get(uint64_t ieee, zb_cap_id_t cap);

/* Extended ZHA cluster functions (FR-12) */
esp_err_t zb_cap_pressure_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_occupancy_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_ias_zone_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_metering_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_electrical_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_analog_input_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_binary_input_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_binary_output_set(uint64_t ieee, zb_cap_id_t cap, bool value);
esp_err_t zb_cap_binary_output_get(uint64_t ieee, zb_cap_id_t cap);

/* Schema extension */
esp_err_t zb_schema_register(const zb_cluster_schema_t *schema);
```

**Upper-layer usage example** (no ZCL knowledge required):
```c
// On device joined event — enumerate capabilities
uint8_t count = 16;
zb_cap_id_t caps[16];
zb_dev_get_caps(ieee, caps, &count);
// caps[0] = ZB_CAP_ID(1, 0x0006)  → OnOff
// caps[1] = ZB_CAP_ID(1, 0x0008)  → LevelControl

// Subscribe to all events from this device
zb_cap_subscribe(ieee, ZB_CAP_ANY, my_event_handler, NULL);

// Control
zb_cap_onoff_set(ieee, caps[0], true);
zb_cap_level_set(ieee, caps[1], 128);

// Event handler — no ZCL details
void my_event_handler(const zb_cap_event_t *e, void *ctx) {
    if (ZB_CAP_CLUSTER(e->cap_id) == 0x0006)
        printf("OnOff: %s\n", e->value.on_off ? "ON" : "OFF");
}
```

#### Network Info API (`zb_framework.h`) — FR-11

```c
/* Query current network status without a ZNP round-trip (FR-11.1, FR-11.5).
 * Safe to call from any state; fields not yet meaningful are zeroed. */
esp_err_t zb_network_info_get(zb_network_info_t *out);

/* Persist updated network parameters; triggers RECONFIGURING if channel or
 * PAN ID has changed while the framework is in READY state (FR-11.4). */
esp_err_t zb_network_param_set(const zb_net_config_t *cfg);
```

#### Storage API (`zb_storage.h`)

```c
esp_err_t zb_storage_init(void);
esp_err_t zb_storage_config_load(zb_net_config_t *cfg);
esp_err_t zb_storage_config_save(const zb_net_config_t *cfg);
esp_err_t zb_storage_device_save(const ZbDeviceRecord *dev);
esp_err_t zb_storage_device_load(uint64_t ieee_addr, ZbDeviceRecord *dev);
esp_err_t zb_storage_device_delete(uint64_t ieee_addr);
esp_err_t zb_storage_device_load_all(zb_storage_device_cb_t cb, void *ctx);
```

---

## 7. Operational Procedures

### 7.1 Initial Hardware Setup

1. Flash CC2652P7 with a validated Z-Stack 3.x Coordinator/ZNP firmware image using
   UniFlash or the TI flasher tool (done once, or via the firmware update flow in Phase 4).
2. Connect UART TX/RX and (optionally) RTS/CTS between ESP32 and CC2652P7.
3. Connect ESP32 GPIO → CC2652P7 RESET pin.
4. Connect ESP32 GPIO → CC2652P7 BSL_INVOKE pin.
5. Power the board; verify both chips are powered at 3.3 V.

### 7.2 First-Time Firmware Flash (ESP32)

```bash
idf.py set-target esp32
idf.py update-dependencies          # fetch joltwallet/littlefs and nanopb/nanopb
idf.py menuconfig                   # set ZNP UART pins, baud rate, PAN ID, channel
idf.py build flash monitor
```

### 7.3 Network Configuration

Before calling `zb_framework_start()`, upper-layer code must set network parameters
via the framework config structure:

```c
zb_config_t cfg = {
    .pan_id      = 0x1A2B,       // 16-bit PAN ID
    .channel     = 15,           // 2.4 GHz channel (11–26)
    .nwk_key     = { /* 16-byte AES-128 key */ },
    .uart_port   = UART_NUM_1,
    .uart_baud   = 115200,
    .gpio_reset  = GPIO_NUM_4,
    .gpio_bsl    = GPIO_NUM_5,
};
zb_framework_init(&cfg);
zb_framework_start();
```

Parameters are persisted to `/zb/config.json` after first successful network formation;
subsequent boots read this file and skip re-formation if the CC2652P7 NV matches.
To force re-formation, delete `/zb/config.json` or call factory reset.

### 7.4 Normal Operation

1. Framework boots to READY automatically (see Section 2.3.2 boot sequence).
2. Upper-layer subscribes to desired events via `zb_subscribe()`.
3. Open the network for joining: `zb_cmd_permit_join(60)`.
4. Zigbee devices join; `ZB_EVENT_DEVICE_JOINED` delivered to subscribers.
5. ZCL attribute reports arrive automatically; `ZB_EVENT_ATTR_REPORT` delivered.
6. Send commands with `zb_cmd_send_zcl()` / `zb_cmd_read_attr()`.

### 7.5 Coprocessor Firmware Update

1. Load CC2652P7 firmware binary into a buffer accessible to the ESP32 (from HTTP,
   SD card, embedded in flash, or passed directly — mechanism is caller's responsibility).
2. Call `zb_cmd_firmware_update(blob, len)`.
3. Framework transitions to FW_UPDATE; normal Zigbee traffic is suspended.
4. Monitor progress via `ZB_EVENT_FW_UPDATE_PROGRESS` subscriber callbacks.
5. On `ZB_EVENT_FW_UPDATE_DONE`: firmware installed, coprocessor re-initialized,
   framework returns to READY automatically.
6. On `ZB_EVENT_FW_UPDATE_FAILED`: inspect error code; retry if appropriate.

### 7.6 Recovery Procedures

| Scenario | Recovery Action |
|----------|----------------|
| Coprocessor spontaneous reset | Framework auto re-enters ZNP_INIT → READY within 15 s; no action required |
| ZNP desync (framing errors) | Transport performs SOF-hunt re-sync automatically |
| ESP32 firmware crash / watchdog | ESP32 reboots; LittleFS remounted; device registry restored from `.pb` files |
| LittleFS mount failure / corruption | `CONFIG_ZB_STORAGE_FORMAT_ON_FAIL=y` formats partition; config and registry lost; devices must re-join |
| Device `.pb` file corrupt (bad protobuf) | `zb_storage_device_load_all()` skips the file with a warning; other devices still loaded |
| Failed CC2652P7 firmware update | Coprocessor remains on previous firmware; retry `zb_cmd_firmware_update()` |
| PAN ID / channel change desired | Call `zb_cmd_change_channel()` or delete `/zb/config.json` and reboot |

### 7.7 Factory Reset

To return the hub to factory state (clear device registry and network config):

```c
// Remove config file — framework will use Kconfig defaults and re-form
remove("/zb/config.json");

// Remove all device files
zb_storage_device_load_all(remove_device_cb, NULL);

// Also clear CC2652P7 NV so coordinator re-forms from scratch
// (handled automatically by RECONFIGURING state on next boot)
esp_restart();
```

Alternatively, format the entire LittleFS partition:
```c
esp_littlefs_format("storage");
esp_restart();
```

The hub will form a new network on next boot using compile-time Kconfig defaults.

---

## 8. Verification & Validation

### 8.1 Phase 1 Verification — ZNP Transport

| Test ID | Feature | Procedure | Success Criteria |
|---------|---------|-----------|-----------------|
| TC-1.01 | UART init | Initialize transport at 115200 bps; verify no error returned | `ESP_OK` returned; no assert |
| TC-1.02 | SYS_PING round-trip | Send `SYS_PING` SREQ; receive SRSP | SRSP received within 100 ms; FCS valid |
| TC-1.03 | AREQ reception | Reset CC2652P7; await `SYS_RESET_IND` | AREQ enqueued within 3 s |
| TC-1.04 | SRSP timeout | Send SREQ with invalid subsystem; wait | Error code `ZNP_ERR_TIMEOUT` returned after 3 s |
| TC-1.05 | FCS error re-sync | Inject byte 0xFF into UART stream; send valid frame | Valid frame received after recovery; no hang |
| TC-1.06 | Framing unit tests | Run offline framing encode/decode tests | 100% of test vectors pass |
| TC-1.07 | UART overrun recovery | Flood RX buffer; send valid frame | Valid frame received; overrun counter incremented |

### 8.2 Phase 2 Verification — State Machine & Network

| Test ID | Feature | Procedure | Success Criteria |
|---------|---------|-----------|-----------------|
| TC-2.01 | Boot to READY | Cold boot; monitor serial | READY state reached within 10 s |
| TC-2.02 | Network formation | Delete `/zb/config.json`; cold boot | Coordinator starts on configured channel; `ZB_EVENT_NETWORK_READY` received; `config.json` created |
| TC-2.03 | Network re-use | Boot with existing `config.json` | Skips FORMING; reaches READY directly |
| TC-2.04 | Device join | Open permit-join; pair test device | `ZB_EVENT_DEVICE_JOINED` delivered within 500 ms |
| TC-2.05 | Coprocessor reset recovery | Assert RESET GPIO during READY; release | Framework re-enters ZNP_INIT; READY restored within 15 s |
| TC-2.06 | RECONFIGURING | Edit `config.json` channel value; reboot | Framework detects mismatch vs. CC2652P7 NV; re-forms on new channel |
| TC-2.09 | LittleFS mount | Cold boot; check serial log | "LittleFS mounted: /zb" logged; no mount error |
| TC-2.10 | Config file created | First boot (no config.json); complete formation | `/zb/config.json` exists with correct pan_id, channel, nwk_key after READY |
| TC-2.07 | State logging | Monitor UART at DEBUG level | All state transitions logged |
| TC-2.08 | `get_state` query | Query state during each phase | Correct state returned at each point |

### 8.3 Phase 3 Verification — Device Manager & API

| Test ID | Feature | Procedure | Success Criteria |
|---------|---------|-----------|-----------------|
| TC-3.01 | Subscribe delivery | Subscribe to `ZB_EVENT_DEVICE_JOINED`; pair device | Callback invoked; IEEE and NWK addr in payload |
| TC-3.02 | Attribute report delivery | Subscribe to `ZB_EVENT_ATTR_REPORT`; trigger report | Callback invoked within 200 ms; correct cluster/attr |
| TC-3.03 | `zb_dev_get` lookup | Join device; call `zb_dev_get(ieee_addr)` | Non-NULL pointer; correct record |
| TC-3.04 | Registry persistence | Join 3 devices; power-cycle ESP32; call `zb_dev_get` | 3 `.pb` files in `/zb/devices/`; all 3 records loaded at READY entry |
| TC-3.05 | `permit_join` command | `zb_cmd_permit_join(60)`; pair device within window | Device joins; join rejected after 60 s |
| TC-3.06 | `read_attr` command | `zb_cmd_read_attr(dev, ep, cluster, attr)` | `ZB_EVENT_ATTR_REPORT` with correct value |
| TC-3.07 | Command queue during FORMING | Issue command before READY; await READY | Command executed post-READY; no error |
| TC-3.08 | Multi-subscriber fanout | Register 3 subscribers for same event; trigger event | All 3 callbacks invoked |
| TC-3.09 | Leave event | Force device to leave; subscribe `ZB_EVENT_DEVICE_LEFT` | Callback invoked; device removed from registry |
| TC-3.10 | LittleFS corruption fallback | Format LittleFS partition (`esp_littlefs_format`); reboot | Framework boots to READY with empty registry; no crash; `config.json` absent → uses defaults |
| TC-3.11 | Device .pb created on join | Join one device | File `/zb/devices/<ieee>.pb` exists; decode with `zb_storage_device_load()` returns correct record |
| TC-3.12 | Device .pb deleted on leave | Force device to leave | `.pb` file removed from `/zb/devices/`; `zb_storage_device_load()` returns `ESP_ERR_NOT_FOUND` |
| TC-3.13 | Load-all at READY | Join 5 devices; reboot | Serial log: "loaded 5 device(s) from storage"; all accessible via `zb_dev_get` |
| TC-3.14 | Corrupt .pb file skipped | Overwrite one `.pb` with random bytes; reboot | Warning logged for bad file; other devices loaded successfully; hub reaches READY |
| TC-3.15 | Capability enumeration | Join OnOff bulb; call `zb_dev_get_caps` | Returns cap list including `ZB_CAP_ID(1, 0x0006)`; no raw endpoint/cluster in upper layer |
| TC-3.16 | Typed OnOff command | `zb_cap_onoff_set(ieee, cap, true)` | Bulb turns on; no ZCL frame construction needed by caller |
| TC-3.17 | Cap event typed delivery | Subscribe with `ZB_CAP_ANY`; trigger attribute report from sensor | `zb_cap_event_t` delivered with `value.temperature_hundredths` populated; no raw bytes in callback |
| TC-3.18 | Schema registration | Register custom cluster schema at runtime | `zb_cap_set/get` dispatch correctly for the new cluster ID |
| TC-3.19 | Unknown cluster raw fallback | Receive attr report from cluster with no schema | `ZB_EVENT_ZNP_RAW_ATTR` emitted; `value.raw` contains attr_id + bytes; no crash |
| TC-3.20 | Button single press → permit join | While in READY: press the GPIO34 button once and release within 1 s | `ZB_EVENT_BUTTON` (kind=SINGLE) delivered after multi-press timeout; `zb_cmd_permit_join(60)` enqueued; `ZDO_MGMT_PERMIT_JOIN_REQ` sent to coprocessor |
| TC-3.21 | Button double press detected | Two presses within 400 ms | One `ZB_EVENT_BUTTON` with kind=DOUBLE; no SINGLE event emitted |
| TC-3.22 | Button long press detected | Hold the button for ≥ 1.5 s | `ZB_EVENT_BUTTON` (kind=LONG) emitted once while still held; no SINGLE event on release |
| TC-3.23 | Button debounce | Inject a 5 ms bounce burst on the input | No event emitted (signal not stable across debounce window) |
| TC-3.24 | `zb_network_info_get` — offline | Call during ZNP_INIT before coprocessor responds | `status==ZB_NET_STATUS_OFFLINE`; `fw_state==ZB_STATE_ZNP_INIT`; `device_count==0` |
| TC-3.25 | `zb_network_info_get` — READY | Call after coordinator reaches READY | `status==ZB_NET_STATUS_READY`; `pan_id` and `channel` match `/zb/config.json` |
| TC-3.26 | `zb_network_info_get` — permit-join | Call after `zb_cmd_permit_join(60)` | `status==ZB_NET_STATUS_PERMIT_JOIN`; `permit_join_ttl` in range 1–60; decrements each second |
| TC-3.27 | `zb_network_param_set` persists | Set new channel while READY; read back `/zb/config.json` | File updated with new channel; reboot detects NV mismatch → RECONFIGURING |
| TC-3.28 | IAS zone status change (unsolicited) | IAS device triggers alarm (Alarm1 bit) | `ZB_EVENT_CAP_REPORT` with `value.ias_zone.zone_status` bit 0 set; no read-attr poll required |
| TC-3.29 | IAS zone type and enroll state | Join IAS contact switch; call `zb_cap_ias_zone_get` | `value.ias_zone.zone_state==1` (enrolled); `zone_type==0x0015` (contact switch) |
| TC-3.30 | Metering summation delivered | Subscribe to smart plug reporting energy | `ZB_EVENT_CAP_REPORT`; `value.metering.summation_delivered > 0`; `instantaneous_demand` reflects live power |
| TC-3.31 | Electrical measurement | Subscribe to smart plug; trigger `zb_cap_electrical_get` | `value.electrical.rms_voltage` in range 2100–2400 (210–240 V); `active_power >= 0` |
| TC-3.32 | Occupancy sensing | Motion triggers PIR sensor report | `ZB_EVENT_CAP_REPORT` with `value.occupancy == true`; clears on subsequent unoccupied report |
| TC-3.33 | Pressure measurement | Subscribe to pressure sensor; trigger report | `value.pressure_hpa` in plausible range 900–1100 hPa |
| TC-3.34 | AnalogInput present value | Subscribe to AI device; trigger attribute report | `value.analog_input` is a finite float; matches raw ZCL float32 attribute |
| TC-3.35 | BinaryOutput set | `zb_cap_binary_output_set(ieee, cap, true)` while READY | ZCL Write Attributes sent to device; output activates; no error returned |

### 8.4 Phase 4 Verification — Firmware Update & Integration

| Test ID | Feature | Procedure | Success Criteria |
|---------|---------|-----------|-----------------|
| TC-4.01 | Valid FW update | Pass valid CC2652P7 firmware blob to `zb_cmd_firmware_update` | `ZB_EVENT_FW_UPDATE_DONE`; coordinator re-initializes |
| TC-4.02 | Update progress events | Subscribe `ZB_EVENT_FW_UPDATE_PROGRESS` during update | Progress events at ≥ 10 % increments |
| TC-4.03 | Invalid blob rejection | Pass zero-length blob | `ESP_ERR_INVALID_ARG`; no flash attempt; READY retained |
| TC-4.04 | Corrupt blob handling | Pass blob with bad CRC | `ZB_EVENT_FW_UPDATE_FAILED`; coprocessor recoverable |
| TC-4.05 | FW update state isolation | Issue `zb_cmd_send_zcl` during FW_UPDATE | Returns `ZNP_ERR_NOT_READY`; command not queued |
| TC-4.06 | Post-update operation | After successful update; send/receive ZCL | Normal ZCL operation restored |
| TC-4.07 | ESP32 OTA (assumed) | Trigger HTTP OTA; verify version string updates | New ESP32 firmware running; `/zb/devices/` files intact; registry restored on READY |

### 8.5 Acceptance Tests

| Test ID | Scenario | Procedure | Success Criteria |
|---------|---------|-----------|-----------------|
| AT-01 | End-to-end attribute pipeline | Join bulb; read brightness attribute | Value delivered to subscriber in < 500 ms end-to-end |
| AT-02 | ZCL command to device | Send OnOff toggle command to bulb | Bulb toggles; no framework error |
| AT-03 | 10-device stress | Join 10 devices; send simultaneous read-attr requests | All responses delivered; no timeout or queue overflow |
| AT-04 | 72-hour stability | Run with 3 joined devices reporting every 60 s | No crash, memory leak, or desync over 72 hours |
| AT-05 | Network rejoin after hub reboot | Hub reboots; devices rejoin; attribute report received | All devices rejoin within 2 minutes; event delivered |
| AT-06 | Coprocessor FW update cycle | Full update → READY → 30 min normal operation | No regressions; version updated |

### 8.6 Traceability Matrix

| Requirement | Priority | Test Case(s) | Status |
|------------|---------|-------------|--------|
| FR-1.1 | Must | TC-1.01 | Covered |
| FR-1.2 | Must | TC-1.06 | Covered |
| FR-1.3 | Must | TC-1.06 | Covered |
| FR-1.4 | Must | TC-1.02, TC-1.04 | Covered |
| FR-1.5 | Must | TC-1.03 | Covered |
| FR-1.6 | Must | TC-1.03, TC-2.05 | Covered |
| FR-1.7 | Must | TC-1.05, TC-1.07 | Covered |
| FR-1.8 | Should | TC-1.07 | Covered |
| FR-2.1 | Must | TC-2.01 | Covered |
| FR-2.2 | Must | TC-2.01, TC-2.03 | Covered |
| FR-2.3 | Must | TC-2.03, TC-2.10 | Covered |
| FR-2.4 | Must | TC-2.02 | Covered |
| FR-2.5 | Must | TC-2.02 | Covered |
| FR-2.6 | Must | TC-2.06 | Covered |
| FR-2.7 | Must | TC-2.01, TC-2.02, TC-2.03 | Covered |
| FR-2.8 | Must | TC-2.05 | Covered |
| FR-2.9 | Should | TC-2.07 | Covered |
| FR-2.10 | Must | TC-2.08 | Covered |
| FR-3.1 | Must | TC-2.02, TC-2.03, TC-2.10 | Covered |
| FR-3.2 | Must | TC-2.02, TC-2.06, TC-2.10 | Covered |
| FR-3.3 | Must | TC-2.02 | Covered |
| FR-3.4 | Must | TC-3.05 | Covered |
| FR-3.5 | Should | TC-2.06 | Covered |
| FR-4.1 | Must | TC-2.04, TC-3.01 | Covered |
| FR-4.2 | Must | TC-3.09 | Covered |
| FR-4.3 | Must | TC-3.03 | Covered |
| FR-4.4 | Must | TC-3.02, TC-3.06 | Covered |
| FR-4.5 | Must | TC-3.04, TC-3.11, TC-3.13 | Covered |
| FR-4.6 | Should | TC-3.03 | Covered |
| FR-4.7 | May | — | — |
| FR-5.1 | Must | TC-3.01 | Covered |
| FR-5.2 | Must | TC-3.01 | Covered |
| FR-5.3 | Must | TC-2.02, TC-2.05 | Covered |
| FR-5.4 | Must | TC-3.01, TC-3.09 | Covered |
| FR-5.5 | Must | TC-3.02, TC-3.06 | Covered |
| FR-5.6 | Must | TC-2.05 | Covered |
| FR-5.7 | Should | TC-3.08 | Covered |
| FR-5.8 | Should | — | GAP |
| FR-6.1 | Must | TC-3.05 | Covered |
| FR-6.2 | Must | AT-02 | Covered |
| FR-6.3 | Must | TC-3.06 | Covered |
| FR-6.4 | Should | TC-2.06 | Covered |
| FR-6.5 | Must | TC-4.01 | Covered |
| FR-6.6 | Should | TC-3.07 | Covered |
| FR-6.7 | Must | TC-4.05 | Covered |
| FR-7.1 | Must | TC-4.05 | Covered |
| FR-7.2 | Must | TC-4.01 | Covered |
| FR-7.3 | Must | TC-4.03 | Covered |
| FR-7.4 | Must | TC-4.01, TC-4.06 | Covered |
| FR-7.5 | Must | TC-4.04 | Covered |
| FR-7.6 | Should | TC-4.02 | Covered |
| NFR-1.1 | Must | TC-1.02 | Covered |
| NFR-1.2 | Must | TC-2.05 | Covered |
| NFR-1.3 | Must | AT-03 | Covered |
| NFR-2.1 | Must | — | GAP |
| NFR-2.2 | Must | TC-3.02 | Covered |
| NFR-3.1 | Must | TC-2.07 | Covered |
| NFR-3.2 | Should | TC-1.01 | Covered |
| FR-8.1 | Must | TC-2.09 | Covered |
| FR-8.2 | Must | TC-2.10, TC-3.10 | Covered |
| FR-8.3 | Must | TC-2.10, TC-2.02 | Covered |
| FR-8.4 | Must | TC-3.11 | Covered |
| FR-8.5 | Must | TC-3.13 | Covered |
| FR-8.6 | Must | TC-3.12 | Covered |
| FR-8.7 | Must | TC-3.13 | Covered |
| FR-8.8 | Should | TC-3.10 | Covered |
| FR-8.9 | Should | TC-3.13 | Covered |
| NFR-4.1 | Must | TC-3.04, TC-3.11 | Covered |
| NFR-4.2 | Must | TC-4.04 | Covered |
| NFR-5.1 | Should | TC-4.07 | Covered |
| NFR-5.2 | May | AT-04 | Covered |
| FR-9.1 | Must | TC-3.15 | Covered |
| FR-9.2 | Must | TC-3.15, TC-3.18 | Covered |
| FR-9.3 | Must | TC-3.15 | Covered |
| FR-9.4 | Must | TC-3.15 | Covered |
| FR-9.5 | Must | TC-3.17 | Covered |
| FR-9.6 | Must | TC-3.16 | Covered |
| FR-9.7 | Must | TC-3.17 | Covered |
| FR-9.8 | Should | TC-3.19 | Covered |
| FR-9.9 | Should | TC-3.18 | Covered |
| FR-10.1 | Must   | TC-3.20 | Covered |
| FR-10.2 | Must   | TC-3.23 | Covered |
| FR-10.3 | Must   | TC-3.20, TC-3.21, TC-3.22 | Covered |
| FR-10.4 | Should | —       | GAP (forward-compat enum; covered by code review) |
| FR-10.5 | Must   | TC-3.20 | Covered |
| FR-10.6 | Must   | TC-3.20 | Covered |
| FR-10.7 | Should | TC-3.20 | Covered (Kconfig wired at build time) |
| FR-10.8 | Should | —       | GAP (implementation detail; code review) |
| NFR-6.1 | Should | TC-3.11, TC-3.12 | Covered |
| NFR-6.2 | Should | TC-3.13, AT-03 | Covered |
| FR-11.1 | Must   | TC-3.24, TC-3.25 | Covered |
| FR-11.2 | Must   | TC-3.24 | Covered |
| FR-11.3 | Must   | TC-3.24, TC-3.25, TC-3.26 | Covered |
| FR-11.4 | Must   | TC-3.27 | Covered |
| FR-11.5 | Should | —       | GAP (thread-safety: code review) |
| FR-11.6 | Should | TC-3.26 | Covered |
| FR-12.1 | Must   | TC-3.30, TC-3.31, TC-3.32, TC-3.33, TC-3.34, TC-3.35 | Covered |
| FR-12.2 | Must   | TC-3.28, TC-3.29 | Covered |
| FR-12.3 | Must   | TC-3.28 | Covered |
| FR-12.4 | Must   | TC-3.30 | Covered |
| FR-12.5 | Must   | TC-3.31 | Covered |
| FR-12.6 | Must   | TC-3.35 | Covered |
| FR-12.7 | Should | TC-3.30 | Covered |
| FR-12.8 | Should | —       | GAP (implementation detail; code review) |

**GAP items:**
- **FR-5.8** (Subscriber callbacks from dedicated notify task, not ISR): requires code review / design verification rather than a black-box test. Add a code-review checklist item.
- **NFR-2.1** (No blocking polling loops on ZNP receive): same as above — requires static analysis or code review, not a functional test. Add a code-review checklist item.
- **FR-11.5** (`zb_network_info_get` thread-safety): implementation must read only module-static variables updated by a single writer task; verify by code review.
- **FR-12.8** (Extended schemas registered at init via `zb_schema_register()`): implementation detail confirmed by TC-3.18 pattern; verify by code review.

---

## 9. Troubleshooting Guide

### 9.1 Boot / Initialization Issues

| Symptom | Likely Cause | Diagnostic Steps | Corrective Action |
|---------|-------------|-----------------|-------------------|
| Stuck in ZNP_INIT indefinitely | CC2652P7 not powered, UART wired wrong, or running wrong firmware | Check power rails; monitor UART RX pin with logic analyzer; verify GPIO connections | Re-wire UART; re-flash CC2652P7 |
| `SYS_PING` times out (3 s) | Wrong baud rate, swapped TX/RX, or coprocessor in BSL mode | Check `uart_baud` in `/zb/config.json`; verify UART polarity; check BSL pin state | Swap TX/RX wires; release BSL pin; match baud rates |
| NETWORK_CHECK fails / LittleFS mount error | Storage partition missing or corrupt | Check serial for "LittleFS mount failed"; verify `partitions.csv` has `storage` entry | Run `idf.py erase-flash`; reflash; re-provision |
| State machine loops FORMING→NETWORK_CHECK | Coordinator start fails (channel busy or key issue) | Check serial log for `ZDO_STARTUP_FROM_APP` error code | Try different channel; verify network key format |

### 9.2 ZNP Communication Errors

| Symptom | Likely Cause | Diagnostic Steps | Corrective Action |
|---------|-------------|-----------------|-------------------|
| Frequent FCS errors | Electrical noise, baud rate mismatch, missing level shifter | Inspect transport error counter; check signal quality | Add 100 Ω series resistor on UART lines; verify 3.3 V signaling |
| SREQ timeout with no SRSP | Coprocessor busy processing heavy ZNP traffic | Increase per-command timeout; check AREQ queue depth | Throttle command rate; increase timeout constant |
| UART overrun (RX buffer full) | Insufficient RX task priority or dense AREQ burst | Check `znp_uart_rx_task` high-water mark | Increase ring buffer size; raise task priority |
| Repeated `SYS_RESET_IND` in READY | Coprocessor watchdog kicking due to assert or stack overflow | Check Z-Stack NV health; review reset reason in AREQ payload | Re-flash CC2652P7 with known-good firmware |

### 9.3 Zigbee Network Issues

| Symptom | Likely Cause | Diagnostic Steps | Corrective Action |
|---------|-------------|-----------------|-------------------|
| Devices cannot join | Permit-join not open, wrong channel, or PAN ID conflict | Verify `zb_cmd_permit_join(60)` called; check channel; scan for PAN conflicts | Open permit-join; change channel |
| Device joins then immediately leaves | Trust center reject (wrong key or install code) | Check serial log for TC reject code | Verify network key matches; check device's join credentials |
| Attribute reports stop arriving | Device left network silently or link quality degraded | Check `last_seen_ms` in device record; check RF link quality | Re-join device; improve antenna placement |
| Registry empty after reboot | LittleFS partition corrupt, or no `.pb` files in `/zb/devices/` | Check serial for "loaded N device(s)" at boot; list files if accessible | Run `esp_littlefs_format("storage")`; reboot; re-join all devices |

### 9.4 Firmware Update Issues

| Symptom | Likely Cause | Diagnostic Steps | Corrective Action |
|---------|-------------|-----------------|-------------------|
| BSL entry fails | BSL_INVOKE GPIO not connected, or GPIO driven too late relative to RESET | Verify GPIO wiring; add oscilloscope check on RESET + BSL timing | Re-wire; adjust GPIO timing in `zb_fw_update.c` |
| `ZB_EVENT_FW_UPDATE_FAILED` after 0% progress | BSL handshake failed (wrong baud, noise) | Check serial for BSL NAK bytes | Reduce BSL baud rate; improve signal integrity |
| Coprocessor unresponsive after failed update | Partial flash; coprocessor in BSL wait mode | Send reset pulse via GPIO; coprocessor will re-enter BSL | Retry update; if still unresponsive, use external programmer |
| Update stalls at N% | USB power insufficient for flash write current | Monitor supply voltage during update | Use dedicated 3.3 V supply with ≥ 500 mA capacity |

---

## 10. Appendix

### 10.1 Configuration Constants

| Constant | Default | Description |
|----------|---------|-------------|
| `ZNP_UART_BAUD` | 115200 | UART baud rate for ZNP protocol |
| `ZNP_SREQ_TIMEOUT_MS` | 3000 | Per-command SREQ timeout in ms |
| `ZNP_RESET_TIMEOUT_MS` | 3000 | Wait for `SYS_RESET_IND` after hard reset |
| `ZNP_RESYNC_MAX_RETRIES` | 5 | Max SOF-hunt attempts before error |
| `ZB_DEFAULT_CHANNEL` | 15 | Default Zigbee channel (2.4 GHz) |
| `ZB_DEFAULT_PAN_ID` | 0x1A2B | Default 16-bit PAN ID |
| `ZB_PERMIT_JOIN_MAX_S` | 254 | Max permit-join duration (seconds) |
| `ZB_MAX_DEVICES` | 100 | Maximum registry entries |
| `ZB_MAX_ENDPOINTS_PER_DEV` | 8 | Per-device endpoint table size |
| `ZB_MAX_CLUSTERS_PER_EP` | 16 | Per-endpoint cluster table size |
| `ZB_MAX_ATTRS_PER_CLUSTER` | 8 | Per-cluster attribute cache entries |
| `ZB_NOTIFY_TASK_STACK` | 4096 | Notify task stack size (bytes) |
| `ZB_FRAMEWORK_TASK_STACK` | 8192 | Framework task stack size (bytes) |
| `ZNP_AREQ_QUEUE_DEPTH` | 32 | AREQ event queue depth (frames) |
| `ZB_CMD_QUEUE_DEPTH` | 16 | Command queue depth |
| `ZB_STORAGE_PARTITION_LABEL` | `"storage"` | LittleFS partition label in `partitions.csv` |
| `ZB_STORAGE_MOUNT_POINT` | `"/zb"` | VFS base path for all storage files |
| `ZB_STORAGE_FORMAT_ON_FAIL` | `y` | Format partition if mount fails (safe default) |
| `ZB_DEV_PB_MAX_SIZE` | 1024 | Max encoded `ZbDeviceRecord` buffer size (bytes) |

### 10.2 ZNP MT Subsystem Reference (Used by This Project)

| Subsystem | ID | Commands Used |
|-----------|-----|--------------|
| SYS | 0x21 | `SYS_PING`, `SYS_VERSION`, `SYS_RESET_REQ`, `SYS_RESET_IND` |
| APP_CNF | 0x26 | `APP_CNF_BDB_START_COMMISSIONING` |
| ZDO | 0x25 | `ZDO_STARTUP_FROM_APP`, `ZDO_MGMT_PERMIT_JOIN_REQ`, `ZDO_TC_DEV_IND`, `ZDO_LEAVE_IND`, `ZDO_END_DEVICE_ANNCE_IND` |
| AF | 0x24 | `AF_REGISTER`, `AF_DATA_REQUEST`, `AF_INCOMING_MSG` |
| ZNP | 0x0F | `ZB_APP_REGISTER_REQUEST`, `ZB_START_REQUEST` |

### 10.3 LittleFS File Layout

| Path | Format | Content |
|------|--------|---------|
| `/zb/config.json` | JSON (cJSON) | `pan_id`, `channel`, `nwk_key` (hex), `uart_baud` |
| `/zb/devices/` | Directory | One `.pb` file per registered device |
| `/zb/devices/<16-hex>.pb` | nanopb binary | `ZbDeviceRecord` protobuf (ieee_addr, nwk_addr, endpoints, clusters, attrs) |

**Partition table entry** (`partitions.csv`):
```
# Name,  Type, SubType, Offset,   Size
storage, data, spiffs,  0x610000, 0x9F0000   # ~11 MB LittleFS
```

**NVS** (system partition, 24 KB at `0x9000`) is reserved for ESP-IDF internals only.
No application data is stored in NVS.

### 10.4 Zigbee Channels (2.4 GHz) Reference

| Channel | Center Freq | Wi-Fi Overlap |
|---------|------------|---------------|
| 11 | 2405 MHz | Overlaps Wi-Fi ch 1 |
| 15 | 2425 MHz | Clean of Wi-Fi ch 1, 6, 11 **(default)** |
| 20 | 2450 MHz | Overlaps Wi-Fi ch 11 |
| 25 | 2475 MHz | Clean (most regions) |
| 26 | 2480 MHz | Restricted TX power in some regions |

### 10.5 Example Boot Log (Expected)

```
I ZB: Initializing framework
I zb_storage: LittleFS mounted: /zb  total=10420224 used=4096
I ZNP: UART init baud=115200 port=1
I ZNP: Resetting CC2652P7 via GPIO4
I ZNP: [ZNP_INIT] SYS_RESET_IND received (reason=0x00 powerup)
I ZNP: [ZNP_INIT] SYS_PING OK, version=Z-Stack 3.6.0
I ZB: [NETWORK_CHECK] loading config from /zb/config.json
I ZB_STORAGE_CFG: config loaded: pan_id=0x1A2B channel=15 baud=115200
I ZB: [NETWORK_CHECK] CC2652P7 NV matches desired config → resuming
I ZB: [FORMING] ZDO_STARTUP_FROM_APP OK
I ZB: [READY] Coordinator started, network ready
I ZB: loading device registry from storage...
I zb_storage_dev: loaded device 00124b001234abcd (48 bytes, 1 endpoint(s))
I zb_storage_dev: loaded device 00124b00abcd1234 (96 bytes, 2 endpoint(s))
I ZB: device registry restored: 2 device(s)
I ZB_NOTIFY: ZB_EVENT_NETWORK_READY delivered to 1 subscriber(s)
```

**First-boot variant** (no `config.json` yet):
```
I zb_storage_cfg: config file not found (/zb/config.json) — first boot
I ZB: [NETWORK_CHECK] CC2652P7 NV empty — needs network formation
I ZB: [FORMING] PAN_ID=0x1A2B channel=15
I ZB: [FORMING] ZDO_STARTUP_FROM_APP OK
I ZB_STORAGE_CFG: config saved: pan_id=0x1A2B channel=15
I ZB: device registry restored: 0 device(s)
```

### 10.6 ZCL Attribute Report — Example Payload

```
ZB_EVENT_ATTR_REPORT:
  ieee_addr  = 0x00124B001234ABCD
  ep         = 1
  cluster_id = 0x0006  (OnOff)
  attr_id    = 0x0000  (OnOff)
  data_type  = 0x10    (Boolean)
  value      = 0x01    (On)
  updated_ms = 123456789
```

---

## 11. Related

- `[[zigbee-hub-mqtt-bridge-fsd]]` — MQTT upper-layer integration (Phase 4 extension)
- `[[zigbee-hub-matter-bridge-fsd]]` — Matter bridge over Wi-Fi (future)
- `[[cc2652p7-znp-firmware-build]]` — Z-Stack build and flashing runbook
