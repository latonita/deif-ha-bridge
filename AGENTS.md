# AGENTS.md - Technical Documentation for AI Assistants

## Project Overview

This is a **Node.js bridge application** that connects a **DEIF GC-1F/2 generator controller** (via Modbus RTU) to **Home Assistant** (via MQTT). It continuously polls the controller for measurements, alarms, and status data, then publishes this information in a format that Home Assistant can automatically discover and display.

## Original Task

**Objective**: Enable real-time monitoring of a diesel generator controller in Home Assistant without custom integrations.

**Requirements**:
- Read all relevant registers from DEIF GC-1F/2 controller via Modbus RTU (RS485)
- Expose 3-phase electrical measurements (voltage, current, frequency, power, energy)
- Monitor alarm states with human-readable descriptions
- Track operational status (engine running, breakers, operating mode)
- Provide Home Assistant auto-discovery via MQTT
- Categorize entities (primary vs diagnostic) for clean UI

## How It Works

### Architecture

```
DEIF GC-1F/2 Controller (Modbus RTU/RS485)
          ↓
    [Serial /dev/ttyUSB0]
          ↓
  deif_to_mqtt.js (this app)
          ↓
    [MQTT Broker]
          ↓
    Home Assistant
```

### Data Flow

1. **Connect Phase**
   - Establishes serial connection to DEIF controller (9600 baud, 8N1)
   - Connects to MQTT broker
   - Publishes Home Assistant discovery messages (one-time, retained)

2. **Poll Cycle** (every 5 seconds by default)
   - Reads Modbus input registers 500-576 (measurements)
   - Reads Modbus input registers 1000-1019 (alarms & status bitfields)
   - Processes data and publishes single JSON state message to `deif/gc1f2/state`

3. **Home Assistant Integration**
   - HA discovers 50+ sensors automatically via retained MQTT discovery messages
   - Sensors update in real-time as new state messages arrive

### Register Mapping

**Measurement Registers (500-576)**:
- 500: Application version (formatted as X.Y.ZZ)
- 504-506: Generator voltage L1-N, L2-N, L3-N (V)
- 507: Generator frequency (Hz, divided by 10)
- 513-515: Generator current L1, L2, L3 (A)
- 519/523/527: Active/Reactive/Apparent power (kW/kVAr/kVA, signed 16-bit)
- 536-537: Energy counter (32-bit unsigned, kWh)
- 538: Power factor (cos φ × 100, signed)
- 542-545: Mains voltage L1-N, L2-N, L3-N, frequency
- 554-555: Running hours (32-bit unsigned, hours)
- 558-560: Alarm counters (total, unacknowledged, acknowledged active)
- 563-566: Operation counters (breaker ops, start attempts)
- 567: Battery/supply voltage (V, divided by 10)
- 576: Engine RPM

**Alarm & Status Registers (1000-1019)**:
- 1000-1005: Protection alarm bitfields
- 1006-1009: Reserved/unused for alarms
- 1010-1011: Multi-input protection alarms
- 1012: Reserved/unused for alarms
- 1013-1015: Analogue input alarms, system alarms
- 1016-1017: Relay output states (not used for active alarms)
- 1018-1019: Status bitfields (NOT alarms - operational state)

### Key Functions

**`decodeAlarms(alarmRegs)`**
- Processes ONLY alarm registers: 1000-1005, 1010, 1011, 1013, 1014, 1015
- Skips status registers 1018-1019 and unused registers
- Checks each bit (0-15) in alarm registers
- Looks up alarm definitions in `ALARM_AND_STATUS_MAP`
- Returns array of active alarm objects with code and description

**`decodeStatus(alarmRegs)`**
- Processes ONLY status registers: 1018, 1019
- Extracts boolean flags for operational state
- Returns object with keys like `1018_0`, `1018_1`, etc.

**`getOperatingModeText(status)`**
- Determines primary mode with priority: OFF > Manual > Test > Auto
- Appends modifiers (AMF, Load Takeover, AMF Active) if present
- Returns formatted string like "Auto (AMF, Load Takeover)"

**`formatActiveAlarms(activeAlarms)`**
- Converts alarm array to multi-line text format
- Each line: "CODE Description"
- Returns "No active alarms" if array is empty

**`publishHassDiscovery(mq)`**
- Creates ~50 sensor and binary sensor entities
- Uses MQTT discovery protocol for auto-configuration
- Categorizes entities: primary (visible) vs diagnostic (hidden)
- All discovery messages are retained for HA restart resilience

### Data Transformations

1. **32-bit counters**: Combine two 16-bit registers (high/low) using big-endian
2. **Signed values**: Convert unsigned 16-bit to signed using two's complement
3. **Frequency**: Divide by 10, format to 1 decimal place
4. **Voltage (battery)**: Divide by 10, format to 1 decimal place
5. **Power factor**: Divide by 100 (stored as cos φ × 100)
6. **App version**: 2220 → "2.2.20" (split digits)
7. **Alarm bitfields**: Convert to hex string (0x1234)

### Entity Categories

**Primary Entities** (visible in main UI):
- Generator & mains measurements (V, A, Hz, kW)
- Run hours, energy produced
- Active alarms text
- Battery voltage
- Operating mode text
- Critical binary sensors (mains failure, engine running, breakers, unacknowledged alarms)

**Diagnostic Entities** (hidden by default):
- App version, device model, timestamp
- Alarm counters, operation counters
- Individual alarm bitfields (hex)
- Individual operating mode flags
- Engine RPM
- Signed energy value

### Configuration

Environment variables (`.env` file):
```bash
DEIF_PORT=/dev/ttyUSB0        # Serial port
DEIF_BAUD=9600                # Baud rate
DEIF_SLAVE=1                  # Modbus slave ID

MQTT_URL=mqtt://localhost:1883
MQTT_USER=                    # Optional
MQTT_PASS=                    # Optional

TOPIC_PREFIX=deif/gc1f2       # MQTT topic prefix
INTERVAL_MS=5000              # Polling interval
RETAIN=true                   # Retain state messages

HASS_DISCOVERY_PREFIX=homeassistant
HASS_NODE_ID=deif-gc1f2-1
```

## Critical Implementation Details

### Why Alarm/Status Separation Matters

**Problem**: Registers 1018-1019 contain operational status (engine running, breaker positions, operating mode), NOT alarm conditions. Including them in active alarms text creates false positives.

**Solution**: `decodeAlarms()` processes only actual alarm registers (1000-1005, 1010-1011, 1013-1015). Status registers are handled separately by `decodeStatus()`.

### MQTT State Structure

Published to `deif/gc1f2/state`:
```json
{
  "device": { "id": "...", "name": "...", "model": "...", "manufacturer": "..." },
  "app_version": "2.2.20",
  "gen": {
    "voltage_l1n_v": 230,
    "voltage_l2n_v": 231,
    "voltage_l3n_v": 229,
    "current_l1_a": 15,
    "current_l2_a": 14,
    "current_l3_a": 16,
    "frequency_hz": 50.0,
    "pgen_kw": 10,
    "qgen_kvar": 2,
    "sgen_kva": 11,
    "cos_phi": 0.95
  },
  "mains": { "voltage_l1n_v": 230, "voltage_l2n_v": 231, "voltage_l3n_v": 229, "frequency_hz": 50.0 },
  "run_hours": 12345,
  "energy_kwh": 54321,
  "energy_signed_kwh": -54321,
  "alarms": {
    "count": 0,
    "unacknowledged": 0,
    "ack_active": 0,
    "bitfield": { "1000": "0x0000", "1001": "0x0000", ... },
    "active": [],
    "active_text": "No active alarms"
  },
  "counters": { "gen_breaker_ops": 100, "mains_breaker_ops": 50, "start_attempts": 200 },
  "status": {
    "1018_0": false,
    "1018_1": true,
    "1018_4": false,
    "1018_6": false,
    "1018_7": false,
    "1018_8": false,
    "1019_0": false,
    "1019_1": false,
    "1019_3": true,
    "1019_4": false,
    "1019_5": false,
    "1019_6": true,
    "1019_10": false,
    "1019_15": true,
    "operating_mode": "Auto (AMF, AMF Active)"
  },
  "rpm": 1500,
  "usupply_v": 13.8,
  "ts": "2026-01-05T12:34:56.789Z"
}
```

### Home Assistant Jinja2 Templates

**Important**: JSON numeric keys require bracket notation:
```jinja2
{{ value_json.alarms.bitfield['1000'] }}  ✓ Correct
{{ value_json.alarms.bitfield.1000 }}     ✗ Wrong (syntax error)
```

## Common Modifications

### Adding a New Sensor
1. Add register address to `R` constant
2. Read register in `readAndPublish()` function
3. Add to published state object
4. Add sensor definition to `publishHassDiscovery()` sensors array

### Enabling Commands (Alarm Ack / Mode)
- Commands are disabled by default; set `ENABLE_COMMANDS=true` (optional `CMD_COOLDOWN_MS=5000`) to expose MQTT command topics and HA discovery buttons.
- Command topics (write-only, no retain):
  - Alarm acknowledge: `<TOPIC_PREFIX>/cmd/alarm_ack`
  - Mode Manual: `<TOPIC_PREFIX>/cmd/mode_manual`
  - Mode Auto: `<TOPIC_PREFIX>/cmd/mode_auto`
- The app enforces a global cooldown between any commands and ignores retained command messages. Secure MQTT with auth/ACLs to restrict publishes to these topics.

### Changing Polling Interval
Set `INTERVAL_MS` environment variable (milliseconds)

### Adding Alarm Definitions
Add entries to `ALARM_AND_STATUS_MAP` with format `"register:bit": { code: "...", text: "..." }`

## Dependencies

- `modbus-serial`: Modbus RTU/TCP client
- `mqtt`: MQTT client
- `dotenv`: Environment variable loader

## Error Handling

- Modbus timeout: 1000ms (configurable in code)
- Poll errors are logged but don't crash the application
- MQTT connection errors exit the process
- Modbus connection errors exit the process

## Testing

Set `INTERVAL_MS=0` to run once and exit (useful for testing without loops)

## Maintenance Notes

- All Home Assistant discovery messages are retained
- State messages use `qos: 0` for performance
- State retention controlled by `RETAIN` environment variable
- Application version from controller is auto-formatted for display
- Energy counter must be unsigned for Home Assistant energy dashboard
- Signed energy value preserved in `energy_signed_kwh` for reference
- `PUBLISH_INDIVIDUAL_TOPICS` controls per-metric MQTT publishing (default true). Set to false to publish only the consolidated `state` message.

## Container Image Notes

- Dockerfile uses a multi-stage build with a distroless runtime to minimize size.
- Builder stage keeps native toolchain; runtime stage includes only app, deps, and minimal passwd/group with `dialout` for serial access.
