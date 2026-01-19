# deif-ha-bridge (DEIF GC-1F/2 to HASS via MQTT)

Bridge the DEIF GC-1F/2 generator controller (Modbus RTU) to MQTT so Home Assistant can discover and track everything without custom code. Repo: https://github.com/latonita/deif-ha-bridge

## Features
- Fresh data every few seconds: power, energy, run hours, alarms, status, and more.
- Per-metric topics only (no consolidated state payload). Key paths: `gen/*`, `mains/*`, `engine/*` (includes RPM/battery/last-run data), `alarms/*`, `status/*`, `counters/*` (run hours, energy, breaker counters).
- Home Assistant auto-discovery for 50+ sensors/binary sensors (retained for restart resilience).
- Clear alarm handling: separates alarms from status bits and prints active alarms in plain text.
- Optional control buttons for alarm acknowledge, start/stop/breakers, and Manual/Auto/Test modes, guarded by a global cooldown and safe handling of retained MQTT messages.
- Built for small, secure deployments (distroless image, dialout-ready for serial devices).
- Tracks and retains engine last-run timestamps and last run duration when start/stop status changes; retains last alarm timestamps/summary and a formatted "Last Alarm" text.

## Requirements
- DEIF GC-1F/2 controller on RS-485.
- MQTT broker reachable from the container/host.
- Node.js 20+ if running locally (Docker image includes runtime).

## Getting Started
1) Clone: `git clone https://github.com/latonita/deif-ha-bridge.git && cd deif-ha-bridge`
2) Configure: create `.env` using the variables below.
3) Run:
   - Docker Compose: `docker compose up --build -d`
   - Docker: `docker build -t deif-ha-bridge .` then `docker run --rm --device /dev/ttyUSB0:/dev/ttyUSB0 --group-add dialout --env-file .env deif-ha-bridge`
   - Local: `npm ci --omit=dev && node deif_to_mqtt.js`

## Commands (optional, per-flag)
- Opt-in per action: set `ENABLE_COMMAND_<NAME>=true`. Only enabled commands publish HA buttons and listen on MQTT.
- Topics and flags:
  - Alarm acknowledge → `<TOPIC_PREFIX>/cmd/alarm_ack` (`ENABLE_COMMAND_ALARM_ACK`)
  - Start → `<TOPIC_PREFIX>/cmd/start` (`ENABLE_COMMAND_START`)
  - GB ON / GB OFF / Stop → `<TOPIC_PREFIX>/cmd/gb_on`, `/cmd/gb_off`, `/cmd/stop` (`ENABLE_COMMAND_GB_ON`, `ENABLE_COMMAND_GB_OFF`, `ENABLE_COMMAND_STOP`)
  - Start + GB ON / GB OFF + Stop → `<TOPIC_PREFIX>/cmd/start_gb_on`, `/cmd/gb_off_stop` (`ENABLE_COMMAND_START_GB_ON`, `ENABLE_COMMAND_GB_OFF_STOP`)
  - MB ON / MB OFF → `<TOPIC_PREFIX>/cmd/mb_on`, `/cmd/mb_off` (`ENABLE_COMMAND_MB_ON`, `ENABLE_COMMAND_MB_OFF`)
  - Mode Manual / Mode Auto / Mode Test → `<TOPIC_PREFIX>/cmd/mode_manual`, `/cmd/mode_auto`, `/cmd/mode_test` (`ENABLE_COMMAND_MANUAL_MODE`, `ENABLE_COMMAND_AUTO_MODE`, `ENABLE_COMMAND_TEST`)
- Global cooldown across all commands via `CMD_COOLDOWN_MS` (default 5000ms).
- Retained command messages are ignored. Secure your MQTT broker/ACLs so only trusted clients can publish to `.../cmd/#`.

See `COMMANDS.md` for a concise command/register summary table.

## Configuration
Set environment variables (example `.env`):
```
DEIF_PORT=/dev/ttyUSB0
DEIF_BAUD=9600
DEIF_SLAVE=1

MQTT_URL=mqtt://mqtt-broker:1883
MQTT_USER=
MQTT_PASS=

TOPIC_PREFIX=deif/gc1f2
INTERVAL_MS=5000
RETAIN=true
PUBLISH_ALARM_BITFIELDS=false
ENABLE_COMMAND_ALARM_ACK=false
ENABLE_COMMAND_START=false
ENABLE_COMMAND_GB_ON=false
ENABLE_COMMAND_GB_OFF=false
ENABLE_COMMAND_STOP=false
ENABLE_COMMAND_START_GB_ON=false
ENABLE_COMMAND_GB_OFF_STOP=false
ENABLE_COMMAND_MB_ON=false
ENABLE_COMMAND_MB_OFF=false
ENABLE_COMMAND_MANUAL_MODE=false
ENABLE_COMMAND_AUTO_MODE=false
ENABLE_COMMAND_TEST=false

HASS_DISCOVERY_PREFIX=homeassistant
HASS_NODE_ID=deif-gc1f2-1
```
Key notes:
- Frequency divisor/decimals can be tuned via `FREQ_DIVISOR` and `FREQ_DECIMALS` if your device scales differently.
- `INTERVAL_MS=0` runs once and exits (useful for tests).
- Status registers 1018-1019 are published under `status.*`, not treated as alarms.
- Discovery templates point to per-metric topics; no consolidated `state` payload is published (per-metric topics are always on). Last run/alarm timestamps and Last Alarm text are retained.

## Run with Docker Compose
```
docker compose up --build
```
The provided `docker-compose.yml` maps `/dev/ttyUSB0` and adds the `dialout` group (GID 20 on most Debian hosts). If your dialout GID differs, set `group_add` to that numeric GID.

## Build/Run Container Manually
Build the minimal image:
```
docker build -t deif-ha-bridge .
```
Run with the serial device mapped and dialout group:
```
docker run --rm \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  --group-add dialout \
  --env-file .env \
  deif-ha-bridge
```

## Run Locally (without Docker)
```
npm ci --omit=dev
node deif_to_mqtt.js
```
Ensure your user has permission to `/dev/ttyUSB0` (often by being in the `dialout` group).

## Home Assistant
- Discovery messages are retained; entities appear automatically under the configured `HASS_DISCOVERY_PREFIX`.
- Alarm bitfields are only published (and discovered) when `PUBLISH_ALARM_BITFIELDS=true`.
- Numeric keys in JSON require bracket notation in templates: `{{ value_json.alarms.bitfield['1000'] }}`.

## Troubleshooting
- Serial permission: verify the container has `dialout` (or correct GID) and the device is mapped.
- MQTT connection failures exit the process; check broker URL/credentials.
- Modbus timeouts are logged and retried on the next poll.

## License
- MIT; see `LICENSE`.
