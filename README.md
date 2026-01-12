# DEIF GC-1F/2 to HASS via MQTT Bridge

Bridge the DEIF GC-1F/2 generator controller (Modbus RTU) to MQTT for automatic Home Assistant discovery.

## Features
- Polls measurement registers 500-576 and alarm/status registers 1000-1019.
- Publishes a consolidated JSON state to `TOPIC_PREFIX/state`.
- Sends retained Home Assistant discovery for 50+ sensors/binary sensors.
- Separates alarms from status bits and formats active alarm text.
- Minimal distroless container image with dialout group support for serial devices.

## Requirements
- DEIF GC-1F/2 controller on RS-485.
- MQTT broker reachable from the container/host.
- Node.js 20+ if running locally (Docker image includes runtime).

## Commands (optional, gated)
- Disabled by default. Set `ENABLE_COMMANDS=true` to expose command topics and Home Assistant buttons for:
  - Alarm acknowledge → `<TOPIC_PREFIX>/cmd/alarm_ack`
  - Mode Manual → `<TOPIC_PREFIX>/cmd/mode_manual`
  - Mode Auto → `<TOPIC_PREFIX>/cmd/mode_auto`
- Global cooldown between any commands via `CMD_COOLDOWN_MS` (default 5000ms).
- Retained command messages are ignored. Secure your MQTT broker/ACLs so only trusted clients can publish to `.../cmd/#`.

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
PUBLISH_INDIVIDUAL_TOPICS=true

HASS_DISCOVERY_PREFIX=homeassistant
HASS_NODE_ID=deif-gc1f2-1
```
Key notes:
- Frequency divisor/decimals can be tuned via `FREQ_DIVISOR` and `FREQ_DECIMALS` if your device scales differently.
- `INTERVAL_MS=0` runs once and exits (useful for tests).
- Status registers 1018-1019 are published under `status.*`, not treated as alarms.
- Set `PUBLISH_INDIVIDUAL_TOPICS=false` to skip per-metric topics and publish only the consolidated `state` payload.

## Run with Docker Compose
```
docker compose up --build
```
The provided `docker-compose.yml` maps `/dev/ttyUSB0` and adds the `dialout` group (GID 20 on most Debian hosts). If your dialout GID differs, set `group_add` to that numeric GID.

## Build/Run Container Manually
Build the minimal image:
```
docker build -t deif-mqtt .
```
Run with the serial device mapped and dialout group:
```
docker run --rm \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  --group-add dialout \
  --env-file .env \
  deif-mqtt
```

## Run Locally (without Docker)
```
npm ci --omit=dev
node deif_to_mqtt.js
```
Ensure your user has permission to `/dev/ttyUSB0` (often by being in the `dialout` group).

## Home Assistant
- Discovery messages are retained; entities appear automatically under the configured `HASS_DISCOVERY_PREFIX`.
- Numeric keys in JSON require bracket notation in templates: `{{ value_json.alarms.bitfield['1000'] }}`.

## Troubleshooting
- Serial permission: verify the container has `dialout` (or correct GID) and the device is mapped.
- MQTT connection failures exit the process; check broker URL/credentials.
- Modbus timeouts are logged and retried on the next poll.
