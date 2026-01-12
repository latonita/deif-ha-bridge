# Command & Register Summary

This project polls Modbus input registers from the DEIF GC-1F/2 controller and publishes the data to MQTT for Home Assistant. Commands flow the opposite direction: MQTT button topics trigger Modbus coil writes. The tables below summarize what is read and where it is published, plus how commands are routed.

## Polled Registers (FC04)

| Register | Purpose | MQTT Field (in `state`) |
| --- | --- | --- |
| 500 | Application version (X.Y.ZZ) | `app_version` |
| 504-506 | Generator voltage L1/L2/L3-N (V) | `gen.voltage_l{1..3}n_v` |
| 507 | Generator frequency (Hz ÷ `FREQ_DIVISOR`) | `gen.frequency_hz` |
| 513-515 | Generator current L1/L2/L3 (A) | `gen.current_l{1..3}_a` |
| 519 / 523 / 527 | Generator active / reactive / apparent power (kW/kVAr/kVA, signed) | `gen.pgen_kw`, `gen.qgen_kvar`, `gen.sgen_kva` |
| 536-537 | Energy counter (32-bit, kWh, unsigned) | `energy_kwh`, `energy_signed_kwh` (-copy) |
| 538 | Power factor (cos φ × 100, signed) | `gen.cos_phi` |
| 542-544 | Mains voltage L1/L2/L3-N (V) | `mains.voltage_l{1..3}n_v` |
| 545 | Mains frequency (Hz ÷ `FREQ_DIVISOR`) | `mains.frequency_hz` |
| 554-555 | Running hours (32-bit) | `run_hours` |
| 558-560 | Alarm counters (total, unack, ack active) | `alarms.count`, `alarms.unacknowledged`, `alarms.ack_active` |
| 563-564 / 566 | Generator breaker ops / mains breaker ops / start attempts | `counters.gen_breaker_ops`, `counters.mains_breaker_ops`, `counters.start_attempts` |
| 567 | Battery/PSU voltage (V ÷ 10) | `usupply_v` |
| 576 | Engine RPM | `rpm` |

| Register Range | Purpose | MQTT Field (in `state`) |
| --- | --- | --- |
| 1000-1005, 1010-1011, 1013-1015 | Alarm bitfields; decoded to human-readable alarms | `alarms.bitfield['<reg>']` (hex), `alarms.active`, `alarms.active_text` |
| 1018-1019 | Status bitfields (operating mode, mains failure, breaker/engine state) | `status['1018_#']`, `status['1019_#']`, `status.operating_mode` |

All polled data is published to `TOPIC_PREFIX/state` as a single JSON. When `PUBLISH_INDIVIDUAL_TOPICS=true`, each leaf field is also published under `TOPIC_PREFIX/<path>` (e.g., `deif/gc1f2/gen/voltage_l1n_v`).

## Reference

- Full register descriptions: see `doc/deif-gc-1f-option-h2-4189340483-uk.pdf`.

## MQTT → Modbus Commands (FC0F write coils)

Commands are enabled individually with `ENABLE_COMMAND_<NAME>=true` and are rate-limited by `CMD_COOLDOWN_MS`. Retained MQTT messages are ignored.

| MQTT Topic | Action | Coil Offset | Enable Flag |
| --- | --- | --- | --- |
| `TOPIC_PREFIX/cmd/start` | Start | `1` | `ENABLE_COMMAND_START` |
| `TOPIC_PREFIX/cmd/gb_on` | Generator breaker ON | `2` | `ENABLE_COMMAND_GB_ON` |
| `TOPIC_PREFIX/cmd/gb_off` | Generator breaker OFF | `3` | `ENABLE_COMMAND_GB_OFF` |
| `TOPIC_PREFIX/cmd/stop` | Stop | `4` | `ENABLE_COMMAND_STOP` |
| `TOPIC_PREFIX/cmd/alarm_ack` | Acknowledge active alarms | `10` | `ENABLE_COMMAND_ALARM_ACK` |
| `TOPIC_PREFIX/cmd/start_gb_on` | Start and GB ON | `15` | `ENABLE_COMMAND_START_GB_ON` |
| `TOPIC_PREFIX/cmd/gb_off_stop` | GB OFF and Stop | `16` | `ENABLE_COMMAND_GB_OFF_STOP` |
| `TOPIC_PREFIX/cmd/mb_on` | Mains breaker ON | `25` | `ENABLE_COMMAND_MB_ON` |
| `TOPIC_PREFIX/cmd/mb_off` | Mains breaker OFF | `26` | `ENABLE_COMMAND_MB_OFF` |
| `TOPIC_PREFIX/cmd/mode_manual` | Switch to Manual mode | `28` | `ENABLE_COMMAND_MANUAL_MODE` |
| `TOPIC_PREFIX/cmd/mode_auto` | Switch to Auto mode | `30` | `ENABLE_COMMAND_AUTO_MODE` |
| `TOPIC_PREFIX/cmd/mode_test` | Switch to Test mode | `31` | `ENABLE_COMMAND_TEST` |

MQTT discovery buttons in Home Assistant publish to these topics so operators do not need to craft payloads manually.
