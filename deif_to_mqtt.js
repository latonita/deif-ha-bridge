#!/usr/bin/env node
'use strict';

require('dotenv').config();

const ModbusRTU = require('modbus-serial');
const mqtt = require('mqtt');

/* =========================
   CONFIG (from .env)
   ========================= */
const SERIAL_PORT = process.env.DEIF_PORT || '/dev/ttyUSB0';
const BAUD = parseInt(process.env.DEIF_BAUD || '9600', 10);
const SLAVE_ID = parseInt(process.env.DEIF_SLAVE || '1', 10);

const MQTT_URL = process.env.MQTT_URL || 'mqtt://localhost:1883';
const MQTT_USER = process.env.MQTT_USER || '';
const MQTT_PASS = process.env.MQTT_PASS || '';

const TOPIC_PREFIX = (process.env.TOPIC_PREFIX || 'deif/gc1f2').replace(/\/+$/, '');
const INTERVAL_MS = parseInt(process.env.INTERVAL_MS || '5000', 10);
const RETAIN = (process.env.RETAIN || 'true').toLowerCase() === 'true';
const CMD_COOLDOWN_MS = parseInt(process.env.CMD_COOLDOWN_MS || '5000', 10);

// Static device metadata
const DEVICE_MODEL = process.env.DEVICE_MODEL || 'DEIF GC-1F/2';
const DEVICE_MANUFACTURER = process.env.DEVICE_MANUFACTURER || 'DEIF';
const DEVICE_NAME = process.env.DEVICE_NAME || `DEIF GC-1F/2 (${SLAVE_ID})`;

// HA Discovery
const HASS_DISCOVERY_PREFIX = (process.env.HASS_DISCOVERY_PREFIX || 'homeassistant').replace(/\/+$/, '');
const HASS_NODE_ID = process.env.HASS_NODE_ID || `deif-gc1f2-${SLAVE_ID}`;
const HASS_DEVICE_ID = process.env.HASS_DEVICE_ID || HASS_NODE_ID;

function envFlag(name, defaultValue = 'false') {
  return (process.env[name] || defaultValue).toLowerCase() === 'true';
}

const ENABLE_COMMAND_ALARM_ACK = envFlag('ENABLE_COMMAND_ALARM_ACK');
const ENABLE_COMMAND_MANUAL_MODE = envFlag('ENABLE_COMMAND_MANUAL_MODE');
const ENABLE_COMMAND_AUTO_MODE = envFlag('ENABLE_COMMAND_AUTO_MODE');
const ENABLE_COMMAND_START = envFlag('ENABLE_COMMAND_START');
const ENABLE_COMMAND_GB_ON = envFlag('ENABLE_COMMAND_GB_ON');
const ENABLE_COMMAND_GB_OFF = envFlag('ENABLE_COMMAND_GB_OFF');
const ENABLE_COMMAND_STOP = envFlag('ENABLE_COMMAND_STOP');
const ENABLE_COMMAND_START_GB_ON = envFlag('ENABLE_COMMAND_START_GB_ON');
const ENABLE_COMMAND_GB_OFF_STOP = envFlag('ENABLE_COMMAND_GB_OFF_STOP');
const ENABLE_COMMAND_MB_ON = envFlag('ENABLE_COMMAND_MB_ON');
const ENABLE_COMMAND_MB_OFF = envFlag('ENABLE_COMMAND_MB_OFF');
const ENABLE_COMMAND_TEST = envFlag('ENABLE_COMMAND_TEST');
const PUBLISH_ALARM_BITFIELDS = envFlag('PUBLISH_ALARM_BITFIELDS');

function describeAlarmKey(key) {
  const [regStr, bitStr] = key.split(':');
  const def = ALARM_MAP[key];
  if (def) {
    return `${regStr}:${bitStr} (${def.code} ${def.text})`;
  }
  return `${regStr}:${bitStr}`;
}

// Manual says Hz/100, but your device shows 500 for 50.0 Hz -> divisor 10.
// Keep configurable.
const FREQ_DIVISOR = parseFloat(process.env.FREQ_DIVISOR || '10');
const FREQ_DECIMALS = parseInt(process.env.FREQ_DECIMALS || '1', 10);

/* =========================
   REGISTER MAP (Measurement table, FC04, addresses 500+)
   From DEIF Option H2 manual:
   - App version: 500
   - Gen U L1-N: 504, Gen I L1: 513, Gen f: 507
   - PGEN/QGEN/SGEN totals: 519 / 523 / 527
   - Cosphi: 538
   - Mains U L1N/L2N/L3N: 542/543/544, fMains: 545
   - EGEN total (32-bit): 536 [HI], 537 [LO]
   - Running time (32-bit): 554 [HI], 555 [LO]
   - USUPPLY: 567
   ========================= */
const R = {
  APP_VERSION: 500,

  GEN_U_L1N: 504,
  GEN_U_L2N: 505,
  GEN_U_L3N: 506,
  
  GEN_F: 507,

  GEN_I_L1: 513,
  GEN_I_L2: 514,
  GEN_I_L3: 515,

  PGEN: 519,
  QGEN: 523,
  SGEN: 527,
  
  EGEN_HI: 536, // Total energy counter. Measured in [kWh]. Max. 300000MWh
  EGEN_LO: 537,

  COS_PHI: 538, // cos f x 100, signed

  MAINS_U_L1N: 542,
  MAINS_U_L2N: 543,
  MAINS_U_L3N: 544,
  MAINS_F: 545,

  RUN_HI: 554, // Absolute running hour counter 
  RUN_LO: 555,

  ALARM_COUNT: 558,           // Number of alarms
  ALARM_UNACK: 559,           // Number of unacknowledged alarms
  ALARM_ACK_ACTIVE: 560,      // Number of acknowledged active alarms

  GB_OPERATIONS: 563,         // Generator/circuit breaker operations counter
  MB_OPERATIONS: 564,         // Mains breaker operation counter
  START_ATTEMPTS: 566,        // Number of start attempts
  USUPPLY: 567,               // V/10 Supply voltage i.e. battery/PSU
  RPM: 576,                   // Running feedback RPM
};

// We'll read one continuous block from 500 to 576 inclusive.
const MEAS_START = 500;
const MEAS_END = 576;
const MEAS_COUNT = (MEAS_END - MEAS_START) + 1;

// Alarm bitfield registers
const ALARM_START = 1000;
const ALARM_END = 1019;
const ALARM_COUNT = (ALARM_END - ALARM_START) + 1;

// Command flags (FC0F write-only) zero-based offsets
const CMD_FLAG_START = 1;
const CMD_FLAG_GB_ON = 2;
const CMD_FLAG_GB_OFF = 3;
const CMD_FLAG_STOP = 4;
const CMD_FLAG_ALARM_ACK = 10;
const CMD_FLAG_START_GB_ON = 15;
const CMD_FLAG_GB_OFF_STOP = 16;
const CMD_FLAG_MB_ON = 25;
const CMD_FLAG_MB_OFF = 26;
const CMD_FLAG_MANUAL_MODE = 28;
const CMD_FLAG_AUTO_MODE = 30;
const CMD_FLAG_TEST = 31;

const COMMAND_DEFS = [
  { key: 'alarm_ack', name: 'Alarm Acknowledge', topic: `${TOPIC_PREFIX}/cmd/alarm_ack`, offset: CMD_FLAG_ALARM_ACK, enabled: ENABLE_COMMAND_ALARM_ACK, icon: 'mdi:alarm-check' },
  { key: 'start', name: 'Start', topic: `${TOPIC_PREFIX}/cmd/start`, offset: CMD_FLAG_START, enabled: ENABLE_COMMAND_START, icon: 'mdi:play-circle' },
  { key: 'gb_on', name: 'GB ON', topic: `${TOPIC_PREFIX}/cmd/gb_on`, offset: CMD_FLAG_GB_ON, enabled: ENABLE_COMMAND_GB_ON, icon: 'mdi:flash' },
  { key: 'gb_off', name: 'GB OFF', topic: `${TOPIC_PREFIX}/cmd/gb_off`, offset: CMD_FLAG_GB_OFF, enabled: ENABLE_COMMAND_GB_OFF, icon: 'mdi:flash-off' },
  { key: 'stop', name: 'Stop', topic: `${TOPIC_PREFIX}/cmd/stop`, offset: CMD_FLAG_STOP, enabled: ENABLE_COMMAND_STOP, icon: 'mdi:stop-circle' },
  { key: 'start_gb_on', name: 'Start + GB ON', topic: `${TOPIC_PREFIX}/cmd/start_gb_on`, offset: CMD_FLAG_START_GB_ON, enabled: ENABLE_COMMAND_START_GB_ON, icon: 'mdi:play-network' },
  { key: 'gb_off_stop', name: 'GB OFF + Stop', topic: `${TOPIC_PREFIX}/cmd/gb_off_stop`, offset: CMD_FLAG_GB_OFF_STOP, enabled: ENABLE_COMMAND_GB_OFF_STOP, icon: 'mdi:power-plug-off' },
  { key: 'mb_on', name: 'MB ON', topic: `${TOPIC_PREFIX}/cmd/mb_on`, offset: CMD_FLAG_MB_ON, enabled: ENABLE_COMMAND_MB_ON, icon: 'mdi:transmission-tower-import' },
  { key: 'mb_off', name: 'MB OFF', topic: `${TOPIC_PREFIX}/cmd/mb_off`, offset: CMD_FLAG_MB_OFF, enabled: ENABLE_COMMAND_MB_OFF, icon: 'mdi:transmission-tower-off' },
  { key: 'mode_manual', name: 'Mode: Manual', topic: `${TOPIC_PREFIX}/cmd/mode_manual`, offset: CMD_FLAG_MANUAL_MODE, enabled: ENABLE_COMMAND_MANUAL_MODE, icon: 'mdi:hand-back-right' },
  { key: 'mode_auto', name: 'Mode: Auto', topic: `${TOPIC_PREFIX}/cmd/mode_auto`, offset: CMD_FLAG_AUTO_MODE, enabled: ENABLE_COMMAND_AUTO_MODE, icon: 'mdi:autorenew' },
  { key: 'mode_test', name: 'Mode: Test', topic: `${TOPIC_PREFIX}/cmd/mode_test`, offset: CMD_FLAG_TEST, enabled: ENABLE_COMMAND_TEST, icon: 'mdi:beaker' },
];

const ENABLED_COMMANDS = COMMAND_DEFS.filter(c => c.enabled);

// Alarm descriptions: key format is "register:bit"
const ALARM_MAP = {
  // 1000 Protection alarms
  '1000:0':  { code: '1000', text: 'G -P> 1' },
  '1000:3':  { code: '1030', text: 'G I> 1' },
  '1000:4':  { code: '1040', text: 'G I> 2' },
  '1000:9':  { code: '1130', text: 'G I>> 1' },
  '1000:10': { code: '1140', text: 'G I>> 2' },
  '1000:11': { code: '1150', text: 'G U> 1' },
  '1000:12': { code: '1160', text: 'G U> 2' },
  '1000:13': { code: '1170', text: 'G U< 1' },
  '1000:14': { code: '1180', text: 'G U< 2' },

  // 1001 Protection alarms
  '1001:0':  { code: '1210', text: 'G f> 1' },
  '1001:1':  { code: '1220', text: 'G f> 2' },
  '1001:3':  { code: '1240', text: 'G f< 1' },
  '1001:4':  { code: '1250', text: 'G f< 2' },
  '1001:6':  { code: 'no code', text: 'BB U> 1' },
  '1001:7':  { code: 'no code', text: 'BB U> 2' },
  '1001:9':  { code: 'no code', text: 'BB U< 1' },
  '1001:10': { code: 'no code', text: 'BB U< 2' },
  '1001:13': { code: 'no code', text: 'BB f> 1' },
  '1001:14': { code: 'no code', text: 'BB f> 2' },

  // 1002 Protection alarms
  '1002:0':  { code: 'no code', text: 'BB f< 1' },
  '1002:1':  { code: 'no code', text: 'BB f< 2' },
  '1002:7':  { code: '1450', text: 'G P> 1' },
  '1002:8':  { code: '1460', text: 'G P> 2' },
  '1002:14': { code: 'no code', text: '-Q>' },
  '1002:15': { code: 'no code', text: 'Q>' },

  // 1003 Protection alarms
  '1003:7':  { code: '1620', text: 'Mains unbalanced voltage' },

  // 1005 Synchronisation alarms
  '1005:3':  { code: '2150', text: 'Phase seq error' },
  '1005:4':  { code: '2160', text: 'GB open failure' },
  '1005:5':  { code: '2170', text: 'GB close failure' },
  '1005:6':  { code: 'no code', text: 'GB pos failure' },
  '1005:7':  { code: '2200', text: 'MB open failure' },
  '1005:8':  { code: '2210', text: 'MB close failure' },
  '1005:9':  { code: 'no code', text: 'MB pos failure' },

  // 1010 Protection alarms (multi-inputs)
  '1010:0':  { code: '3400', text: 'Dig. multi-input 1' },
  '1010:1':  { code: '3410', text: 'Dig. multi-input 2' },
  '1010:2':  { code: '3420', text: 'Dig. multi-input 3' },
  '1010:3':  { code: '3404', text: 'Wire failure, dig. multi-input 1' },
  '1010:4':  { code: '3404', text: 'Wire failure, dig. multi-input 2' },
  '1010:5':  { code: '3424', text: 'Wire failure, dig. multi-input 3' },
  '1010:12': { code: '3490', text: 'Dig. input 19-20/Emergency STOP' },

  // 1013 Analogue inputs
  '1013:0':  { code: 'no code', text: 'Multi-input 1.1' },
  '1013:1':  { code: 'no code', text: 'Multi-input 1.2' },
  '1013:2':  { code: 'no code', text: 'W. failure, multi-input 1' },
  '1013:3':  { code: 'no code', text: 'Multi-input 2.1' },
  '1013:4':  { code: 'no code', text: 'Multi-input 2.2' },
  '1013:5':  { code: 'no code', text: 'W. failure, multi-input 2' },
  '1013:6':  { code: 'no code', text: 'Multi-input 3.1' },
  '1013:7':  { code: 'no code', text: 'Multi-input 3.2' },
  '1013:8':  { code: 'no code', text: 'W. failure, multi-input 3' },
  '1013:9':  { code: '4510', text: 'Overspeed 1' },
  '1013:10': { code: '4520', text: 'Overspeed 2' },
  '1013:11': { code: '4620', text: 'VDO fuel level 1.3' },
  '1013:12': { code: '4610', text: 'Charger gen' },
  '1013:13': { code: '4600', text: 'V-Belt' },
  '1013:14': { code: '4560', text: 'Generator Hz/V failure' },
  '1013:15': { code: 'no code', text: 'Start failure' },

  // 1014 Analogue inputs
  '1014:0':  { code: '4580', text: 'Stop failure' },
  '1014:1':  { code: '4960', text: 'U< aux. supply term. 1' },
  '1014:2':  { code: '4970', text: 'U> aux. supply term. 1' },
  '1014:5':  { code: '4610', text: 'Charger Gen' },

  // 1015 System/general alarms
  '1015:0':  { code: '6110', text: 'Service timer 1' },
  '1015:1':  { code: '6120', text: 'Service timer 2' },
  '1015:13': { code: 'no code', text: 'Fuel fill check' },
};

const STATUS_MAP = {
  // 1018 Status
  '1018:0':  { code: 'no code', text: 'Mains failure' },
  '1018:1':  { code: 'no code', text: 'MB pos ON' },
  '1018:4':  { code: 'no code', text: 'GB pos ON' },
  '1018:6':  { code: 'no code', text: 'Engine running' },
  '1018:7':  { code: 'no code', text: 'Running detection, timer expired' },
  '1018:8':  { code: 'no code', text: 'DG Hz/V OK, timer expired' },

  // 1019 Operating mode status
  '1019:0':  { code: 'no code', text: 'OFF' },
  '1019:1':  { code: 'no code', text: 'Manual' },
  '1019:3':  { code: 'no code', text: 'Auto' },
  '1019:4':  { code: 'no code', text: 'Test' },
  '1019:5':  { code: 'no code', text: 'Island' },
  '1019:6':  { code: 'no code', text: 'AMF' },
  '1019:10': { code: 'no code', text: 'Load take over' },
  '1019:15': { code: 'no code', text: 'AMF active' }
};

const ALARM_REGISTERS = Array.from(new Set(Object.keys(ALARM_MAP)
  .map(k => parseInt(k.split(':')[0], 10)))).sort((a, b) => a - b);
const ALARM_REG_SET = new Set(ALARM_REGISTERS);

const STATUS_REGISTERS = Array.from(new Set(Object.keys(STATUS_MAP)
  .map(k => parseInt(k.split(':')[0], 10)))).sort((a, b) => a - b);

const BIT_MASKS = Array.from({ length: 16 }, (_, bit) => 1 << bit);


/* =========================
   HELPERS
   ========================= */
function u32(hi, lo) {
  return ((hi & 0xffff) << 16) | (lo & 0xffff);
}

function s16(x) {
  const v = x & 0xffff;
  return v > 0x7fff ? v - 0x10000 : v;
}

function fmtAppVersion(raw) {
  const str = String(raw).padStart(4, '0');
  const first = str.charAt(0);
  const second = str.charAt(1);
  const lastTwo = str.slice(2);
  return `${first}.${second}.${lastTwo}`;
}

function freqFloat(x) {
  return Number.parseFloat(Number(x).toFixed(FREQ_DECIMALS));
}

function toHex(x) {
  return '0x' + (x & 0xffff).toString(16).toUpperCase().padStart(4, '0');
}

function decodeAlarms(alarmRegs) {
  const active = [];
  for (let i = 0; i < alarmRegs.length; i++) {
    const regAddr = ALARM_START + i;

    if (!ALARM_REG_SET.has(regAddr)) continue;

    const regValue = alarmRegs[i];

    // Check each bit in the register
    for (let bit = 0; bit < BIT_MASKS.length; bit++) {
      if (regValue & BIT_MASKS[bit]) {
        const key = `${regAddr}:${bit}`;
        const alarm = ALARM_MAP[key];
        if (alarm) {
          active.push({
            register: regAddr,
            bit,
            code: alarm.code,
            text: alarm.text
          });
        }
      }
    }
  }
  return active;
}

function formatActiveAlarms(activeAlarms) {
  if (!activeAlarms || activeAlarms.length === 0) {
    return 'No active alarms';
  }
  return activeAlarms.map(a => `${a.code} ${a.text}`).join('\n');
}

function decodeStatus(alarmRegs) {
  const status = {};

  for (const regAddr of STATUS_REGISTERS) {
    const regIdx = regAddr - ALARM_START;
    if (regIdx < 0 || regIdx >= alarmRegs.length) continue;

    const regValue = alarmRegs[regIdx];

    // Check each bit in the register
    for (let bit = 0; bit < BIT_MASKS.length; bit++) {
      const key = `${regAddr}:${bit}`;
      const statusDef = STATUS_MAP[key];
      if (statusDef) {
        const isActive = !!(regValue & BIT_MASKS[bit]);
        const statusKey = key.replace(':', '_');
        status[statusKey] = isActive;
      }
    }
  }
  
  return status;
}

function getOperatingModeText(status) {
  // Determine primary mode (priority order: OFF > Manual > Test > Auto)
  let primaryMode = null;
  
  if (status['1019_0']) primaryMode = 'OFF';
  else if (status['1019_1']) primaryMode = 'Manual';
  else if (status['1019_4']) primaryMode = 'Test';
  else if (status['1019_3']) primaryMode = 'Auto';
  
  if (!primaryMode) primaryMode = 'Unknown';
  
  // Collect modifiers
  const modifiers = [];
  if (status['1019_6']) modifiers.push('AMF');
  if (status['1019_10']) modifiers.push('Load Takeover');
  if (status['1019_15']) modifiers.push('AMF Active');
  
  // Format: "Primary (Modifier1, Modifier2)"
  if (modifiers.length > 0) {
    return `${primaryMode} (${modifiers.join(', ')})`;
  }
  
  return primaryMode;
}

function publish(mq, key, value, retainOverride) {
  const topic = `${TOPIC_PREFIX}/${key}`;
  const retain = (typeof retainOverride === 'boolean') ? retainOverride : RETAIN;
  const payload = (typeof value === 'object') ? JSON.stringify(value) : String(value);
  mq.publish(topic, payload, { qos: 0, retain });
}

function publishFlat(mq, prefixKey, obj, retainOverride) {
  const stack = [{ path: prefixKey, value: obj }];
  while (stack.length) {
    const { path, value } = stack.pop();
    if (value !== null && typeof value === 'object' && !Array.isArray(value)) {
      for (const [k, v] of Object.entries(value)) {
        stack.push({ path: `${path}/${k}`, value: v });
      }
    } else {
      publish(mq, path, value, retainOverride);
    }
  }
}

async function readInputBlock(mb, start, count) {
  const res = await mb.readInputRegisters(start, count);
  return res.data; // array of 16-bit register values
}

async function writeCommandFlag(mb, offset) {
  await mb.writeCoils(offset, [true]);
}

function createCommandHandler(mb, commands) {
  let lastRun = 0;
  const commandByTopic = new Map(commands.map(c => [c.topic, c]));

  function withinCooldown() {
    const now = Date.now();
    if (now - lastRun < CMD_COOLDOWN_MS) return false;
    lastRun = now;
    return true;
  }

  return async function handleCommand(topic) {
    try {
      const cmd = commandByTopic.get(topic);
      if (!cmd) return;
      console.log(`Command received: ${cmd.key} (topic=${topic}, offset=${cmd.offset})`);
      if (!withinCooldown()) {
        console.log('Command skipped (cooldown active)');
        return;
      }
      await writeCommandFlag(mb, cmd.offset);
      console.log(`Command sent: ${cmd.key}`);
    } catch (err) {
      console.error('Command error', topic, err && err.message ? err.message : err);
    }
  };
}

function getReg(block, addr) {
  const idx = addr - MEAS_START;
  if (idx < 0 || idx >= block.length) return undefined;
  return block[idx];
}

function readGen(block) {
  return {
    voltage_l1n_v: getReg(block, R.GEN_U_L1N),
    voltage_l2n_v: getReg(block, R.GEN_U_L2N),
    voltage_l3n_v: getReg(block, R.GEN_U_L3N),
    current_l1_a: getReg(block, R.GEN_I_L1),
    current_l2_a: getReg(block, R.GEN_I_L2),
    current_l3_a: getReg(block, R.GEN_I_L3),
    frequency_hz: freqFloat(getReg(block, R.GEN_F) / FREQ_DIVISOR),
    pgen_kw: s16(getReg(block, R.PGEN)),
    qgen_kvar: s16(getReg(block, R.QGEN)),
    sgen_kva: s16(getReg(block, R.SGEN)),
    cos_phi: s16(getReg(block, R.COS_PHI)) / 100.0,
  };
}

function readMains(block) {
  return {
    voltage_l1n_v: getReg(block, R.MAINS_U_L1N),
    voltage_l2n_v: getReg(block, R.MAINS_U_L2N),
    voltage_l3n_v: getReg(block, R.MAINS_U_L3N),
    frequency_hz: freqFloat(getReg(block, R.MAINS_F) / FREQ_DIVISOR),
  };
}

function readRunHours(block) {
  return u32(getReg(block, R.RUN_HI), getReg(block, R.RUN_LO));
}

function readEnergy(block) {
  const energyKwh = u32(getReg(block, R.EGEN_HI), getReg(block, R.EGEN_LO));
  return { energyKwh, energySignedKwh: -energyKwh };
}

function readCounters(block) {
  return {
    gen_breaker_ops: getReg(block, R.GB_OPERATIONS),
    mains_breaker_ops: getReg(block, R.MB_OPERATIONS),
    start_attempts: getReg(block, R.START_ATTEMPTS),
  };
}

function readUsupply(block) {
  return Number.parseFloat((getReg(block, R.USUPPLY) / 10.0).toFixed(1));
}

/* =========================
   HOME ASSISTANT DISCOVERY
   ========================= */
function publishHassDiscovery(mq) {
  const stateTopic = `${TOPIC_PREFIX}/state`;

  const device = {
    identifiers: [HASS_DEVICE_ID],
    name: DEVICE_NAME,
    manufacturer: DEVICE_MANUFACTURER,
    model: DEVICE_MODEL,
  };

  function dataTopicFromPath(jsonPath) {
    return `${TOPIC_PREFIX}/${jsonPath.replace(/\./g, '/')}`;
  }

  function pubSensor(key, cfg) {
    const objectId = `${HASS_NODE_ID}-${key}`;
    const topic = `${HASS_DISCOVERY_PREFIX}/sensor/${HASS_NODE_ID}/${key}/config`;

    const statTopic = dataTopicFromPath(cfg.jsonPath);

    const valueTemplate = '{{ value }}';

    const payload = {
      name: cfg.name,
      uniq_id: objectId,
      obj_id: objectId,
      stat_t: statTopic,
      val_tpl: valueTemplate,
      en: true,
      force_update: true,
      device,
      ...(cfg.icon ? { ic: cfg.icon } : {}),
      ...(cfg.deviceClass ? { dev_cla: cfg.deviceClass } : {}),
      ...(cfg.stateClass ? { stat_cla: cfg.stateClass } : {}),
      ...(cfg.unit ? { unit_of_meas: cfg.unit } : {}),
      ...(cfg.entityCategory ? { ent_cat: cfg.entityCategory } : {}),
    };

    mq.publish(topic, JSON.stringify(payload), { qos: 0, retain: true });
  }

  function pubBinarySensor(key, cfg) {
    const objectId = `${HASS_NODE_ID}-${key}`;
    const topic = `${HASS_DISCOVERY_PREFIX}/binary_sensor/${HASS_NODE_ID}/${key}/config`;

    const statTopic = dataTopicFromPath(cfg.jsonPath || '');

    const defaultTemplate = '{{ "ON" if value|string|lower in ["true","on","1"] else "OFF" }}';
    const valueTemplate = cfg.valueTemplateFlat || defaultTemplate;

    const payload = {
      name: cfg.name,
      uniq_id: objectId,
      obj_id: objectId,
      stat_t: statTopic,
      val_tpl: valueTemplate,
      en: true,
      device,
      ...(cfg.icon ? { ic: cfg.icon } : {}),
      ...(cfg.deviceClass ? { dev_cla: cfg.deviceClass } : {}),
      ...(cfg.entityCategory ? { ent_cat: cfg.entityCategory } : {}),
    };

    mq.publish(topic, JSON.stringify(payload), { qos: 0, retain: true });
  }

  function pubButton(key, cfg) {
    const objectId = `${HASS_NODE_ID}-${key}`;
    const topic = `${HASS_DISCOVERY_PREFIX}/button/${HASS_NODE_ID}/${key}/config`;

    const payload = {
      name: cfg.name,
      uniq_id: objectId,
      obj_id: objectId,
      cmd_t: cfg.commandTopic,
      pl_prs: cfg.payloadPress || '1',
      en: true,
      device,
      ...(cfg.icon ? { ic: cfg.icon } : {}),
      ...(cfg.entityCategory ? { ent_cat: cfg.entityCategory } : {}),
    };

    mq.publish(topic, JSON.stringify(payload), { qos: 0, retain: true });
  }

  const sensors = [
    // Metadata / diagnostics
    { key: 'app_version', name: 'App Version', jsonPath: 'app_version', entityCategory: 'diagnostic', icon: 'mdi:information' },
    { key: 'device_model', name: 'Device Model', jsonPath: 'device.model', entityCategory: 'diagnostic', icon: 'mdi:chip' },
    { key: 'timestamp', name: 'Last Seen', jsonPath: 'ts', deviceClass: 'timestamp', entityCategory: 'diagnostic', icon: 'mdi:clock' },

    // Generator
    { key: 'gen_voltage_l1n', name: 'Generator Voltage L1-N', jsonPath: 'gen.voltage_l1n_v', deviceClass: 'voltage', unit: 'V', stateClass: 'measurement', icon: 'mdi:sine-wave' },
    { key: 'gen_voltage_l2n', name: 'Generator Voltage L2-N', jsonPath: 'gen.voltage_l2n_v', deviceClass: 'voltage', unit: 'V', stateClass: 'measurement', icon: 'mdi:sine-wave' },
    { key: 'gen_voltage_l3n', name: 'Generator Voltage L3-N', jsonPath: 'gen.voltage_l3n_v', deviceClass: 'voltage', unit: 'V', stateClass: 'measurement', icon: 'mdi:sine-wave' },
    { key: 'gen_current_l1', name: 'Generator Current L1', jsonPath: 'gen.current_l1_a', deviceClass: 'current', unit: 'A', stateClass: 'measurement', icon: 'mdi:current-ac' },
    { key: 'gen_current_l2', name: 'Generator Current L2', jsonPath: 'gen.current_l2_a', deviceClass: 'current', unit: 'A', stateClass: 'measurement', icon: 'mdi:current-ac' },
    { key: 'gen_current_l3', name: 'Generator Current L3', jsonPath: 'gen.current_l3_a', deviceClass: 'current', unit: 'A', stateClass: 'measurement', icon: 'mdi:current-ac' },
    { key: 'gen_frequency', name: 'Generator Frequency', jsonPath: 'gen.frequency_hz', deviceClass: 'frequency', unit: 'Hz', stateClass: 'measurement', icon: 'mdi:waveform' },
    { key: 'gen_pgen', name: 'Generator Active Power', jsonPath: 'gen.pgen_kw', deviceClass: 'power', unit: 'kW', stateClass: 'measurement', icon: 'mdi:flash' },
    { key: 'gen_qgen', name: 'Generator Reactive Power', jsonPath: 'gen.qgen_kvar', unit: 'kVAr', stateClass: 'measurement', icon: 'mdi:flash-outline' },
    { key: 'gen_sgen', name: 'Generator Apparent Power', jsonPath: 'gen.sgen_kva', unit: 'kVA', stateClass: 'measurement', icon: 'mdi:flash-triangle' },
    { key: 'gen_cos_phi', name: 'Generator Power Factor', jsonPath: 'gen.cos_phi', stateClass: 'measurement', icon: 'mdi:cosine-wave' },

    // Mains
    { key: 'mains_voltage_l1n', name: 'Mains Voltage L1-N', jsonPath: 'mains.voltage_l1n_v', deviceClass: 'voltage', unit: 'V', stateClass: 'measurement', icon: 'mdi:transmission-tower' },
    { key: 'mains_voltage_l2n', name: 'Mains Voltage L2-N', jsonPath: 'mains.voltage_l2n_v', deviceClass: 'voltage', unit: 'V', stateClass: 'measurement', icon: 'mdi:transmission-tower' },
    { key: 'mains_voltage_l3n', name: 'Mains Voltage L3-N', jsonPath: 'mains.voltage_l3n_v', deviceClass: 'voltage', unit: 'V', stateClass: 'measurement', icon: 'mdi:transmission-tower' },
    { key: 'mains_frequency', name: 'Mains Frequency', jsonPath: 'mains.frequency_hz', deviceClass: 'frequency', unit: 'Hz', stateClass: 'measurement', icon: 'mdi:waveform' },

    // Counters
    { key: 'run_hours', name: 'Generator Run Hours', jsonPath: 'run_hours', unit: 'h', stateClass: 'total_increasing', icon: 'mdi:timer-outline' },
    { key: 'energy_kwh', name: 'Energy Produced', jsonPath: 'energy_kwh', deviceClass: 'energy', unit: 'kWh', stateClass: 'total_increasing', icon: 'mdi:lightning-bolt' },

    // Alarms (primary)
    { key: 'active_alarms_text', name: 'Active Alarms', jsonPath: 'alarms.active_text', icon: 'mdi:alarm-light' },

    // Battery/supply
    { key: 'usupply_v', name: 'Battery Voltage', jsonPath: 'usupply_v', deviceClass: 'voltage', unit: 'V', stateClass: 'measurement', icon: 'mdi:car-battery' },

    // Alarms (diagnostic)
    { key: 'alarm_count', name: 'Alarms Total', jsonPath: 'alarms.count', stateClass: 'measurement', entityCategory: 'diagnostic', icon: 'mdi:counter' },
    { key: 'alarm_unacknowledged', name: 'Alarms Unacknowledged', jsonPath: 'alarms.unacknowledged', stateClass: 'measurement', entityCategory: 'diagnostic', icon: 'mdi:alert-circle' },
    { key: 'alarm_ack_active', name: 'Alarms Acknowledged Active', jsonPath: 'alarms.ack_active', stateClass: 'measurement', entityCategory: 'diagnostic', icon: 'mdi:alert-circle-check' },

    // Energy/counters (diagnostic)
    { key: 'energy_signed_kwh', name: 'Energy Produced (Signed)', jsonPath: 'energy_signed_kwh', unit: 'kWh', entityCategory: 'diagnostic', icon: 'mdi:swap-horizontal' },
    { key: 'gen_breaker_ops', name: 'Generator Breaker Operations', jsonPath: 'counters.gen_breaker_ops', stateClass: 'total_increasing', entityCategory: 'diagnostic', icon: 'mdi:electric-switch' },
    { key: 'mains_breaker_ops', name: 'Mains Breaker Operations', jsonPath: 'counters.mains_breaker_ops', stateClass: 'total_increasing', entityCategory: 'diagnostic', icon: 'mdi:electric-switch' },
    { key: 'start_attempts', name: 'Start Attempts', jsonPath: 'counters.start_attempts', stateClass: 'total_increasing', entityCategory: 'diagnostic', icon: 'mdi:restart' },


    // Technical (diagnostic)
    { key: 'rpm', name: 'Engine RPM', jsonPath: 'rpm', unit: 'RPM', stateClass: 'measurement', entityCategory: 'diagnostic', icon: 'mdi:engine' },
    { key: 'last_run_started', name: 'Last Run Started', jsonPath: 'last_run_started', deviceClass: 'timestamp', entityCategory: 'diagnostic', icon: 'mdi:clock-start' },
    { key: 'last_run_stopped', name: 'Last Run Stopped', jsonPath: 'last_run_stopped', deviceClass: 'timestamp', entityCategory: 'diagnostic', icon: 'mdi:clock-end' },
    { key: 'last_run_duration_s', name: 'Last Run Duration', jsonPath: 'last_run_duration_s', unit: 's', stateClass: 'measurement', entityCategory: 'diagnostic', icon: 'mdi:timer-outline' },
    
    // Operating mode (primary status)
    { key: 'operating_mode', name: 'Operating Mode', jsonPath: 'status.operating_mode', icon: 'mdi:state-machine' },
  ];

  for (const s of sensors) pubSensor(s.key, s);

  if (PUBLISH_ALARM_BITFIELDS) {
    const bitfieldSensors = [
      { key: 'alarm_bitfield_1000', name: 'Alarm Bitfield 1000', valueTemplate: "{{ value_json.alarms.bitfield['1000'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1001', name: 'Alarm Bitfield 1001', valueTemplate: "{{ value_json.alarms.bitfield['1001'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1002', name: 'Alarm Bitfield 1002', valueTemplate: "{{ value_json.alarms.bitfield['1002'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1003', name: 'Alarm Bitfield 1003', valueTemplate: "{{ value_json.alarms.bitfield['1003'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1004', name: 'Alarm Bitfield 1004', valueTemplate: "{{ value_json.alarms.bitfield['1004'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1005', name: 'Alarm Bitfield 1005', valueTemplate: "{{ value_json.alarms.bitfield['1005'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1006', name: 'Alarm Bitfield 1006', valueTemplate: "{{ value_json.alarms.bitfield['1006'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1007', name: 'Alarm Bitfield 1007', valueTemplate: "{{ value_json.alarms.bitfield['1007'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1008', name: 'Alarm Bitfield 1008', valueTemplate: "{{ value_json.alarms.bitfield['1008'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1009', name: 'Alarm Bitfield 1009', valueTemplate: "{{ value_json.alarms.bitfield['1009'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1010', name: 'Alarm Bitfield 1010', valueTemplate: "{{ value_json.alarms.bitfield['1010'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1011', name: 'Alarm Bitfield 1011', valueTemplate: "{{ value_json.alarms.bitfield['1011'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1012', name: 'Alarm Bitfield 1012', valueTemplate: "{{ value_json.alarms.bitfield['1012'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1013', name: 'Alarm Bitfield 1013', valueTemplate: "{{ value_json.alarms.bitfield['1013'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1014', name: 'Alarm Bitfield 1014', valueTemplate: "{{ value_json.alarms.bitfield['1014'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
      { key: 'alarm_bitfield_1015', name: 'Alarm Bitfield 1015', valueTemplate: "{{ value_json.alarms.bitfield['1015'] | is_defined }}", entityCategory: 'diagnostic', icon: 'mdi:code-brackets' },
    ];

    for (const s of bitfieldSensors) pubSensor(s.key, s);
  }

  // Binary sensors
  const binarySensors = [
    // Critical alerts (primary)
    { 
      key: 'has_unack_alarms', 
      name: 'Unacknowledged Alarms Active', 
      jsonPath: 'alarms.unacknowledged',
      valueTemplate: '{{ "ON" if value_json.alarms.unacknowledged > 0 else "OFF" }}',
      valueTemplateFlat: '{{ "ON" if (value | int(0)) > 0 else "OFF" }}',
      deviceClass: 'problem',
      icon: 'mdi:alert'
    },
    // Status binary sensors (primary)
    {
      key: 'status_mains_failure',
      name: 'Mains Failure',
      jsonPath: 'status.1018_0',
      valueTemplate: '{{ "ON" if value_json.status["1018_0"] else "OFF" }}',
      deviceClass: 'problem',
      icon: 'mdi:transmission-tower-off'
    },
    {
      key: 'status_mb_on',
      name: 'Mains Breaker ON',
      jsonPath: 'status.1018_1',
      valueTemplate: '{{ "ON" if value_json.status["1018_1"] else "OFF" }}',
      icon: 'mdi:electric-switch'
    },
    {
      key: 'status_gb_on',
      name: 'Generator Breaker ON',
      jsonPath: 'status.1018_4',
      valueTemplate: '{{ "ON" if value_json.status["1018_4"] else "OFF" }}',
      icon: 'mdi:electric-switch'
    },
    {
      key: 'status_engine_running',
      name: 'Engine Running',
      jsonPath: 'status.1018_6',
      valueTemplate: '{{ "ON" if value_json.status["1018_6"] else "OFF" }}',
      deviceClass: 'running',
      icon: 'mdi:engine'
    },
    {
      key: 'status_gen_ok',
      name: 'Generator Hz/V OK',
      jsonPath: 'status.1018_8',
      valueTemplate: '{{ "ON" if value_json.status["1018_8"] else "OFF" }}',
      icon: 'mdi:check-circle'
    },
    // Operating mode status (diagnostic)
    {
      key: 'status_running_timer',
      name: 'Running Detection Timer Expired',
      jsonPath: 'status.1018_7',
      valueTemplate: '{{ "ON" if value_json.status["1018_7"] else "OFF" }}',
      entityCategory: 'diagnostic',
      icon: 'mdi:timer-check'
    },
    {
      key: 'mode_off',
      name: 'Mode: OFF',
      jsonPath: 'status.1019_0',
      valueTemplate: '{{ "ON" if value_json.status["1019_0"] else "OFF" }}',
      entityCategory: 'diagnostic',
      icon: 'mdi:power-off'
    },
    {
      key: 'mode_manual',
      name: 'Mode: Manual',
      jsonPath: 'status.1019_1',
      valueTemplate: '{{ "ON" if value_json.status["1019_1"] else "OFF" }}',
      entityCategory: 'diagnostic',
      icon: 'mdi:hand-back-right'
    },
    {
      key: 'mode_auto',
      name: 'Mode: Auto',
      jsonPath: 'status.1019_3',
      valueTemplate: '{{ "ON" if value_json.status["1019_3"] else "OFF" }}',
      entityCategory: 'diagnostic',
      icon: 'mdi:autorenew'
    },
    {
      key: 'mode_test',
      name: 'Mode: Test',
      jsonPath: 'status.1019_4',
      valueTemplate: '{{ "ON" if value_json.status["1019_4"] else "OFF" }}',
      entityCategory: 'diagnostic',
      icon: 'mdi:test-tube'
    },
    {
      key: 'mode_island',
      name: 'Mode: Island',
      jsonPath: 'status.1019_5',
      valueTemplate: '{{ "ON" if value_json.status["1019_5"] else "OFF" }}',
      entityCategory: 'diagnostic',
      icon: 'mdi:island'
    },
    {
      key: 'mode_amf',
      name: 'Mode: AMF',
      jsonPath: 'status.1019_6',
      valueTemplate: '{{ "ON" if value_json.status["1019_6"] else "OFF" }}',
      entityCategory: 'diagnostic',
      icon: 'mdi:auto-mode'
    },
    {
      key: 'load_takeover',
      name: 'Load Take Over',
      jsonPath: 'status.1019_10',
      valueTemplate: '{{ "ON" if value_json.status["1019_10"] else "OFF" }}',
      entityCategory: 'diagnostic',
      icon: 'mdi:transfer'
    },
    {
      key: 'amf_active',
      name: 'AMF Active',
      jsonPath: 'status.1019_15',
      valueTemplate: '{{ "ON" if value_json.status["1019_15"] else "OFF" }}',
      entityCategory: 'diagnostic',
      icon: 'mdi:lightning-bolt'
    },
  ];

  for (const bs of binarySensors) pubBinarySensor(bs.key, bs);

  if (ENABLED_COMMANDS.length > 0) {
    for (const cmd of ENABLED_COMMANDS) {
      pubButton(`cmd_${cmd.key}`, {
        name: cmd.name,
        commandTopic: cmd.topic,
        icon: cmd.icon,
      });
    }
  }
}

/* =========================
   MAIN
   ========================= */
(async () => {
  // MQTT connect
  const mqttOpts = (MQTT_USER || MQTT_PASS)
    ? { username: MQTT_USER, password: MQTT_PASS }
    : {};
  const mq = mqtt.connect(MQTT_URL, mqttOpts);

  await new Promise((resolve, reject) => {
    mq.once('connect', resolve);
    mq.once('error', reject);
  });

  // Modbus connect
  const mb = new ModbusRTU();
  await mb.connectRTUBuffered(SERIAL_PORT, {
    baudRate: BAUD,
    parity: 'none',
    dataBits: 8,
    stopBits: 1,
  });
  mb.setID(SLAVE_ID);
  mb.setTimeout(1000);

  console.log(`DEIF ? MQTT started (MEAS 500+ only): slave=${SLAVE_ID} port=${SERIAL_PORT} mqtt=${MQTT_URL}`);

  const handleCommand = createCommandHandler(mb, ENABLED_COMMANDS);
  const cmdTopicSet = new Set(ENABLED_COMMANDS.map(c => c.topic));

  if (cmdTopicSet.size > 0) {
    mq.subscribe(Array.from(cmdTopicSet));
    mq.on('message', (topic, message, packet) => {
      if (!cmdTopicSet.has(topic)) return;
      if (packet && packet.retain) return; // ignore retained commands
      handleCommand(topic);
    });
    const enabledNames = ENABLED_COMMANDS.map(c => c.key).join(', ');
    console.log(`Command topics enabled (${cmdTopicSet.size} topics, cooldown ${CMD_COOLDOWN_MS}ms): ${enabledNames}`);
  } else {
    console.log('Command topics disabled (no ENABLE_COMMAND_* flags set)');
  }

  // Send HA discovery ONCE (retained)
  publishHassDiscovery(mq);

  // Publish static metadata once (retained)
  publish(mq, 'device/model', DEVICE_MODEL, true);
  publish(mq, 'device/manufacturer', DEVICE_MANUFACTURER, true);
  publish(mq, 'device/name', DEVICE_NAME, true);

  let prevActiveAlarmKeys = new Set();
  let prevEngineRunning = null;
  let lastRunStarted = null;
  let lastRunStopped = null;
  let lastRunStartedMs = null;
  let lastRunDurationSeconds = null;

  async function readAndPublish() {
    // Read measurement table block 500..576
    const b = await readInputBlock(mb, MEAS_START, MEAS_COUNT);

    // Read alarm bitfield registers 1000..1019 (includes status at 1018-1019)
    const alarmRegs = await readInputBlock(mb, ALARM_START, ALARM_COUNT);

    const appRaw = getReg(b, R.APP_VERSION);
    const appVersion = fmtAppVersion(appRaw);

    const gen = readGen(b);
    const mains = readMains(b);
    const runHours = readRunHours(b);

    // Energy counter (must be non-negative for HA energy sensor)
    const { energyKwh, energySignedKwh } = readEnergy(b);

    const alarms = {
      count: getReg(b, R.ALARM_COUNT),
      unacknowledged: getReg(b, R.ALARM_UNACK),
      ack_active: getReg(b, R.ALARM_ACK_ACTIVE),
      active: []
    };

    if (PUBLISH_ALARM_BITFIELDS) {
      alarms.bitfield = {};
      for (let i = 0; i < ALARM_COUNT; i++) {
        const regAddr = ALARM_START + i;
        alarms.bitfield[regAddr] = toHex(alarmRegs[i]);
      }
    }

    // Decode active alarms with descriptions
    alarms.active = decodeAlarms(alarmRegs);

    const currentActiveAlarmKeys = new Set();
    const activatedAlarms = [];
    for (const a of alarms.active) {
      const key = `${a.register}:${a.bit}`;
      currentActiveAlarmKeys.add(key);
      if (!prevActiveAlarmKeys.has(key)) activatedAlarms.push(a);
    }

    const clearedAlarms = [];
    for (const key of prevActiveAlarmKeys) {
      if (!currentActiveAlarmKeys.has(key)) clearedAlarms.push(key);
    }

    if (activatedAlarms.length > 0) {
      const msg = activatedAlarms.map(a => `${a.register}:${a.bit} (${a.code} ${a.text})`).join('; ');
      console.log(`Alarms set: ${msg}`);
    }
    if (clearedAlarms.length > 0) {
      const msg = clearedAlarms.map(describeAlarmKey).join('; ');
      console.log(`Alarms cleared: ${msg}`);
    }

    prevActiveAlarmKeys = currentActiveAlarmKeys;
    alarms.active_text = formatActiveAlarms(alarms.active);

    // Decode status bits from registers 1018-1019
    const status = decodeStatus(alarmRegs);
    status.operating_mode = getOperatingModeText(status);

    // Engine run/stop logging based on status 1018:6 (Engine running)
    const engineRunning = !!status['1018_6'];
    if (prevEngineRunning === null) {
      prevEngineRunning = engineRunning;
    } else if (engineRunning !== prevEngineRunning) {
      console.log(engineRunning ? 'Engine status: STARTED' : 'Engine status: STOPPED');
      const nowIso = new Date().toISOString();
      if (engineRunning) {
        lastRunStarted = nowIso;
        lastRunStartedMs = Date.now();
        publish(mq, 'status/last_run_started', lastRunStarted, true);
      } else {
        lastRunStopped = nowIso;
        const stopMs = Date.now();
        if (lastRunStartedMs !== null) {
          const durationSec = Math.max(0, Math.round((stopMs - lastRunStartedMs) / 1000));
          lastRunDurationSeconds = durationSec;
          publish(mq, 'status/last_run_duration_s', lastRunDurationSeconds, true);
        }
        publish(mq, 'status/last_run_stopped', lastRunStopped, true);
      }
      prevEngineRunning = engineRunning;
    }

    const counters = readCounters(b);

    const rpm = getReg(b, R.RPM);
    const usupplyV = readUsupply(b);

    publishFlat(mq, 'device', {
      id: HASS_DEVICE_ID,
      name: DEVICE_NAME,
      manufacturer: DEVICE_MANUFACTURER,
      model: DEVICE_MODEL,
    }, true);
    publishFlat(mq, 'gen', gen, RETAIN);
    publishFlat(mq, 'mains', mains, RETAIN);
    publish(mq, 'run_hours', runHours, RETAIN);
    publish(mq, 'energy_kwh', energyKwh, RETAIN);
    publish(mq, 'energy_signed_kwh', energySignedKwh, RETAIN);
    publishFlat(mq, 'alarms', alarms, RETAIN);
    publishFlat(mq, 'counters', counters, RETAIN);
    publishFlat(mq, 'status', status, RETAIN);
    publish(mq, 'rpm', rpm, RETAIN);
    publish(mq, 'usupply_v', usupplyV, RETAIN);
    publish(mq, 'last_run_started', lastRunStarted, RETAIN);
    publish(mq, 'last_run_stopped', lastRunStopped, RETAIN);
    publish(mq, 'last_run_duration_s', lastRunDurationSeconds, RETAIN);
    publish(mq, 'ts', new Date().toISOString(), RETAIN);
  }

  const run = async () => {
    try {
      await readAndPublish();
    } catch (err) {
      console.error('Poll error:', err && err.message ? err.message : err);
    }
  };

  if (INTERVAL_MS <= 0) {
    await run();
    process.exit(0);
  } else {
    await run();
    setInterval(run, INTERVAL_MS);
  }
})().catch(err => {
  console.error(err);
  process.exit(1);
});
