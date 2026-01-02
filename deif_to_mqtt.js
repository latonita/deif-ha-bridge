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

// Static device metadata
const DEVICE_MODEL = process.env.DEVICE_MODEL || 'DEIF GC-1F/2';
const DEVICE_MANUFACTURER = process.env.DEVICE_MANUFACTURER || 'DEIF';
const DEVICE_NAME = process.env.DEVICE_NAME || `DEIF GC-1F/2 (${SLAVE_ID})`;

// HA Discovery
const HASS_DISCOVERY_PREFIX = (process.env.HASS_DISCOVERY_PREFIX || 'homeassistant').replace(/\/+$/, '');
const HASS_NODE_ID = process.env.HASS_NODE_ID || `deif-gc1f2-${SLAVE_ID}`;
const HASS_DEVICE_ID = process.env.HASS_DEVICE_ID || HASS_NODE_ID;

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

  USUPPLY: 567, // V/10 Supply voltage i.e. battery/PSU
};

// We'll read one continuous block from 500 to 567 inclusive.
const MEAS_START = 500;
const MEAS_END = 567;
const MEAS_COUNT = (MEAS_END - MEAS_START) + 1;

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
  const major = Math.floor(raw / 100);
  const minor = raw % 100;
  return `${major}.${String(minor).padStart(2, '0')}`;
}

function freqFloat(x) {
  return Number.parseFloat(Number(x).toFixed(FREQ_DECIMALS));
}

function publish(mq, key, value, retainOverride) {
  const topic = `${TOPIC_PREFIX}/${key}`;
  const retain = (typeof retainOverride === 'boolean') ? retainOverride : RETAIN;
  const payload = (typeof value === 'object') ? JSON.stringify(value) : String(value);
  mq.publish(topic, payload, { qos: 0, retain });
}

async function readInputBlock(mb, start, count) {
  const res = await mb.readInputRegisters(start, count);
  return res.data; // array of 16-bit register values
}

function getReg(block, addr) {
  const idx = addr - MEAS_START;
  if (idx < 0 || idx >= block.length) return undefined;
  return block[idx];
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

  function pubSensor(key, cfg) {
    const objectId = `${HASS_NODE_ID}-${key}`;
    const topic = `${HASS_DISCOVERY_PREFIX}/sensor/${HASS_NODE_ID}/${key}/config`;

    const payload = {
      name: cfg.name,
      uniq_id: objectId,
      obj_id: objectId,
      stat_t: stateTopic,
      val_tpl: `{{ value_json.${cfg.jsonPath} | is_defined }}`,
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

    // Counters / diagnostics
    { key: 'run_hours', name: 'Generator Run Hours', jsonPath: 'run_hours', unit: 'h', stateClass: 'total_increasing', entityCategory: 'diagnostic', icon: 'mdi:timer-outline' },

    // MUST be non-negative for HA energy device_class
    { key: 'energy_kwh', name: 'Generator Energy', jsonPath: 'energy_kwh', deviceClass: 'energy', unit: 'kWh', stateClass: 'total_increasing', entityCategory: 'diagnostic', icon: 'mdi:counter' },

    // Signed energy (negative) as diagnostic only
    { key: 'energy_signed_kwh', name: 'Generator Energy (Signed)', jsonPath: 'energy_signed_kwh', unit: 'kWh', entityCategory: 'diagnostic', icon: 'mdi:swap-horizontal' },

    { key: 'usupply_v', name: 'Battery Voltage', jsonPath: 'usupply_v', deviceClass: 'voltage', unit: 'V', stateClass: 'measurement', entityCategory: 'diagnostic', icon: 'mdi:car-battery' },
  ];

  for (const s of sensors) pubSensor(s.key, s);
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

  // Send HA discovery ONCE (retained)
  publishHassDiscovery(mq);

  // Publish static metadata once (retained)
  publish(mq, 'device/model', DEVICE_MODEL, true);
  publish(mq, 'device/manufacturer', DEVICE_MANUFACTURER, true);
  publish(mq, 'device/name', DEVICE_NAME, true);

  async function pollOnce() {
    // Read measurement table block 500..567
    const b = await readInputBlock(mb, MEAS_START, MEAS_COUNT);

    const appRaw = getReg(b, R.APP_VERSION);
    const appVersion = fmtAppVersion(appRaw);

    const gen = {
      voltage_l1n_v: getReg(b, R.GEN_U_L1N),                 // int
      voltage_l2n_v: getReg(b, R.GEN_U_L2N),                 // int
      voltage_l3n_v: getReg(b, R.GEN_U_L3N),                 // int
      current_l1_a: getReg(b, R.GEN_I_L1),                   // int
      current_l2_a: getReg(b, R.GEN_I_L2),                   // int
      current_l3_a: getReg(b, R.GEN_I_L3),                   // int
      frequency_hz: freqFloat(getReg(b, R.GEN_F) / FREQ_DIVISOR), // decimal
      pgen_kw: s16(getReg(b, R.PGEN)),                       // signed per manual
      qgen_kvar: s16(getReg(b, R.QGEN)),                     // keep signed to be safe
      sgen_kva: s16(getReg(b, R.SGEN)),                      // keep signed to be safe
      cos_phi: s16(getReg(b, R.COS_PHI)) / 100.0,            // signed, cosf x100
    };

    const mains = {
      voltage_l1n_v: getReg(b, R.MAINS_U_L1N),               // int
      voltage_l2n_v: getReg(b, R.MAINS_U_L2N),               // int
      voltage_l3n_v: getReg(b, R.MAINS_U_L3N),               // int
      frequency_hz: freqFloat(getReg(b, R.MAINS_F) / FREQ_DIVISOR), // decimal
    };

    const runHours = u32(getReg(b, R.RUN_HI), getReg(b, R.RUN_LO));

    // Energy counter (must be non-negative for HA energy sensor)
    const energyKwh = u32(getReg(b, R.EGEN_HI), getReg(b, R.EGEN_LO));
    const energySignedKwh = -energyKwh;

    const usupplyV = Number.parseFloat((getReg(b, R.USUPPLY) / 10.0).toFixed(1));

    publish(mq, 'state', {
      device: {
        id: HASS_DEVICE_ID,
        name: DEVICE_NAME,
        manufacturer: DEVICE_MANUFACTURER,
        model: DEVICE_MODEL,
      },
      app_version: appVersion,
      gen,
      mains,
      run_hours: runHours,
      energy_kwh: energyKwh,
      energy_signed_kwh: energySignedKwh,
      usupply_v: usupplyV,
      ts: new Date().toISOString(),
    });
  }

  const run = async () => {
    try {
      await pollOnce();
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
