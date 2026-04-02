package main

import (
	"encoding/binary"
	"encoding/json"
	"fmt"
	"log"
	"os"
	"os/signal"
	"sort"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/goburrow/modbus"
)

const (
	defaultSerialPort = "/dev/ttyUSB0"
	defaultBaud       = 9600
	defaultSlaveID    = 1
	defaultMQTTURL    = "mqtt://localhost:1883"
	defaultTopicPref  = "deif/gc1f2"
	defaultIntervalMS = 5000
	defaultRetain     = true
	defaultCooldownMS = 5000
	defaultDeviceName = "DEIF GC-1F/2"

	defaultDiscoveryPrefix = "homeassistant"
	defaultFreqDivisor     = 10.0
	defaultFreqDecimals    = 1

	measStart  = 500
	measEnd    = 576
	measCount  = measEnd - measStart + 1
	alarmStart = 1000
	alarmEnd   = 1019
	alarmCount = alarmEnd - alarmStart + 1

	cmdFlagStart       = 1
	cmdFlagGBOn        = 2
	cmdFlagGBOff       = 3
	cmdFlagStop        = 4
	cmdFlagAlarmAck    = 10
	cmdFlagStartGBOn   = 15
	cmdFlagGBOffStop   = 16
	cmdFlagMBOn        = 25
	cmdFlagMBOff       = 26
	cmdFlagManualMode  = 28
	cmdFlagAutoMode    = 30
	cmdFlagTest        = 31
	modbusTimeout      = time.Second
	mqttPublishTimeout = 10 * time.Second
)

type registers struct {
	AppVersion    uint16
	GenUL1N       uint16
	GenUL2N       uint16
	GenUL3N       uint16
	GenF          uint16
	GenIL1        uint16
	GenIL2        uint16
	GenIL3        uint16
	GenPL1        uint16
	GenPL2        uint16
	GenPL3        uint16
	PGen          uint16
	QGen          uint16
	SGen          uint16
	EGenHi        uint16
	EGenLo        uint16
	CosPhi        uint16
	MainsUL1N     uint16
	MainsUL2N     uint16
	MainsUL3N     uint16
	MainsF        uint16
	RunHi         uint16
	RunLo         uint16
	AlarmCount    uint16
	AlarmUnack    uint16
	AlarmAckAct   uint16
	GBOperations  uint16
	MBOperations  uint16
	StartAttempts uint16
	USupply       uint16
	RPM           uint16
}

var R = registers{
	AppVersion:    500,
	GenUL1N:       504,
	GenUL2N:       505,
	GenUL3N:       506,
	GenF:          507,
	GenIL1:        513,
	GenIL2:        514,
	GenIL3:        515,
	GenPL1:        516,
	GenPL2:        517,
	GenPL3:        518,
	PGen:          519,
	QGen:          523,
	SGen:          527,
	EGenHi:        536,
	EGenLo:        537,
	CosPhi:        538,
	MainsUL1N:     542,
	MainsUL2N:     543,
	MainsUL3N:     544,
	MainsF:        545,
	RunHi:         554,
	RunLo:         555,
	AlarmCount:    558,
	AlarmUnack:    559,
	AlarmAckAct:   560,
	GBOperations:  563,
	MBOperations:  564,
	StartAttempts: 566,
	USupply:       567,
	RPM:           576,
}

type config struct {
	SerialPort         string
	Baud               int
	SlaveID            int
	MQTTURL            string
	MQTTUser           string
	MQTTPass           string
	TopicPrefix        string
	Interval           time.Duration
	Retain             bool
	CommandCooldown    time.Duration
	DeviceModel        string
	DeviceManufacturer string
	DeviceName         string
	DiscoveryPrefix    string
	NodeID             string
	DeviceID           string
	FreqDivisor        float64
	FreqDecimals       int

	EnableAlarmAck   bool
	EnableManualMode bool
	EnableAutoMode   bool
	EnableStart      bool
	EnableGBOn       bool
	EnableGBOff      bool
	EnableStop       bool
	EnableStartGBOn  bool
	EnableGBOffStop  bool
	EnableMBOn       bool
	EnableMBOff      bool
	EnableTest       bool

	PublishAlarmBitfields bool
	PublishStatusFlags    bool
}

type alarmDefinition struct {
	Code string
	Text string
}

type activeAlarm struct {
	Register uint16 `json:"register"`
	Bit      uint8  `json:"bit"`
	Code     string `json:"code"`
	Text     string `json:"text"`
}

type commandDef struct {
	Key     string
	Name    string
	Topic   string
	Offset  uint16
	Enabled bool
	Icon    string
}

type hassSensor struct {
	Key            string
	Name           string
	JSONPath       string
	Icon           string
	DeviceClass    string
	StateClass     string
	Unit           string
	EntityCategory string
}

type hassBinarySensor struct {
	Key               string
	Name              string
	JSONPath          string
	Icon              string
	DeviceClass       string
	EntityCategory    string
	ValueTemplateFlat string
}

type bridge struct {
	cfg             config
	commandDefs     []commandDef
	commandByTopic  map[string]commandDef
	modbusHandler   *modbus.RTUClientHandler
	modbusClient    modbus.Client
	mqttClient      mqtt.Client
	modbusMu        sync.Mutex
	commandMu       sync.Mutex
	lastCommandAt   time.Time
	prevAlarmKeys   map[string]struct{}
	prevEngineState *bool
	lastRunStarted  string
	lastRunStopped  string
	lastRunStartAt  time.Time
	lastRunDuration *int64
	lastAlarmSetAt  string
	lastAlarmSum    string
	lastAlarmClrAt  string
	lastAlarmText   string
	bitfieldsCleared bool
	statusFlagsCleared bool
}

var alarmMap = map[string]alarmDefinition{
	"1000:0":  {Code: "1000", Text: "G -P> 1"},
	"1000:3":  {Code: "1030", Text: "G I> 1"},
	"1000:4":  {Code: "1040", Text: "G I> 2"},
	"1000:9":  {Code: "1130", Text: "G I>> 1"},
	"1000:10": {Code: "1140", Text: "G I>> 2"},
	"1000:11": {Code: "1150", Text: "G U> 1"},
	"1000:12": {Code: "1160", Text: "G U> 2"},
	"1000:13": {Code: "1170", Text: "G U< 1"},
	"1000:14": {Code: "1180", Text: "G U< 2"},

	"1001:0":  {Code: "1210", Text: "G f> 1"},
	"1001:1":  {Code: "1220", Text: "G f> 2"},
	"1001:3":  {Code: "1240", Text: "G f< 1"},
	"1001:4":  {Code: "1250", Text: "G f< 2"},
	"1001:6":  {Code: "no code", Text: "BB U> 1"},
	"1001:7":  {Code: "no code", Text: "BB U> 2"},
	"1001:9":  {Code: "no code", Text: "BB U< 1"},
	"1001:10": {Code: "no code", Text: "BB U< 2"},
	"1001:13": {Code: "no code", Text: "BB f> 1"},
	"1001:14": {Code: "no code", Text: "BB f> 2"},

	"1002:0":  {Code: "no code", Text: "BB f< 1"},
	"1002:1":  {Code: "no code", Text: "BB f< 2"},
	"1002:7":  {Code: "1450", Text: "G P> 1"},
	"1002:8":  {Code: "1460", Text: "G P> 2"},
	"1002:14": {Code: "no code", Text: "-Q>"},
	"1002:15": {Code: "no code", Text: "Q>"},

	"1003:7": {Code: "1620", Text: "Mains unbalanced voltage"},

	"1005:3": {Code: "2150", Text: "Phase seq error"},
	"1005:4": {Code: "2160", Text: "GB open failure"},
	"1005:5": {Code: "2170", Text: "GB close failure"},
	"1005:6": {Code: "no code", Text: "GB pos failure"},
	"1005:7": {Code: "2200", Text: "MB open failure"},
	"1005:8": {Code: "2210", Text: "MB close failure"},
	"1005:9": {Code: "no code", Text: "MB pos failure"},

	"1010:0":  {Code: "3400", Text: "Dig. multi-input 1"},
	"1010:1":  {Code: "3410", Text: "Dig. multi-input 2"},
	"1010:2":  {Code: "3420", Text: "Dig. multi-input 3"},
	"1010:3":  {Code: "3404", Text: "Wire failure, dig. multi-input 1"},
	"1010:4":  {Code: "3404", Text: "Wire failure, dig. multi-input 2"},
	"1010:5":  {Code: "3424", Text: "Wire failure, dig. multi-input 3"},
	"1010:12": {Code: "3490", Text: "Dig. input 19-20/Emergency STOP"},

	"1013:0":  {Code: "no code", Text: "Multi-input 1.1"},
	"1013:1":  {Code: "no code", Text: "Multi-input 1.2"},
	"1013:2":  {Code: "no code", Text: "W. failure, multi-input 1"},
	"1013:3":  {Code: "no code", Text: "Multi-input 2.1"},
	"1013:4":  {Code: "no code", Text: "Multi-input 2.2"},
	"1013:5":  {Code: "no code", Text: "W. failure, multi-input 2"},
	"1013:6":  {Code: "no code", Text: "Multi-input 3.1"},
	"1013:7":  {Code: "no code", Text: "Multi-input 3.2"},
	"1013:8":  {Code: "no code", Text: "W. failure, multi-input 3"},
	"1013:9":  {Code: "4510", Text: "Overspeed 1"},
	"1013:10": {Code: "4520", Text: "Overspeed 2"},
	"1013:11": {Code: "4620", Text: "VDO fuel level 1.3"},
	"1013:12": {Code: "4610", Text: "Charger gen"},
	"1013:13": {Code: "4600", Text: "V-Belt"},
	"1013:14": {Code: "4560", Text: "Generator Hz/V failure"},
	"1013:15": {Code: "no code", Text: "Start failure"},

	"1014:0": {Code: "4580", Text: "Stop failure"},
	"1014:1": {Code: "4960", Text: "U< aux. supply term. 1"},
	"1014:2": {Code: "4970", Text: "U> aux. supply term. 1"},
	"1014:5": {Code: "4610", Text: "Charger Gen"},

	"1015:0":  {Code: "6110", Text: "Service timer 1"},
	"1015:1":  {Code: "6120", Text: "Service timer 2"},
	"1015:13": {Code: "no code", Text: "Fuel fill check"},
}

var statusMap = map[string]alarmDefinition{
	"1018:0":  {Code: "no code", Text: "Mains failure"},
	"1018:1":  {Code: "no code", Text: "MB pos ON"},
	"1018:4":  {Code: "no code", Text: "GB pos ON"},
	"1018:6":  {Code: "no code", Text: "Engine running"},
	"1018:7":  {Code: "no code", Text: "Running detection, timer expired"},
	"1018:8":  {Code: "no code", Text: "DG Hz/V OK, timer expired"},
	"1019:0":  {Code: "no code", Text: "OFF"},
	"1019:1":  {Code: "no code", Text: "Manual"},
	"1019:3":  {Code: "no code", Text: "Auto"},
	"1019:4":  {Code: "no code", Text: "Test"},
	"1019:5":  {Code: "no code", Text: "Island"},
	"1019:6":  {Code: "no code", Text: "AMF"},
	"1019:10": {Code: "no code", Text: "Load take over"},
	"1019:15": {Code: "no code", Text: "AMF active"},
}

var (
	alarmRegisterSet = registerSet(alarmMap)
	statusRegisters  = registerList(statusMap)
	alarmBitfieldRegisters = []string{
		"1000", "1001", "1002", "1003", "1004", "1005", "1006", "1007", "1008", "1009",
		"1010", "1011", "1012", "1013", "1014", "1015", "1016", "1017", "1018", "1019",
	}
	statusFlagBinarySensors = []string{
		"status_mains_failure",
		"status_mb_on",
		"status_gb_on",
		"status_engine_running",
		"status_gen_ok",
		"status_running_timer",
		"mode_off",
		"mode_manual",
		"mode_auto",
		"mode_test",
		"mode_island",
		"mode_amf",
		"load_takeover",
		"amf_active",
	}
)

func main() {
	cfg := loadConfig()
	cmdDefs := buildCommandDefs(cfg)

	handler := modbus.NewRTUClientHandler(cfg.SerialPort)
	handler.BaudRate = cfg.Baud
	handler.DataBits = 8
	handler.Parity = "N"
	handler.StopBits = 1
	handler.Timeout = modbusTimeout
	handler.SlaveId = byte(cfg.SlaveID)

	if err := handler.Connect(); err != nil {
		log.Fatalf("modbus connect failed: %v", err)
	}
	defer handler.Close()

	mqOpts := mqtt.NewClientOptions().
		AddBroker(cfg.MQTTURL).
		SetClientID(cfg.NodeID + "-bridge").
		SetAutoReconnect(true).
		SetConnectRetry(true).
		SetOrderMatters(false)
	if cfg.MQTTUser != "" || cfg.MQTTPass != "" {
		mqOpts.SetUsername(cfg.MQTTUser)
		mqOpts.SetPassword(cfg.MQTTPass)
	}

	b := &bridge{
		cfg:            cfg,
		commandDefs:    cmdDefs,
		commandByTopic: make(map[string]commandDef, len(cmdDefs)),
		modbusHandler:  handler,
		modbusClient:   modbus.NewClient(handler),
		prevAlarmKeys:  make(map[string]struct{}),
	}
	for _, cmd := range cmdDefs {
		b.commandByTopic[cmd.Topic] = cmd
	}

	mqOpts.SetDefaultPublishHandler(func(_ mqtt.Client, msg mqtt.Message) {
		if _, ok := b.commandByTopic[msg.Topic()]; !ok {
			return
		}
		if msg.Retained() {
			return
		}
		go b.handleCommand(msg.Topic())
	})
	mqOpts.OnConnect = func(client mqtt.Client) {
		b.mqttClient = client
		if len(b.commandDefs) > 0 {
			topics := make(map[string]byte, len(b.commandDefs))
			for _, cmd := range b.commandDefs {
				topics[cmd.Topic] = 0
			}
			token := client.SubscribeMultiple(topics, nil)
			if !token.WaitTimeout(mqttPublishTimeout) {
				log.Printf("mqtt subscribe timeout")
			} else if err := token.Error(); err != nil {
				log.Printf("mqtt subscribe error: %v", err)
			}
		}
		if err := b.publishHassDiscovery(); err != nil {
			log.Printf("discovery publish error: %v", err)
		}
		b.publishStaticMetadata()
	}
	mqOpts.OnConnectionLost = func(_ mqtt.Client, err error) {
		log.Printf("mqtt connection lost: %v", err)
	}

	client := mqtt.NewClient(mqOpts)
	if token := client.Connect(); !token.WaitTimeout(mqttPublishTimeout) {
		log.Fatalf("mqtt connect timeout")
	} else if err := token.Error(); err != nil {
		log.Fatalf("mqtt connect failed: %v", err)
	}
	defer client.Disconnect(250)
	b.mqttClient = client

	log.Printf("DEIF -> MQTT started: slave=%d port=%s mqtt=%s", cfg.SlaveID, cfg.SerialPort, cfg.MQTTURL)
	if len(b.commandDefs) > 0 {
		names := make([]string, 0, len(b.commandDefs))
		for _, cmd := range b.commandDefs {
			names = append(names, cmd.Key)
		}
		log.Printf("Command topics enabled (%d topics, cooldown %s): %s", len(b.commandDefs), cfg.CommandCooldown, strings.Join(names, ", "))
	} else {
		log.Printf("Command topics disabled (no ENABLE_COMMAND_* flags set)")
	}

	stop := make(chan os.Signal, 1)
	signal.Notify(stop, os.Interrupt, syscall.SIGTERM)

	if cfg.Interval <= 0 {
		if err := b.readAndPublish(); err != nil {
			log.Printf("poll error: %v", err)
		}
		return
	}

	if err := b.readAndPublish(); err != nil {
		log.Printf("poll error: %v", err)
	}

	ticker := time.NewTicker(cfg.Interval)
	defer ticker.Stop()

	for {
		select {
		case <-stop:
			return
		case <-ticker.C:
			if err := b.readAndPublish(); err != nil {
				log.Printf("poll error: %v", err)
			}
		}
	}
}

func loadConfig() config {
	slaveID := envInt("DEIF_SLAVE", defaultSlaveID)
	nodeID := envString("HASS_NODE_ID", fmt.Sprintf("deif-gc1f2-%d", slaveID))
	return config{
		SerialPort:            envString("DEIF_PORT", defaultSerialPort),
		Baud:                  envInt("DEIF_BAUD", defaultBaud),
		SlaveID:               slaveID,
		MQTTURL:               envString("MQTT_URL", defaultMQTTURL),
		MQTTUser:              envString("MQTT_USER", ""),
		MQTTPass:              envString("MQTT_PASS", ""),
		TopicPrefix:           trimRightSlash(envString("TOPIC_PREFIX", defaultTopicPref)),
		Interval:              time.Duration(envInt("INTERVAL_MS", defaultIntervalMS)) * time.Millisecond,
		Retain:                envBool("RETAIN", defaultRetain),
		CommandCooldown:       time.Duration(envInt("CMD_COOLDOWN_MS", defaultCooldownMS)) * time.Millisecond,
		DeviceModel:           envString("DEVICE_MODEL", defaultDeviceName),
		DeviceManufacturer:    envString("DEVICE_MANUFACTURER", "DEIF"),
		DeviceName:            envString("DEVICE_NAME", fmt.Sprintf("%s (%d)", defaultDeviceName, slaveID)),
		DiscoveryPrefix:       trimRightSlash(envString("HASS_DISCOVERY_PREFIX", defaultDiscoveryPrefix)),
		NodeID:                nodeID,
		DeviceID:              envString("HASS_DEVICE_ID", nodeID),
		FreqDivisor:           envFloat("FREQ_DIVISOR", defaultFreqDivisor),
		FreqDecimals:          envInt("FREQ_DECIMALS", defaultFreqDecimals),
		EnableAlarmAck:        envBool("ENABLE_COMMAND_ALARM_ACK", false),
		EnableManualMode:      envBool("ENABLE_COMMAND_MANUAL_MODE", false),
		EnableAutoMode:        envBool("ENABLE_COMMAND_AUTO_MODE", false),
		EnableStart:           envBool("ENABLE_COMMAND_START", false),
		EnableGBOn:            envBool("ENABLE_COMMAND_GB_ON", false),
		EnableGBOff:           envBool("ENABLE_COMMAND_GB_OFF", false),
		EnableStop:            envBool("ENABLE_COMMAND_STOP", false),
		EnableStartGBOn:       envBool("ENABLE_COMMAND_START_GB_ON", false),
		EnableGBOffStop:       envBool("ENABLE_COMMAND_GB_OFF_STOP", false),
			EnableMBOn:            envBool("ENABLE_COMMAND_MB_ON", false),
			EnableMBOff:           envBool("ENABLE_COMMAND_MB_OFF", false),
			EnableTest:            envBool("ENABLE_COMMAND_TEST", false),
			PublishAlarmBitfields: envBool("PUBLISH_ALARM_BITFIELDS", false),
			PublishStatusFlags:    envBool("PUBLISH_STATUS_FLAGS", false),
		}
}

func envString(name, fallback string) string {
	if value, ok := os.LookupEnv(name); ok {
		return value
	}
	return fallback
}

func envInt(name string, fallback int) int {
	value := envString(name, "")
	if value == "" {
		return fallback
	}
	parsed, err := strconv.Atoi(value)
	if err != nil {
		return fallback
	}
	return parsed
}

func envFloat(name string, fallback float64) float64 {
	value := envString(name, "")
	if value == "" {
		return fallback
	}
	parsed, err := strconv.ParseFloat(value, 64)
	if err != nil {
		return fallback
	}
	return parsed
}

func envBool(name string, fallback bool) bool {
	value := strings.ToLower(strings.TrimSpace(envString(name, "")))
	if value == "" {
		return fallback
	}
	return value == "true"
}

func trimRightSlash(value string) string {
	return strings.TrimRight(value, "/")
}

func buildCommandDefs(cfg config) []commandDef {
	commands := []commandDef{
		{Key: "alarm_ack", Name: "Alarm Acknowledge", Topic: cfg.TopicPrefix + "/cmd/alarm_ack", Offset: cmdFlagAlarmAck, Enabled: cfg.EnableAlarmAck, Icon: "mdi:alarm-check"},
		{Key: "start", Name: "Start", Topic: cfg.TopicPrefix + "/cmd/start", Offset: cmdFlagStart, Enabled: cfg.EnableStart, Icon: "mdi:play-circle"},
		{Key: "gb_on", Name: "GB ON", Topic: cfg.TopicPrefix + "/cmd/gb_on", Offset: cmdFlagGBOn, Enabled: cfg.EnableGBOn, Icon: "mdi:flash"},
		{Key: "gb_off", Name: "GB OFF", Topic: cfg.TopicPrefix + "/cmd/gb_off", Offset: cmdFlagGBOff, Enabled: cfg.EnableGBOff, Icon: "mdi:flash-off"},
		{Key: "stop", Name: "Stop", Topic: cfg.TopicPrefix + "/cmd/stop", Offset: cmdFlagStop, Enabled: cfg.EnableStop, Icon: "mdi:stop-circle"},
		{Key: "start_gb_on", Name: "Start + GB ON", Topic: cfg.TopicPrefix + "/cmd/start_gb_on", Offset: cmdFlagStartGBOn, Enabled: cfg.EnableStartGBOn, Icon: "mdi:play-network"},
		{Key: "gb_off_stop", Name: "GB OFF + Stop", Topic: cfg.TopicPrefix + "/cmd/gb_off_stop", Offset: cmdFlagGBOffStop, Enabled: cfg.EnableGBOffStop, Icon: "mdi:power-plug-off"},
		{Key: "mb_on", Name: "MB ON", Topic: cfg.TopicPrefix + "/cmd/mb_on", Offset: cmdFlagMBOn, Enabled: cfg.EnableMBOn, Icon: "mdi:transmission-tower-import"},
		{Key: "mb_off", Name: "MB OFF", Topic: cfg.TopicPrefix + "/cmd/mb_off", Offset: cmdFlagMBOff, Enabled: cfg.EnableMBOff, Icon: "mdi:transmission-tower-off"},
		{Key: "mode_manual", Name: "Mode: Manual", Topic: cfg.TopicPrefix + "/cmd/mode_manual", Offset: cmdFlagManualMode, Enabled: cfg.EnableManualMode, Icon: "mdi:hand-back-right"},
		{Key: "mode_auto", Name: "Mode: Auto", Topic: cfg.TopicPrefix + "/cmd/mode_auto", Offset: cmdFlagAutoMode, Enabled: cfg.EnableAutoMode, Icon: "mdi:autorenew"},
		{Key: "mode_test", Name: "Mode: Test", Topic: cfg.TopicPrefix + "/cmd/mode_test", Offset: cmdFlagTest, Enabled: cfg.EnableTest, Icon: "mdi:beaker"},
	}
	enabled := make([]commandDef, 0, len(commands))
	for _, cmd := range commands {
		if cmd.Enabled {
			enabled = append(enabled, cmd)
		}
	}
	return enabled
}

func (b *bridge) handleCommand(topic string) {
	cmd, ok := b.commandByTopic[topic]
	if !ok {
		return
	}

	b.commandMu.Lock()
	now := time.Now()
	if now.Sub(b.lastCommandAt) < b.cfg.CommandCooldown {
		b.commandMu.Unlock()
		log.Printf("Command skipped (cooldown active): %s", cmd.Key)
		return
	}
	b.lastCommandAt = now
	b.commandMu.Unlock()

	log.Printf("Command received: %s (topic=%s, offset=%d)", cmd.Key, topic, cmd.Offset)

	b.modbusMu.Lock()
	defer b.modbusMu.Unlock()
	if _, err := b.modbusClient.WriteMultipleCoils(cmd.Offset, 1, []byte{0x01}); err != nil {
		log.Printf("Command error %s: %v", cmd.Key, err)
		return
	}
	log.Printf("Command sent: %s", cmd.Key)
}

func (b *bridge) readAndPublish() error {
	b.modbusMu.Lock()
	measBlock, err := b.readInputBlock(measStart, measCount)
	if err == nil {
		var alarmRegs []uint16
		alarmRegs, err = b.readInputBlock(alarmStart, alarmCount)
		if err == nil {
			err = b.publishPoll(measBlock, alarmRegs)
		}
	}
	b.modbusMu.Unlock()
	return err
}

func (b *bridge) readInputBlock(start, count int) ([]uint16, error) {
	raw, err := b.modbusClient.ReadInputRegisters(uint16(start), uint16(count))
	if err != nil {
		return nil, err
	}
	if len(raw)%2 != 0 {
		return nil, fmt.Errorf("unexpected register byte length %d", len(raw))
	}
	values := make([]uint16, len(raw)/2)
	for i := 0; i < len(values); i++ {
		values[i] = binary.BigEndian.Uint16(raw[i*2 : i*2+2])
	}
	return values, nil
}

func (b *bridge) publishPoll(measBlock, alarmRegs []uint16) error {
	appVersion := formatAppVersion(getReg(measBlock, R.AppVersion))
	gen := readGen(measBlock, b.cfg)
	mains := readMains(measBlock, b.cfg)
	runHours := readRunHours(measBlock)
	energyKWh, energySignedKWh := readEnergy(measBlock)

	alarms := map[string]any{
		"count":          getReg(measBlock, R.AlarmCount),
		"unacknowledged": getReg(measBlock, R.AlarmUnack),
		"ack_active":     getReg(measBlock, R.AlarmAckAct),
		"active":         []activeAlarm{},
	}
	if b.cfg.PublishAlarmBitfields {
		bitfield := make(map[string]string, alarmCount)
		for i := 0; i < len(alarmRegs); i++ {
			bitfield[strconv.Itoa(alarmStart+i)] = toHex(alarmRegs[i])
		}
		alarms["bitfield"] = bitfield
		b.bitfieldsCleared = false
	} else if !b.bitfieldsCleared {
		if err := b.clearAlarmBitfieldTopics(); err != nil {
			return err
		}
		b.bitfieldsCleared = true
	}

	active := decodeAlarms(alarmRegs)
	alarms["active"] = active
	alarms["active_text"] = formatActiveAlarms(active)

	currentKeys := make(map[string]struct{}, len(active))
	activated := make([]activeAlarm, 0)
	for _, alarm := range active {
		key := fmt.Sprintf("%d:%d", alarm.Register, alarm.Bit)
		currentKeys[key] = struct{}{}
		if _, exists := b.prevAlarmKeys[key]; !exists {
			activated = append(activated, alarm)
		}
	}

	cleared := make([]string, 0)
	for key := range b.prevAlarmKeys {
		if _, exists := currentKeys[key]; !exists {
			cleared = append(cleared, key)
		}
	}

	if len(activated) > 0 {
		parts := make([]string, 0, len(activated))
		for _, alarm := range activated {
			parts = append(parts, fmt.Sprintf("%d:%d (%s %s)", alarm.Register, alarm.Bit, alarm.Code, alarm.Text))
		}
		now := time.Now()
		b.lastAlarmSetAt = now.UTC().Format(time.RFC3339Nano)
		b.lastAlarmSum = strings.Join(parts, "; ")
		b.lastAlarmText = fmt.Sprintf("%s %s", now.Format("2006/01/02 15:04:05"), b.lastAlarmSum)
		if err := b.publish("alarms/last_set_at", b.lastAlarmSetAt, true); err != nil {
			return err
		}
		if err := b.publish("alarms/last_set_summary", b.lastAlarmSum, true); err != nil {
			return err
		}
		if err := b.publish("alarms/last_text", b.lastAlarmText, true); err != nil {
			return err
		}
		log.Printf("Alarms set: %s", b.lastAlarmSum)
	}
	if len(cleared) > 0 {
		parts := make([]string, 0, len(cleared))
		for _, key := range cleared {
			parts = append(parts, describeAlarmKey(key))
		}
		b.lastAlarmClrAt = time.Now().UTC().Format(time.RFC3339Nano)
		if err := b.publish("alarms/last_cleared_at", b.lastAlarmClrAt, true); err != nil {
			return err
		}
		log.Printf("Alarms cleared: %s", strings.Join(parts, "; "))
	}
	b.prevAlarmKeys = currentKeys

	status := decodeStatus(alarmRegs)
	statusFlags := status
	status = map[string]any{
		"operating_mode": operatingModeText(statusFlags),
	}
	if b.cfg.PublishStatusFlags {
		for key, value := range statusFlags {
			status[key] = value
		}
		b.statusFlagsCleared = false
	} else if !b.statusFlagsCleared {
		if err := b.clearStatusFlagTopics(); err != nil {
			return err
		}
		b.statusFlagsCleared = true
	}

	engineRunning := statusFlags["1018_6"] == true
	if b.prevEngineState == nil {
		b.prevEngineState = boolPtr(engineRunning)
	} else if *b.prevEngineState != engineRunning {
		now := time.Now()
		if engineRunning {
			log.Printf("Engine status: STARTED")
			b.lastRunStarted = now.UTC().Format(time.RFC3339Nano)
			b.lastRunStartAt = now
			if err := b.publish("engine/last_run_started", b.lastRunStarted, true); err != nil {
				return err
			}
		} else {
			log.Printf("Engine status: STOPPED")
			b.lastRunStopped = now.UTC().Format(time.RFC3339Nano)
			if !b.lastRunStartAt.IsZero() {
				duration := int64(now.Sub(b.lastRunStartAt).Round(time.Second) / time.Second)
				if duration < 0 {
					duration = 0
				}
				b.lastRunDuration = &duration
				if err := b.publish("engine/last_run_duration_s", duration, true); err != nil {
					return err
				}
			}
			if err := b.publish("engine/last_run_stopped", b.lastRunStopped, true); err != nil {
				return err
			}
		}
		*b.prevEngineState = engineRunning
	}

	counters := map[string]any{
		"gen_breaker_ops":   getReg(measBlock, R.GBOperations),
		"mains_breaker_ops": getReg(measBlock, R.MBOperations),
		"start_attempts":    getReg(measBlock, R.StartAttempts),
		"run_hours":         runHours,
		"energy_kwh":        energyKWh,
		"energy_signed_kwh": energySignedKWh,
	}

	device := map[string]any{
		"id":           b.cfg.DeviceID,
		"name":         b.cfg.DeviceName,
		"manufacturer": b.cfg.DeviceManufacturer,
		"model":        b.cfg.DeviceModel,
		"app_version":  appVersion,
	}

	if err := b.publishFlat("device", device, true); err != nil {
		return err
	}
	if err := b.publishFlat("gen", gen, b.cfg.Retain); err != nil {
		return err
	}
	if err := b.publishFlat("mains", mains, b.cfg.Retain); err != nil {
		return err
	}
	if err := b.publishFlat("alarms", alarms, b.cfg.Retain); err != nil {
		return err
	}
	if err := b.publishFlat("counters", counters, b.cfg.Retain); err != nil {
		return err
	}
	if err := b.publishFlat("status", status, b.cfg.Retain); err != nil {
		return err
	}
	if err := b.publish("engine/rpm", getReg(measBlock, R.RPM), b.cfg.Retain); err != nil {
		return err
	}
	if err := b.publish("engine/battery", readUSupply(measBlock), b.cfg.Retain); err != nil {
		return err
	}
	if b.lastRunStarted != "" {
		if err := b.publish("engine/last_run_started", b.lastRunStarted, b.cfg.Retain); err != nil {
			return err
		}
	}
	if b.lastRunStopped != "" {
		if err := b.publish("engine/last_run_stopped", b.lastRunStopped, b.cfg.Retain); err != nil {
			return err
		}
	}
	if b.lastRunDuration != nil {
		if err := b.publish("engine/last_run_duration_s", *b.lastRunDuration, b.cfg.Retain); err != nil {
			return err
		}
	}
	if b.lastAlarmText != "" {
		if err := b.publish("alarms/last_text", b.lastAlarmText, b.cfg.Retain); err != nil {
			return err
		}
	}
	if b.lastAlarmSetAt != "" {
		if err := b.publish("alarms/last_set_at", b.lastAlarmSetAt, b.cfg.Retain); err != nil {
			return err
		}
	}
	if b.lastAlarmSum != "" {
		if err := b.publish("alarms/last_set_summary", b.lastAlarmSum, b.cfg.Retain); err != nil {
			return err
		}
	}
	if b.lastAlarmClrAt != "" {
		if err := b.publish("alarms/last_cleared_at", b.lastAlarmClrAt, b.cfg.Retain); err != nil {
			return err
		}
	}
	return b.publish("ts", time.Now().UTC().Format(time.RFC3339Nano), b.cfg.Retain)
}

func (b *bridge) publishStaticMetadata() {
	if err := b.publish("device/model", b.cfg.DeviceModel, true); err != nil {
		log.Printf("metadata publish error: %v", err)
	}
	if err := b.publish("device/manufacturer", b.cfg.DeviceManufacturer, true); err != nil {
		log.Printf("metadata publish error: %v", err)
	}
	if err := b.publish("device/name", b.cfg.DeviceName, true); err != nil {
		log.Printf("metadata publish error: %v", err)
	}
}

func (b *bridge) publish(topicKey string, value any, retain bool) error {
	if b.mqttClient == nil || !b.mqttClient.IsConnected() {
		return fmt.Errorf("mqtt client not connected")
	}
	topic := b.cfg.TopicPrefix + "/" + topicKey
	payload, err := marshalTopicValue(value)
	if err != nil {
		return err
	}
	token := b.mqttClient.Publish(topic, 0, retain, payload)
	if !token.WaitTimeout(mqttPublishTimeout) {
		return fmt.Errorf("publish timeout: %s", topic)
	}
	return token.Error()
}

func (b *bridge) publishFlat(prefix string, value any, retain bool) error {
	return publishFlat(prefix, value, retain, b.publish)
}

func publishFlat(prefix string, value any, retain bool, publisher func(string, any, bool) error) error {
	switch typed := value.(type) {
	case map[string]any:
		keys := make([]string, 0, len(typed))
		for key := range typed {
			keys = append(keys, key)
		}
		sort.Strings(keys)
		for _, key := range keys {
			next := typed[key]
			path := prefix + "/" + key
			if nested, ok := next.(map[string]any); ok {
				if err := publishFlat(path, nested, retain, publisher); err != nil {
					return err
				}
				continue
			}
			if nested, ok := next.(map[string]string); ok {
				nestedAny := make(map[string]any, len(nested))
				for k, v := range nested {
					nestedAny[k] = v
				}
				if err := publishFlat(path, nestedAny, retain, publisher); err != nil {
					return err
				}
				continue
			}
			if err := publisher(path, next, retain); err != nil {
				return err
			}
		}
	case map[string]string:
		nested := make(map[string]any, len(typed))
		for key, val := range typed {
			nested[key] = val
		}
		return publishFlat(prefix, nested, retain, publisher)
	default:
		return publisher(prefix, typed, retain)
	}
	return nil
}

func marshalTopicValue(value any) (string, error) {
	switch typed := value.(type) {
	case string:
		return typed, nil
	case fmt.Stringer:
		return typed.String(), nil
	case []byte:
		return string(typed), nil
	case nil:
		return "", nil
	default:
		if isScalar(value) {
			return fmt.Sprint(value), nil
		}
		raw, err := json.Marshal(value)
		if err != nil {
			return "", err
		}
		return string(raw), nil
	}
}

func (b *bridge) clearRetained(topicKey string) error {
	return b.clearRetainedAbsolute(b.cfg.TopicPrefix + "/" + topicKey)
}

func (b *bridge) clearRetainedAbsolute(topic string) error {
	if b.mqttClient == nil || !b.mqttClient.IsConnected() {
		return fmt.Errorf("mqtt client not connected")
	}
	token := b.mqttClient.Publish(topic, 0, true, "")
	if !token.WaitTimeout(mqttPublishTimeout) {
		return fmt.Errorf("publish timeout: %s", topic)
	}
	return token.Error()
}

func isScalar(value any) bool {
	switch value.(type) {
	case bool, int, int8, int16, int32, int64, uint, uint8, uint16, uint32, uint64, float32, float64:
		return true
	default:
		return false
	}
}

func (b *bridge) publishHassDiscovery() error {
	device := map[string]any{
		"identifiers":  []string{b.cfg.DeviceID},
		"name":         b.cfg.DeviceName,
		"manufacturer": b.cfg.DeviceManufacturer,
		"model":        b.cfg.DeviceModel,
	}

	pubSensor := func(cfg hassSensor) error {
		objectID := b.cfg.NodeID + "-" + cfg.Key
		payload := map[string]any{
			"name":         cfg.Name,
			"uniq_id":      objectID,
			"obj_id":       objectID,
			"stat_t":       b.dataTopicFromPath(cfg.JSONPath),
			"val_tpl":      "{{ value }}",
			"en":           true,
			"force_update": true,
			"device":       device,
		}
		if cfg.Icon != "" {
			payload["ic"] = cfg.Icon
		}
		if cfg.DeviceClass != "" {
			payload["dev_cla"] = cfg.DeviceClass
		}
		if cfg.StateClass != "" {
			payload["stat_cla"] = cfg.StateClass
		}
		if cfg.Unit != "" {
			payload["unit_of_meas"] = cfg.Unit
		}
		if cfg.EntityCategory != "" {
			payload["ent_cat"] = cfg.EntityCategory
		}
		return b.publishDiscovery("sensor", cfg.Key, payload)
	}

	pubBinarySensor := func(cfg hassBinarySensor) error {
		objectID := b.cfg.NodeID + "-" + cfg.Key
		valTpl := cfg.ValueTemplateFlat
		if valTpl == "" {
			valTpl = `{{ "ON" if value|string|lower in ["true","on","1"] else "OFF" }}`
		}
		payload := map[string]any{
			"name":    cfg.Name,
			"uniq_id": objectID,
			"obj_id":  objectID,
			"stat_t":  b.dataTopicFromPath(cfg.JSONPath),
			"val_tpl": valTpl,
			"en":      true,
			"device":  device,
		}
		if cfg.Icon != "" {
			payload["ic"] = cfg.Icon
		}
		if cfg.DeviceClass != "" {
			payload["dev_cla"] = cfg.DeviceClass
		}
		if cfg.EntityCategory != "" {
			payload["ent_cat"] = cfg.EntityCategory
		}
		return b.publishDiscovery("binary_sensor", cfg.Key, payload)
	}

	pubButton := func(cmd commandDef) error {
		objectID := b.cfg.NodeID + "-cmd_" + cmd.Key
		payload := map[string]any{
			"name":    cmd.Name,
			"uniq_id": objectID,
			"obj_id":  objectID,
			"cmd_t":   cmd.Topic,
			"pl_prs":  "1",
			"en":      true,
			"device":  device,
			"ic":      cmd.Icon,
		}
		return b.publishDiscovery("button", "cmd_"+cmd.Key, payload)
	}

	sensors := []hassSensor{
		{Key: "app_version", Name: "App Version", JSONPath: "device.app_version", EntityCategory: "diagnostic", Icon: "mdi:information"},
		{Key: "device_model", Name: "Device Model", JSONPath: "device.model", EntityCategory: "diagnostic", Icon: "mdi:chip"},
		{Key: "timestamp", Name: "Last Seen", JSONPath: "ts", DeviceClass: "timestamp", EntityCategory: "diagnostic", Icon: "mdi:clock"},
		{Key: "gen_voltage_l1n", Name: "Generator Voltage L1-N", JSONPath: "gen.voltage_l1n_v", DeviceClass: "voltage", Unit: "V", StateClass: "measurement", Icon: "mdi:sine-wave"},
		{Key: "gen_voltage_l2n", Name: "Generator Voltage L2-N", JSONPath: "gen.voltage_l2n_v", DeviceClass: "voltage", Unit: "V", StateClass: "measurement", Icon: "mdi:sine-wave"},
		{Key: "gen_voltage_l3n", Name: "Generator Voltage L3-N", JSONPath: "gen.voltage_l3n_v", DeviceClass: "voltage", Unit: "V", StateClass: "measurement", Icon: "mdi:sine-wave"},
		{Key: "gen_current_l1", Name: "Generator Current L1", JSONPath: "gen.current_l1_a", DeviceClass: "current", Unit: "A", StateClass: "measurement", Icon: "mdi:current-ac"},
		{Key: "gen_current_l2", Name: "Generator Current L2", JSONPath: "gen.current_l2_a", DeviceClass: "current", Unit: "A", StateClass: "measurement", Icon: "mdi:current-ac"},
		{Key: "gen_current_l3", Name: "Generator Current L3", JSONPath: "gen.current_l3_a", DeviceClass: "current", Unit: "A", StateClass: "measurement", Icon: "mdi:current-ac"},
		{Key: "gen_frequency", Name: "Generator Frequency", JSONPath: "gen.frequency_hz", DeviceClass: "frequency", Unit: "Hz", StateClass: "measurement", Icon: "mdi:waveform"},
		{Key: "gen_pgen", Name: "Generator Active Power", JSONPath: "gen.pgen_kw", DeviceClass: "power", Unit: "kW", StateClass: "measurement", Icon: "mdi:flash"},
		{Key: "gen_qgen", Name: "Generator Reactive Power", JSONPath: "gen.qgen_kvar", Unit: "kVAr", StateClass: "measurement", Icon: "mdi:flash-outline"},
		{Key: "gen_sgen", Name: "Generator Apparent Power", JSONPath: "gen.sgen_kva", Unit: "kVA", StateClass: "measurement", Icon: "mdi:flash-triangle"},
		{Key: "gen_cos_phi", Name: "Generator Power Factor", JSONPath: "gen.cos_phi", StateClass: "measurement", Icon: "mdi:cosine-wave"},
		{Key: "gen_pgen_l1", Name: "Generator Active Power L1", JSONPath: "gen.pgen_l1_kw", DeviceClass: "power", Unit: "kW", StateClass: "measurement", Icon: "mdi:flash"},
		{Key: "gen_pgen_l2", Name: "Generator Active Power L2", JSONPath: "gen.pgen_l2_kw", DeviceClass: "power", Unit: "kW", StateClass: "measurement", Icon: "mdi:flash"},
		{Key: "gen_pgen_l3", Name: "Generator Active Power L3", JSONPath: "gen.pgen_l3_kw", DeviceClass: "power", Unit: "kW", StateClass: "measurement", Icon: "mdi:flash"},
		{Key: "gen_apparent_l1", Name: "Generator Apparent Power L1", JSONPath: "gen.apparent_l1_va", DeviceClass: "apparent_power", Unit: "VA", StateClass: "measurement", Icon: "mdi:flash-triangle"},
		{Key: "gen_apparent_l2", Name: "Generator Apparent Power L2", JSONPath: "gen.apparent_l2_va", DeviceClass: "apparent_power", Unit: "VA", StateClass: "measurement", Icon: "mdi:flash-triangle"},
		{Key: "gen_apparent_l3", Name: "Generator Apparent Power L3", JSONPath: "gen.apparent_l3_va", DeviceClass: "apparent_power", Unit: "VA", StateClass: "measurement", Icon: "mdi:flash-triangle"},
		{Key: "mains_voltage_l1n", Name: "Mains Voltage L1-N", JSONPath: "mains.voltage_l1n_v", DeviceClass: "voltage", Unit: "V", StateClass: "measurement", Icon: "mdi:transmission-tower"},
		{Key: "mains_voltage_l2n", Name: "Mains Voltage L2-N", JSONPath: "mains.voltage_l2n_v", DeviceClass: "voltage", Unit: "V", StateClass: "measurement", Icon: "mdi:transmission-tower"},
		{Key: "mains_voltage_l3n", Name: "Mains Voltage L3-N", JSONPath: "mains.voltage_l3n_v", DeviceClass: "voltage", Unit: "V", StateClass: "measurement", Icon: "mdi:transmission-tower"},
		{Key: "mains_frequency", Name: "Mains Frequency", JSONPath: "mains.frequency_hz", DeviceClass: "frequency", Unit: "Hz", StateClass: "measurement", Icon: "mdi:waveform"},
		{Key: "run_hours", Name: "Generator Run Hours", JSONPath: "counters.run_hours", Unit: "h", StateClass: "total_increasing", Icon: "mdi:timer-outline"},
		{Key: "energy_kwh", Name: "Energy Produced", JSONPath: "counters.energy_kwh", DeviceClass: "energy", Unit: "kWh", StateClass: "total_increasing", Icon: "mdi:lightning-bolt"},
		{Key: "active_alarms_text", Name: "Active Alarms", JSONPath: "alarms.active_text", Icon: "mdi:alarm-light"},
		{Key: "engine_battery", Name: "Battery Voltage", JSONPath: "engine.battery", DeviceClass: "voltage", Unit: "V", StateClass: "measurement", Icon: "mdi:car-battery"},
		{Key: "alarm_count", Name: "Alarms Total", JSONPath: "alarms.count", StateClass: "measurement", EntityCategory: "diagnostic", Icon: "mdi:counter"},
		{Key: "alarm_unacknowledged", Name: "Alarms Unacknowledged", JSONPath: "alarms.unacknowledged", StateClass: "measurement", EntityCategory: "diagnostic", Icon: "mdi:alert-circle"},
		{Key: "alarm_ack_active", Name: "Alarms Acknowledged Active", JSONPath: "alarms.ack_active", StateClass: "measurement", EntityCategory: "diagnostic", Icon: "mdi:alert-circle-check"},
		{Key: "energy_signed_kwh", Name: "Energy Produced (Signed)", JSONPath: "counters.energy_signed_kwh", Unit: "kWh", EntityCategory: "diagnostic", Icon: "mdi:swap-horizontal"},
		{Key: "gen_breaker_ops", Name: "Generator Breaker Operations", JSONPath: "counters.gen_breaker_ops", StateClass: "total_increasing", EntityCategory: "diagnostic", Icon: "mdi:electric-switch"},
		{Key: "mains_breaker_ops", Name: "Mains Breaker Operations", JSONPath: "counters.mains_breaker_ops", StateClass: "total_increasing", EntityCategory: "diagnostic", Icon: "mdi:electric-switch"},
		{Key: "start_attempts", Name: "Start Attempts", JSONPath: "counters.start_attempts", StateClass: "total_increasing", EntityCategory: "diagnostic", Icon: "mdi:restart"},
		{Key: "rpm", Name: "Engine RPM", JSONPath: "engine.rpm", Unit: "RPM", StateClass: "measurement", EntityCategory: "diagnostic", Icon: "mdi:engine"},
		{Key: "last_run_started", Name: "Last Run Started", JSONPath: "engine.last_run_started", DeviceClass: "timestamp", EntityCategory: "diagnostic", Icon: "mdi:clock-start"},
		{Key: "last_run_stopped", Name: "Last Run Stopped", JSONPath: "engine.last_run_stopped", DeviceClass: "timestamp", EntityCategory: "diagnostic", Icon: "mdi:clock-end"},
		{Key: "last_run_duration_s", Name: "Last Run Duration", JSONPath: "engine.last_run_duration_s", Unit: "s", StateClass: "measurement", EntityCategory: "diagnostic", Icon: "mdi:timer-outline"},
		{Key: "last_alarm_text", Name: "Last Alarm", JSONPath: "alarms.last_text", EntityCategory: "diagnostic", Icon: "mdi:alert-decagram"},
		{Key: "operating_mode", Name: "Operating Mode", JSONPath: "status.operating_mode", Icon: "mdi:state-machine"},
	}
	for _, sensor := range sensors {
		if err := pubSensor(sensor); err != nil {
			return err
		}
	}

	if b.cfg.PublishAlarmBitfields {
		for _, reg := range alarmBitfieldRegisters {
			if err := pubSensor(hassSensor{
				Key:            "alarm_bitfield_" + reg,
				Name:           "Alarm Bitfield " + reg,
				JSONPath:       "alarms.bitfield." + reg,
				EntityCategory: "diagnostic",
				Icon:           "mdi:code-brackets",
			}); err != nil {
				return err
			}
		}
	} else {
		for _, reg := range alarmBitfieldRegisters {
			if err := b.clearDiscovery("sensor", "alarm_bitfield_"+reg); err != nil {
				return err
			}
		}
	}

	binarySensors := []hassBinarySensor{
		{Key: "has_unack_alarms", Name: "Unacknowledged Alarms Active", JSONPath: "alarms.unacknowledged", ValueTemplateFlat: `{{ "ON" if (value | int(0)) > 0 else "OFF" }}`, DeviceClass: "problem", Icon: "mdi:alert"},
	}
	statusBinarySensors := []hassBinarySensor{
		{Key: "status_mains_failure", Name: "Mains Failure", JSONPath: "status.1018_0", DeviceClass: "problem", Icon: "mdi:transmission-tower-off"},
		{Key: "status_mb_on", Name: "Mains Breaker ON", JSONPath: "status.1018_1", Icon: "mdi:electric-switch"},
		{Key: "status_gb_on", Name: "Generator Breaker ON", JSONPath: "status.1018_4", Icon: "mdi:electric-switch"},
		{Key: "status_engine_running", Name: "Engine Running", JSONPath: "status.1018_6", DeviceClass: "running", Icon: "mdi:engine"},
		{Key: "status_gen_ok", Name: "Generator Hz/V OK", JSONPath: "status.1018_8", Icon: "mdi:check-circle"},
		{Key: "status_running_timer", Name: "Running Detection Timer Expired", JSONPath: "status.1018_7", EntityCategory: "diagnostic", Icon: "mdi:timer-check"},
		{Key: "mode_off", Name: "Mode: OFF", JSONPath: "status.1019_0", EntityCategory: "diagnostic", Icon: "mdi:power-off"},
		{Key: "mode_manual", Name: "Mode: Manual", JSONPath: "status.1019_1", EntityCategory: "diagnostic", Icon: "mdi:hand-back-right"},
		{Key: "mode_auto", Name: "Mode: Auto", JSONPath: "status.1019_3", EntityCategory: "diagnostic", Icon: "mdi:autorenew"},
		{Key: "mode_test", Name: "Mode: Test", JSONPath: "status.1019_4", EntityCategory: "diagnostic", Icon: "mdi:test-tube"},
		{Key: "mode_island", Name: "Mode: Island", JSONPath: "status.1019_5", EntityCategory: "diagnostic", Icon: "mdi:island"},
		{Key: "mode_amf", Name: "Mode: AMF", JSONPath: "status.1019_6", EntityCategory: "diagnostic", Icon: "mdi:auto-mode"},
		{Key: "load_takeover", Name: "Load Take Over", JSONPath: "status.1019_10", EntityCategory: "diagnostic", Icon: "mdi:transfer"},
		{Key: "amf_active", Name: "AMF Active", JSONPath: "status.1019_15", EntityCategory: "diagnostic", Icon: "mdi:lightning-bolt"},
	}
	if b.cfg.PublishStatusFlags {
		binarySensors = append(binarySensors, statusBinarySensors...)
	} else {
		for _, key := range statusFlagBinarySensors {
			if err := b.clearDiscovery("binary_sensor", key); err != nil {
				return err
			}
		}
	}
	for _, bs := range binarySensors {
		if err := pubBinarySensor(bs); err != nil {
			return err
		}
	}

	for _, cmd := range b.commandDefs {
		if err := pubButton(cmd); err != nil {
			return err
		}
	}
	return nil
}

func (b *bridge) publishDiscovery(entityType, key string, payload map[string]any) error {
	raw, err := json.Marshal(payload)
	if err != nil {
		return err
	}
	topic := fmt.Sprintf("%s/%s/%s/%s/config", b.cfg.DiscoveryPrefix, entityType, b.cfg.NodeID, key)
	token := b.mqttClient.Publish(topic, 0, true, raw)
	if !token.WaitTimeout(mqttPublishTimeout) {
		return fmt.Errorf("discovery publish timeout: %s", topic)
	}
	return token.Error()
}

func (b *bridge) clearDiscovery(entityType, key string) error {
	topic := fmt.Sprintf("%s/%s/%s/%s/config", b.cfg.DiscoveryPrefix, entityType, b.cfg.NodeID, key)
	return b.clearRetainedAbsolute(topic)
}

func (b *bridge) dataTopicFromPath(jsonPath string) string {
	return b.cfg.TopicPrefix + "/" + strings.ReplaceAll(jsonPath, ".", "/")
}

func (b *bridge) clearAlarmBitfieldTopics() error {
	for _, reg := range alarmBitfieldRegisters {
		if err := b.clearRetained("alarms/bitfield/" + reg); err != nil {
			return err
		}
	}
	return nil
}

func (b *bridge) clearStatusFlagTopics() error {
	for key := range statusMap {
		topicKey := "status/" + strings.ReplaceAll(key, ":", "_")
		if err := b.clearRetained(topicKey); err != nil {
			return err
		}
	}
	return nil
}

func getReg(block []uint16, addr uint16) uint16 {
	idx := int(addr - measStart)
	if idx < 0 || idx >= len(block) {
		return 0
	}
	return block[idx]
}

func readGen(block []uint16, cfg config) map[string]any {
	u1 := getReg(block, R.GenUL1N)
	u2 := getReg(block, R.GenUL2N)
	u3 := getReg(block, R.GenUL3N)
	i1 := getReg(block, R.GenIL1)
	i2 := getReg(block, R.GenIL2)
	i3 := getReg(block, R.GenIL3)
	return map[string]any{
		"voltage_l1n_v":  u1,
		"voltage_l2n_v":  u2,
		"voltage_l3n_v":  u3,
		"current_l1_a":   i1,
		"current_l2_a":   i2,
		"current_l3_a":   i3,
		"frequency_hz":   freqFloat(getReg(block, R.GenF), cfg),
		"pgen_kw":        s16(getReg(block, R.PGen)),
		"qgen_kvar":      s16(getReg(block, R.QGen)),
		"sgen_kva":       s16(getReg(block, R.SGen)),
		"cos_phi":        roundFloat(float64(s16(getReg(block, R.CosPhi)))/100.0, 2),
		"pgen_l1_kw":     s16(getReg(block, R.GenPL1)),
		"pgen_l2_kw":     s16(getReg(block, R.GenPL2)),
		"pgen_l3_kw":     s16(getReg(block, R.GenPL3)),
		"apparent_l1_va": uint32(u1) * uint32(i1),
		"apparent_l2_va": uint32(u2) * uint32(i2),
		"apparent_l3_va": uint32(u3) * uint32(i3),
	}
}

func readMains(block []uint16, cfg config) map[string]any {
	return map[string]any{
		"voltage_l1n_v": getReg(block, R.MainsUL1N),
		"voltage_l2n_v": getReg(block, R.MainsUL2N),
		"voltage_l3n_v": getReg(block, R.MainsUL3N),
		"frequency_hz":  freqFloat(getReg(block, R.MainsF), cfg),
	}
}

func readRunHours(block []uint16) uint32 {
	return u32(getReg(block, R.RunHi), getReg(block, R.RunLo))
}

func readEnergy(block []uint16) (uint32, int64) {
	value := u32(getReg(block, R.EGenHi), getReg(block, R.EGenLo))
	return value, -int64(value)
}

func readUSupply(block []uint16) float64 {
	return roundFloat(float64(getReg(block, R.USupply))/10.0, 1)
}

func decodeAlarms(alarmRegs []uint16) []activeAlarm {
	active := make([]activeAlarm, 0)
	for idx, value := range alarmRegs {
		regAddr := uint16(alarmStart + idx)
		if _, ok := alarmRegisterSet[regAddr]; !ok {
			continue
		}
		for bit := 0; bit < 16; bit++ {
			if value&(1<<bit) == 0 {
				continue
			}
			key := fmt.Sprintf("%d:%d", regAddr, bit)
			def, ok := alarmMap[key]
			if !ok {
				continue
			}
			active = append(active, activeAlarm{
				Register: regAddr,
				Bit:      uint8(bit),
				Code:     def.Code,
				Text:     def.Text,
			})
		}
	}
	return active
}

func decodeStatus(alarmRegs []uint16) map[string]any {
	status := make(map[string]any)
	for _, regAddr := range statusRegisters {
		regIdx := int(regAddr) - alarmStart
		if regIdx < 0 || regIdx >= len(alarmRegs) {
			continue
		}
		value := alarmRegs[regIdx]
		for bit := 0; bit < 16; bit++ {
			key := fmt.Sprintf("%d:%d", regAddr, bit)
			if _, ok := statusMap[key]; !ok {
				continue
			}
			status[strings.ReplaceAll(key, ":", "_")] = value&(1<<bit) != 0
		}
	}
	return status
}

func formatActiveAlarms(active []activeAlarm) string {
	if len(active) == 0 {
		return "No active alarms"
	}
	lines := make([]string, 0, len(active))
	for _, alarm := range active {
		lines = append(lines, alarm.Code+" "+alarm.Text)
	}
	return strings.Join(lines, "\n")
}

func operatingModeText(status map[string]any) string {
	primary := "Unknown"
	if status["1019_0"] == true {
		primary = "OFF"
	} else if status["1019_1"] == true {
		primary = "Manual"
	} else if status["1019_4"] == true {
		primary = "Test"
	} else if status["1019_3"] == true {
		primary = "Auto"
	}

	modifiers := make([]string, 0, 3)
	if status["1019_6"] == true {
		modifiers = append(modifiers, "AMF")
	}
	if status["1019_10"] == true {
		modifiers = append(modifiers, "Load Takeover")
	}
	if status["1019_15"] == true {
		modifiers = append(modifiers, "AMF Active")
	}
	if len(modifiers) == 0 {
		return primary
	}
	return primary + " (" + strings.Join(modifiers, ", ") + ")"
}

func describeAlarmKey(key string) string {
	def, ok := alarmMap[key]
	if !ok {
		return key
	}
	parts := strings.SplitN(key, ":", 2)
	return fmt.Sprintf("%s:%s (%s %s)", parts[0], parts[1], def.Code, def.Text)
}

func formatAppVersion(raw uint16) string {
	value := fmt.Sprintf("%04d", raw)
	return fmt.Sprintf("%s.%s.%s", value[:1], value[1:2], value[2:])
}

func freqFloat(raw uint16, cfg config) float64 {
	return roundFloat(float64(raw)/cfg.FreqDivisor, cfg.FreqDecimals)
}

func roundFloat(value float64, decimals int) float64 {
	if decimals < 0 {
		return value
	}
	scale := 1.0
	for i := 0; i < decimals; i++ {
		scale *= 10
	}
	if value >= 0 {
		return float64(int64(value*scale+0.5)) / scale
	}
	return float64(int64(value*scale-0.5)) / scale
}

func u32(hi, lo uint16) uint32 {
	return uint32(hi)<<16 | uint32(lo)
}

func s16(value uint16) int16 {
	return int16(value)
}

func toHex(value uint16) string {
	return fmt.Sprintf("0x%04X", value)
}

func registerSet(defs map[string]alarmDefinition) map[uint16]struct{} {
	out := make(map[uint16]struct{})
	for key := range defs {
		reg, err := parseRegister(key)
		if err == nil {
			out[reg] = struct{}{}
		}
	}
	return out
}

func registerList(defs map[string]alarmDefinition) []uint16 {
	set := registerSet(defs)
	values := make([]uint16, 0, len(set))
	for reg := range set {
		values = append(values, reg)
	}
	sort.Slice(values, func(i, j int) bool { return values[i] < values[j] })
	return values
}

func parseRegister(key string) (uint16, error) {
	parts := strings.SplitN(key, ":", 2)
	if len(parts) != 2 {
		return 0, fmt.Errorf("invalid alarm key %q", key)
	}
	value, err := strconv.ParseUint(parts[0], 10, 16)
	if err != nil {
		return 0, err
	}
	return uint16(value), nil
}

func boolPtr(value bool) *bool {
	v := value
	return &v
}
