/*
 * Baja SAE Car Computer - ESP32-S3
 * ESP32 Arduino Core: 2.x (must match dashboard)
 *
 * Hardware:
 *   - 2x TI-DRV5013 RPM Hall-Effect Sensors  (GPIO 4, 5)
 *   - 1x DS18B20 Temperature Sensor           (GPIO 6, OneWire bus)
 *   - NEO-9M GPS Module                       (UART1: RX=18, TX=17)
 *   - INA219 Current/Voltage Sensor           (I2C: SDA=8, SCL=9)
 *   - RYLR998 LoRa Module                     (UART2: RX=16, TX=15)
 *   - SD Card (SPI)                           (CS=10, MOSI=11, MISO=13, CLK=12)
 *   - 4WD Relay Output                        (GPIO 3)
 *   - DAQ Mode Switch                         (GPIO 7, pull HIGH to enable DAQ)
 *
 * ---- SAMPLE RATES ----
 *   Normal:  GPS 10Hz, RPM 10Hz, INA219 10Hz, Temp 1Hz
 *   DAQ:     GPS 20Hz, RPM 20Hz, INA219 20Hz, Temp 1Hz
 *   ESP-NOW to dash: always 10Hz
 *   LoRa to pit:     always 1Hz
 *
 * ---- DAQ MODE ACTIVATION ----
 *   GPIO 7 switch HIGH  → forces DAQ on regardless of LoRa/dashboard commands
 *   GPIO 7 switch LOW   → DAQ controlled normally by LoRa/dashboard commands
 *
 * ---- GPS UART ----
 *   Starts at 9600, switched to 115200 on boot.
 *   GLL, GSV, VTG sentences disabled.
 *   Rate: 10Hz normal, 20Hz during DAQ.
 *
 * ---- LORA PARAMETERS ----
 *   SF10, BW125, CR4/5, 22dBm  (AT+PARAMETER=10,7,1,22)
 *   Pit receiver: AT+NETWORKID=5, AT+ADDRESS=2, AT+PARAMETER=10,7,1,22
 */

// ============================================================
//  INCLUDES
// ============================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_INA219.h>

// ============================================================
//  PIN DEFINITIONS
// ============================================================
#define RPM1_PIN       4    // DRV5013 engine RPM
#define RPM2_PIN       5    // DRV5013 transmission RPM
#define ONE_WIRE_PIN   6    // DS18B20 data
#define DAQ_SW_PIN     7    // DAQ mode switch (pull HIGH to enable)
#define RELAY_4WD_PIN  21    // 4WD relay output

#define GPS_RX_PIN    18
#define GPS_TX_PIN    17

#define LORA_RX_PIN   16
#define LORA_TX_PIN   15

#define SD_CS_PIN     10
#define SD_MOSI_PIN   11
#define SD_MISO_PIN   13
#define SD_CLK_PIN    12

#define I2C_SDA_PIN    8
#define I2C_SCL_PIN    9

// ============================================================
//  SAMPLE RATE PERIODS (ms)
// ============================================================
#define RPM_PERIOD_NORMAL_MS     100   // 10Hz
#define RPM_PERIOD_DAQ_MS         50   // 20Hz

#define TEMP_PERIOD_MS          1000   // 1Hz always
#define TEMP_CONV_WAIT_MS        100   // DS18B20 9-bit conversion time

#define INA_PERIOD_NORMAL_MS     100   // 10Hz
#define INA_PERIOD_DAQ_MS         50   // 20Hz

#define DAQ_WRITE_PERIOD_MS       50   // 20Hz SD writes

#define ESPNOW_PERIOD_MS         100   // 10Hz to dash
#define LORA_TX_PERIOD_MS       1000   //  1Hz to pit

// ============================================================
//  MAC ADDRESSES
// ============================================================
uint8_t dashboardMac[6] = {0x4C, 0xC3, 0x82, 0xC3, 0xBB, 0xA0};

// ============================================================
//  DATA STRUCTURES
// ============================================================
// No #pragma pack on TelemetryData — must match dashboard natural alignment
typedef struct {
  int     rpm;
  int     speed;
  float   voltage;
  int     satCount;
  char    driveMode[4];
  bool    wheelLF;
  bool    wheelRF;
  bool    wheelRL;
  bool    wheelRR;
  bool    diffFront;
  bool    diffRear;
  bool    wifiConn;
  bool    loraConn;
  int     odometer;
  int     hourMeter;
  bool    warningActive;
  char    warningMsg[32];
} TelemetryData;

#pragma pack(push, 1)

typedef struct {
  bool swState;
  bool relayOn;
  bool daqMode;
  bool pitSignal;
} DashboardCommand;

typedef struct {
  uint16_t r1[10];
  uint16_t r2[10];
  uint16_t v[10];
  uint8_t  t1;
  uint8_t  t2;
  uint16_t bat;
  uint8_t  pct;
  int32_t  lat1;
  int32_t  lon1;
  int32_t  lat2;
  int32_t  lon2;
  uint8_t  awd;
  uint16_t crc16;
} LoRaPacket;

#pragma pack(pop)

// ============================================================
//  GLOBALS
// ============================================================

HardwareSerial    gpsSerial(1);
HardwareSerial    loraSerial(2);
TinyGPSPlus       gps;
OneWire           oneWire(ONE_WIRE_PIN);
DallasTemperature tempSensors(&oneWire);
Adafruit_INA219   ina219;

// RPM
volatile uint32_t rpm1PulseCount = 0;
volatile uint32_t rpm2PulseCount = 0;
uint32_t lastRpmCalcMs = 0;
float    currentRpm1   = 0;
float    currentRpm2   = 0;

// Temperature (non-blocking, one sensor, temp2 always 0)
float    temp1C            = 0;
float    temp2C            = 0;   // always 0, no second sensor
bool     tempConvPending   = false;
uint32_t tempConvStartMs   = 0;
uint32_t lastTempRequestMs = 0;

// INA219
float    batteryVoltage = 12.6f;
float    batteryCurrent = 0;
float    batteryPercent = 100;
uint32_t lastInaMs      = 0;

// GPS
double   gpsLat      = 0;
double   gpsLon      = 0;
double   gpsPrevLat  = 0;
double   gpsPrevLon  = 0;
float    gpsSpeedMph = 0;
int      gpsSats     = 0;
bool     gpsNewFix   = false;

// Vehicle
bool     fourWDActive = false;

// Hour meter & odometer
uint32_t hourMeterSeconds = 0;
uint32_t odometerFeet     = 0;
uint32_t lastHourOdomMs   = 0;

// DAQ
// daqMode = true when either the switch is HIGH or commanded by LoRa/dashboard.
// daqSwitchActive tracks the physical switch state separately so we don't
// stop DAQ via LoRa/dashboard while the switch is still held on.
bool     daqMode          = false;
bool     daqSwitchActive  = false;
uint32_t lastDaqWriteMs   = 0;
File     daqFile;
bool     daqFileOpen      = false;

// ESP-NOW
bool     dashConnected  = false;
bool     loraConnected  = false;
uint32_t lastDashRxMs   = 0;
uint32_t lastEspNowTxMs = 0;
uint32_t lastLoraTxMs   = 0;

// LoRa 10Hz ring buffer
#define LORA_SAMPLES 10
uint16_t loraBufR1[LORA_SAMPLES] = {};
uint16_t loraBufR2[LORA_SAMPLES] = {};
uint16_t loraBufV[LORA_SAMPLES]  = {};
int32_t  loraLat1         = 0;
int32_t  loraLon1         = 0;
uint8_t  loraSampleIdx    = 0;
uint32_t lastLoraSampleMs = 0;

// Pit signal
bool     pitSignal      = false;
uint32_t pitSignalSetMs = 0;

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
void startDAQ();
void stopDAQ();
void gpsSetRate(int hz);

// ============================================================
//  ISR
// ============================================================
void IRAM_ATTR rpm1ISR() { rpm1PulseCount++; }
void IRAM_ATTR rpm2ISR() { rpm2PulseCount++; }

// ============================================================
//  CRC16 MODBUS
// ============================================================
uint16_t crc16_modbus(uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  return crc;
}

// ============================================================
//  GPS UBX HELPERS
// ============================================================
void gpsSendUBX(uint8_t *msg, uint8_t len) {
  uint8_t ckA = 0, ckB = 0;
  for (int i = 2; i < len - 2; i++) { ckA += msg[i]; ckB += ckA; }
  msg[len - 2] = ckA;
  msg[len - 1] = ckB;
  gpsSerial.write(msg, len);
  gpsSerial.flush();
  delay(20);
}

void gpsSetBaud(uint32_t baud) {
  uint8_t msg[28] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
    0x01, 0x00, 0x00, 0x00,
    0xD0, 0x08, 0x00, 0x00,
    (uint8_t)(baud & 0xFF), (uint8_t)(baud >> 8),
    (uint8_t)(baud >> 16),  (uint8_t)(baud >> 24),
    0x07, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00
  };
  gpsSendUBX(msg, 28);
  delay(200);
  gpsSerial.updateBaudRate(baud);
  delay(100);
  Serial.printf("[GPS] Baud -> %lu\n", baud);
}

void gpsSetRate(int hz) {
  uint16_t measMs = 1000 / hz;
  uint8_t msg[14] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    (uint8_t)(measMs & 0xFF), (uint8_t)(measMs >> 8),
    0x01, 0x00, 0x01, 0x00,
    0x00, 0x00
  };
  gpsSendUBX(msg, 14);
  Serial.printf("[GPS] Rate -> %dHz\n", hz);
}

void gpsSetNMEA(uint8_t msgId, bool enable) {
  uint8_t msg[16] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0xF0, msgId, 0x00,
    enable ? (uint8_t)1 : (uint8_t)0,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00
  };
  gpsSendUBX(msg, 16);
}

void gpsInit() {
  gpsSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(1000);
  gpsSetNMEA(0x01, false); // GLL off
  gpsSetNMEA(0x03, false); // GSV off
  gpsSetNMEA(0x05, false); // VTG off
  gpsSetRate(10);
  Serial.println("[GPS] Ready: 115200 baud, 10Hz, lean NMEA");
}

// ============================================================
//  ESP-NOW CALLBACKS  (core 2.x signatures)
// ============================================================
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) dashConnected = true;
}

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(DashboardCommand)) return;
  DashboardCommand cmd;
  memcpy(&cmd, data, len);

  digitalWrite(RELAY_4WD_PIN, cmd.relayOn ? HIGH : LOW);
  fourWDActive = cmd.swState;

  // Only honour DAQ commands from dashboard/LoRa when the physical
  // switch is NOT overriding — if the switch is on, DAQ stays on.
  if (!daqSwitchActive) {
    if      ( cmd.daqMode && !daqMode) startDAQ();
    else if (!cmd.daqMode &&  daqMode) stopDAQ();
  }

  if (cmd.pitSignal) { pitSignal = true; pitSignalSetMs = millis(); }

  lastDashRxMs  = millis();
  dashConnected = true;
}

// ============================================================
//  DAQ
// ============================================================
void startDAQ() {
  if (daqMode) return;  // already running
  Serial.println("[DAQ] Starting — GPS 20Hz, RPM/INA 20Hz");
  daqMode = true;
  gpsSetRate(20);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("[DAQ] SD not ready — fast rates active, no file logging");
    return;
  }
  char fname[32];
  int idx = 0;
  do { snprintf(fname, sizeof(fname), "/DAQ_%04d.csv", idx++); }
  while (SD.exists(fname) && idx < 9999);

  daqFile = SD.open(fname, FILE_WRITE);
  if (!daqFile) { Serial.println("[DAQ] File open failed"); return; }

  daqFile.println("timestamp_iso,lat,lon,speed_mph,gps_new,rpm1,rpm2,temp1_c,voltage_v,current_a");
  daqFileOpen = true;
  Serial.printf("[DAQ] Logging -> %s @ 20Hz\n", fname);
}

void stopDAQ() {
  // Do not stop if the physical switch is holding DAQ on
  if (daqSwitchActive) return;
  if (!daqMode) return;

  Serial.println("[DAQ] Stopping — GPS 10Hz, RPM/INA 10Hz");
  daqMode = false;
  gpsSetRate(10);

  if (daqFileOpen) {
    daqFile.flush();
    daqFile.close();
    daqFileOpen = false;
    Serial.println("[DAQ] File closed");
  }
}

void writeDAQRow() {
  if (!daqFileOpen) return;
  char ts[24] = "NOGPS";
  if (gps.date.isValid() && gps.time.isValid()) {
    snprintf(ts, sizeof(ts), "%04d-%02d-%02dT%02d:%02d:%02d",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
  }
  daqFile.printf("%s,%.7f,%.7f,%.2f,%d,%.0f,%.0f,%.2f,%.3f,%.3f\n",
    ts, gpsLat, gpsLon, gpsSpeedMph, gpsNewFix ? 1 : 0,
    currentRpm1, currentRpm2,
    temp1C,
    batteryVoltage, batteryCurrent);
}

// ============================================================
//  DAQ SWITCH POLL
//  Called every loop. If switch goes HIGH, force DAQ on.
//  If switch goes LOW, allow normal DAQ stop logic.
// ============================================================
void checkDaqSwitch() {
  bool swHigh = digitalRead(DAQ_SW_PIN) == HIGH;

  if (swHigh && !daqSwitchActive) {
    // Switch just turned on
    daqSwitchActive = true;
    Serial.println("[DAQ] Switch ON — forcing DAQ active");
    if (!daqMode) startDAQ();
  } else if (!swHigh && daqSwitchActive) {
    // Switch just turned off
    daqSwitchActive = false;
    Serial.println("[DAQ] Switch OFF — DAQ control returned to LoRa/dashboard");
    // Stop DAQ unless dashboard/LoRa has independently requested it
    // Since we have no persistent "remote requested" flag, we stop it
    // and let the remote re-enable if it wants to
    stopDAQ();
  }
}

// ============================================================
//  LORA
// ============================================================
void loraInit() {
  loraSerial.begin(115200, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  delay(500);
  loraSerial.println("AT+NETWORKID=5");          delay(200);
  loraSerial.println("AT+ADDRESS=1");            delay(200);
  loraSerial.println("AT+PARAMETER=10,7,1,22");  delay(200);
  while (loraSerial.available()) Serial.println(loraSerial.readStringUntil('\n'));
  loraConnected = true;
  Serial.println("[LoRa] Ready: SF10, BW125, 22dBm");
}

void loraSendPacket() {
  LoRaPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  for (int i = 0; i < LORA_SAMPLES; i++) {
    pkt.r1[i] = loraBufR1[i];
    pkt.r2[i] = loraBufR2[i];
    pkt.v[i]  = loraBufV[i];
  }
  pkt.t1   = (uint8_t)constrain((int)temp1C + 40, 0, 255);
  pkt.t2   = 40;  // 0°C offset = no sensor, sends 40 (0°C) as placeholder
  pkt.bat  = (uint16_t)(batteryVoltage * 100);
  pkt.pct  = (uint8_t)constrain((int)batteryPercent, 0, 100);
  pkt.lat1 = loraLat1;
  pkt.lon1 = loraLon1;
  pkt.lat2 = (int32_t)(gpsLat * 10000000.0);
  pkt.lon2 = (int32_t)(gpsLon * 10000000.0);
  pkt.awd  = fourWDActive ? 1 : 0;
  pkt.crc16 = crc16_modbus((uint8_t*)&pkt, 78);
  loraSerial.print("AT+SEND=2,80,");
  loraSerial.write((uint8_t*)&pkt, 80);
  loraSerial.print("\r\n");
}

void loraHandleIncoming() {
  if (!loraSerial.available()) return;
  String line = loraSerial.readStringUntil('\n');
  if (!line.startsWith("+RCV=")) return;
  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  if (c1 < 0 || c2 < 0) return;
  if (line.substring(c1 + 1, c2).toInt() >= 1) {
    uint8_t cmd = (uint8_t)line[c2 + 1];
    // LoRa commands only take effect when physical switch is off
    if (!daqSwitchActive) {
      if      (cmd == 0x01 && !daqMode) startDAQ();
      else if (cmd == 0x00 &&  daqMode) stopDAQ();
    }
    if (cmd == 0x02) { pitSignal = true; pitSignalSetMs = millis(); }
    loraConnected = true;
  }
}

// ============================================================
//  SENSOR READS
// ============================================================

void calcRPM() {
  uint32_t now     = millis();
  uint32_t period  = daqMode ? RPM_PERIOD_DAQ_MS : RPM_PERIOD_NORMAL_MS;
  uint32_t elapsed = now - lastRpmCalcMs;
  if (elapsed < period) return;

  noInterrupts();
  uint32_t c1 = rpm1PulseCount; rpm1PulseCount = 0;
  uint32_t c2 = rpm2PulseCount; rpm2PulseCount = 0;
  interrupts();

  float sec   = elapsed / 1000.0f;
  currentRpm1 = (c1 / sec) * 60.0f;
  currentRpm2 = (c2 / sec) * 60.0f;
  lastRpmCalcMs = now;
}

// Non-blocking DS18B20 — single sensor, temp2 stays 0.
void updateTemperatures() {
  uint32_t now = millis();
  if (!tempConvPending) {
    if (now - lastTempRequestMs >= TEMP_PERIOD_MS) {
      tempSensors.requestTemperatures();
      tempConvPending   = true;
      tempConvStartMs   = now;
      lastTempRequestMs = now;
    }
  } else {
    if (now - tempConvStartMs >= TEMP_CONV_WAIT_MS) {
      float t1 = tempSensors.getTempCByIndex(0);
      if (t1 != DEVICE_DISCONNECTED_C) temp1C = t1;
      // temp2C intentionally left as 0 — no second sensor
      tempConvPending = false;
    }
  }
}

void readINA219() {
  uint32_t now    = millis();
  uint32_t period = daqMode ? INA_PERIOD_DAQ_MS : INA_PERIOD_NORMAL_MS;
  if (now - lastInaMs < period) return;
  batteryVoltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000.0f);
  batteryCurrent = ina219.getCurrent_mA() / 1000.0f;
  batteryPercent = constrain((batteryVoltage - 10.5f) / (13.0f - 10.5f) * 100.0f, 0.0f, 100.0f);
  lastInaMs      = now;
}

void updateGPS() {
  gpsNewFix = false;
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  if (gps.location.isUpdated()) {
    gpsPrevLat  = gpsLat;
    gpsPrevLon  = gpsLon;
    loraLat1    = (int32_t)(gpsPrevLat * 10000000.0);
    loraLon1    = (int32_t)(gpsPrevLon * 10000000.0);
    gpsLat      = gps.location.lat();
    gpsLon      = gps.location.lng();
    gpsSpeedMph = gps.speed.mph();
    gpsSats     = gps.satellites.value();
    gpsNewFix   = true;
  }
}

// ============================================================
//  HOUR METER & ODOMETER
// ============================================================
void updateHourOdometer() {
  uint32_t now     = millis();
  uint32_t elapsed = now - lastHourOdomMs;
  if (elapsed < 1000) return;
  if (currentRpm1 > 800) {
    hourMeterSeconds += elapsed / 1000;
    odometerFeet     += (uint32_t)(gpsSpeedMph * (elapsed / 3600000.0f) * 5280.0f);
  }
  lastHourOdomMs = now;
}

// ============================================================
//  LORA 10Hz RING BUFFER
// ============================================================
void pushLoraSample() {
  uint32_t now = millis();
  if (now - lastLoraSampleMs < 100) return;
  lastLoraSampleMs = now;
  int idx = loraSampleIdx++ % LORA_SAMPLES;
  loraBufR1[idx] = (uint16_t)constrain((int)currentRpm1, 0, 65535);
  loraBufR2[idx] = (uint16_t)constrain((int)currentRpm2, 0, 65535);
  loraBufV[idx]  = (uint16_t)(gpsSpeedMph * 10.0f);
}

// ============================================================
//  ESP-NOW TX TO DASHBOARD
// ============================================================
void sendToDashboard() {
  TelemetryData t;
  memset(&t, 0, sizeof(t));
  t.rpm           = (int)currentRpm1;
  t.speed         = (int)gpsSpeedMph;
  t.voltage       = batteryVoltage;
  t.satCount      = gpsSats;
  strlcpy(t.driveMode, fourWDActive ? "4WD" : "2WD", 4);
  t.wheelLF       = true;
  t.wheelRF       = true;
  t.wheelRL       = true;
  t.wheelRR       = true;
  t.diffFront     = false;
  t.diffRear      = false;
  t.wifiConn      = dashConnected;
  t.loraConn      = loraConnected;
  t.odometer      = (int)(odometerFeet / 5280);
  t.hourMeter     = (int)(hourMeterSeconds / 3600);
  t.warningActive = pitSignal;
  strlcpy(t.warningMsg, pitSignal ? "Return To Pit" : "", 32);
  esp_now_send(dashboardMac, (uint8_t*)&t, sizeof(TelemetryData));
}

// ============================================================
//  SERIAL COMMANDS
// ============================================================
void handleSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (!line.length()) return;

  if (line == "help") {
    Serial.println("=== Baja Car Computer Commands ===");
    Serial.println("  daq=1/0     start/stop DAQ (ignored if switch is ON)");
    Serial.println("  4wd=1/0     set 4WD relay");
    Serial.println("  pit=1/0     set/clear pit warning");
    Serial.println("  status      print all current values");
    return;
  }

  if (line == "status") {
    Serial.println("=== STATUS ===");
    Serial.printf("DAQ:     %s  Switch: %s\n",
                  daqMode ? "ON" : "OFF",
                  daqSwitchActive ? "ON (forcing DAQ)" : "OFF");
    Serial.printf("RPM1:    %.0f  RPM2: %.0f\n", currentRpm1, currentRpm2);
    Serial.printf("Temp1:   %.1fC\n", temp1C);
    Serial.printf("Voltage: %.2fV  Current: %.2fA  Pct: %.0f%%\n",
                  batteryVoltage, batteryCurrent, batteryPercent);
    Serial.printf("Speed:   %.1f mph\n", gpsSpeedMph);
    Serial.printf("GPS:     %.6f, %.6f  Sats: %d\n", gpsLat, gpsLon, gpsSats);
    Serial.printf("Hour:    %lu h  ODO: %lu mi\n",
                  hourMeterSeconds / 3600, odometerFeet / 5280);
    Serial.printf("4WD:     %s  Pit: %s\n",
                  fourWDActive ? "ON" : "OFF",
                  pitSignal ? "ACTIVE" : "clear");
    Serial.printf("Dash:    %s  LoRa: %s\n",
                  dashConnected ? "connected" : "no signal",
                  loraConnected ? "connected" : "no signal");
    Serial.printf("TelemetryData: %d bytes\n", sizeof(TelemetryData));
    return;
  }

  int eq = line.indexOf('=');
  if (eq < 0) { Serial.println("Unknown command. Try 'help'"); return; }

  String key = line.substring(0, eq); key.toLowerCase();
  String val = line.substring(eq + 1);

  if (key == "daq") {
    if (daqSwitchActive) {
      Serial.println("[DAQ] Ignored — physical switch is ON");
    } else {
      if      (val.toInt() && !daqMode) startDAQ();
      else if (!val.toInt() && daqMode) stopDAQ();
    }
  }
  else if (key == "4wd") {
    fourWDActive = val.toInt() != 0;
    digitalWrite(RELAY_4WD_PIN, fourWDActive ? HIGH : LOW);
    Serial.printf("4WD relay: %s\n", fourWDActive ? "ON" : "OFF");
  }
  else if (key == "pit") {
    pitSignal = val.toInt() != 0;
    if (pitSignal) pitSignalSetMs = millis();
    Serial.printf("Pit signal: %s\n", pitSignal ? "ACTIVE" : "clear");
  }
  else { Serial.printf("Unknown key: '%s' — try 'help'\n", key.c_str()); }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n===== Baja SAE Car Computer =====");
  Serial.println("ESP32 core 2.x");
  Serial.printf("TelemetryData: %d bytes\n", sizeof(TelemetryData));
  Serial.println("Type 'help' for commands");

  // GPIO
  pinMode(RPM1_PIN,      INPUT_PULLUP);
  pinMode(RPM2_PIN,      INPUT_PULLUP);
  pinMode(RELAY_4WD_PIN, OUTPUT);
  pinMode(DAQ_SW_PIN,    INPUT);        // external pulldown — HIGH = DAQ on
  digitalWrite(RELAY_4WD_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(RPM1_PIN), rpm1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RPM2_PIN), rpm2ISR, FALLING);
  Serial.println("[RPM] Interrupts attached");

  // I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // INA219
  if (!ina219.begin()) Serial.println("[WARN] INA219 not found — check wiring");
  else                 Serial.println("[INA219] Ready");

  // DS18B20
  tempSensors.begin();
  tempSensors.setResolution(9);
  tempSensors.setWaitForConversion(false);
  Serial.printf("[TEMP] %d DS18B20(s) found\n", tempSensors.getDeviceCount());

  // GPS
  gpsInit();

  // LoRa
  loraInit();

  // SD card
  SPI.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN)) Serial.println("[WARN] SD card not found");
  else                       Serial.println("[SD] Ready");

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.print("[MAC] Car: "); Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init FAILED");
  } else {
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, dashboardMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) Serial.println("[ESP-NOW] Dashboard peer added");
    else                                   Serial.println("[ESP-NOW] Failed to add peer");
  }

  // Init timers
  uint32_t now      = millis();
  lastRpmCalcMs     = now;
  lastHourOdomMs    = now;
  lastLoraSampleMs  = now;
  lastTempRequestMs = now;
  lastInaMs         = now;
  lastEspNowTxMs    = now;
  lastLoraTxMs      = now;

  // Check if DAQ switch is already on at boot
  if (digitalRead(DAQ_SW_PIN) == HIGH) {
    Serial.println("[DAQ] Switch ON at boot — starting DAQ immediately");
    daqSwitchActive = true;
    startDAQ();
  }

  Serial.println("=================================\n");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  uint32_t now = millis();

  handleSerialCommands();
  checkDaqSwitch();

  updateGPS();
  calcRPM();
  updateTemperatures();
  readINA219();

  updateHourOdometer();
  pushLoraSample();

  // ESP-NOW to dashboard: 10Hz
  if (now - lastEspNowTxMs >= ESPNOW_PERIOD_MS) {
    sendToDashboard();
    lastEspNowTxMs = now;
    if (now - lastDashRxMs > 3000) dashConnected = false;
  }

  // LoRa to pit: 1Hz
  if (now - lastLoraTxMs >= LORA_TX_PERIOD_MS) {
    loraSendPacket();
    lastLoraTxMs = now;
  }

  // LoRa incoming: check every loop
  loraHandleIncoming();

  // DAQ write: 20Hz
  if (daqMode && (now - lastDaqWriteMs >= DAQ_WRITE_PERIOD_MS)) {
    writeDAQRow();
    lastDaqWriteMs = now;
    // Flush to SD every second
    static uint32_t lastFlushMs = 0;
    if (now - lastFlushMs >= 1000) {
      if (daqFileOpen) daqFile.flush();
      lastFlushMs = now;
    }
  }

  // gpsNewFix valid for one loop iteration only
  gpsNewFix = false;

  // Pit signal auto-clears after 10s
  if (pitSignal && pitSignalSetMs && (now - pitSignalSetMs > 10000)) {
    pitSignal      = false;
    pitSignalSetMs = 0;
  }

  // Debug print every 2s
  static uint32_t lastDebugMs = 0;
  if (now - lastDebugMs >= 2000) {
    Serial.printf("[%lus] RPM1:%.0f RPM2:%.0f SPD:%.1f T1:%.1f "
                  "V:%.2f SATS:%d DAQ:%s SW:%s\n",
                  now / 1000,
                  currentRpm1, currentRpm2, gpsSpeedMph,
                  temp1C, batteryVoltage, gpsSats,
                  daqMode         ? "ON"  : "off",
                  daqSwitchActive ? "ON"  : "off");
    lastDebugMs = now;
  }
}
