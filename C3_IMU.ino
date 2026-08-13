/*
  ============================================================================
  GAIT IMU NODE  —  Seeed XIAO ESP32-C3 + MPU6050
  ============================================================================
  Streams 6-axis IMU data at 200 Hz over BLE to a PC or Android browser
  running the companion index.html (Web Bluetooth) app, alongside two other
  identical nodes (one per limb/segment).

  FLASH THIS SAME SKETCH ONTO ALL 3 BOARDS — the only thing you change per
  board is the DEVICE_ROLE line below. That sets the BLE advertised name so
  the host app knows which limb/segment it is talking to.

      ROLE_RIGHT_LEG  -> "GAIT_C3_RLEG"
      ROLE_LEFT_LEG   -> "GAIT_C3_LLEG"
      ROLE_TORSO      -> "GAIT_C3_TORSO"

  Board package : "esp32" by Espressif (XIAO_ESP32C3)
  Libraries     : BLEDevice / BLEServer / BLEUtils / BLE2902 (bundled with
                  the ESP32 Arduino core's classic BLE stack — same one the
                  original single-node sketch used), Wire, esp_task_wdt

  BLE protocol (same Service/Control/Data UUIDs as the original sketch, plus
  a new Status characteristic):
    Service            4fafc201-1fb5-459e-8fcc-c5c9c331914b
      Control (Write)  beb5483e-36e1-4688-b7f5-ea07361b26a8
      Data    (Notify) 0000ffe1-0000-1000-8000-00805f9b34fb
      Status  (Notify) 0000ffe2-0000-1000-8000-00805f9b34fb

  COMMANDS written to Control characteristic (ASCII text, same style as the
  original sketch):
    "START"            -> Start recording (clears ring buffer + counters)
    "STOP"             -> Stop recording (drains any buffered samples, then idle)
    "RESET"            -> Reset (clears buffer + counters, preserves record state)
    "TIME:<epoch_ms>"  -> Time sync: host tells the node "right now is epoch_ms",
                          sent as decimal ASCII, e.g. "TIME:1786002365365"

  DATA packet (Notify, header + up to BATCH_SIZE samples, always fits in one
  BLE notification under the negotiated MTU):
    byte 0        : packet type = 0x01
    byte 1        : sample count N in this packet
    bytes 2..5    : uint32 LE sequence number of the FIRST sample in packet
    then N * 16-byte samples:
        uint32 LE   t_ms   (millis() on the node when this sample was captured)
        int16  LE   ax, ay, az   (raw MPU6050 accel counts, +/-ACCEL_RANGE_G g full scale)
        int16  LE   gx, gy, gz   (raw MPU6050 gyro counts,  +/-GYRO_RANGE_DPS dps full scale)

  STATUS packet (Notify, ~1 Hz, or immediately after a time-sync command):
    byte 0        : packet type = 0x02 (time-sync ack) or 0x03 (heartbeat)
    type 0x02 payload: uint64 LE epoch_ms_echo, uint32 LE millis_at_sync
    type 0x03 payload: uint8 recording(0/1), uint8 bufferUsedPercent,
                        uint32 LE overflowCount, uint16 LE sampleRateHzx10,
                        uint32 LE writeFailCount (always 0 on this board —
                        ESP32's classic BLE stack, unlike ArduinoBLE, doesn't
                        report whether a notify() actually reached the queue,
                        so this can't be tracked here; kept in the layout
                        purely so the host-side parser matches),
                        uint32 LE missedSampleCount (samples genuinely lost
                        to a large scheduling stall, e.g. a long BLE hiccup
                        — should stay at 0 under normal conditions)

  Sequence numbers let the host detect dropped packets (gaps in the
  numbering) even though the ring buffer is designed to make that rare.
  ============================================================================
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <esp_task_wdt.h>

// ---------------------------------------------------------------------------
// 1. SET THIS BEFORE UPLOADING TO EACH BOARD
// ---------------------------------------------------------------------------
#define ROLE_RIGHT_LEG 1
#define ROLE_LEFT_LEG  2
#define ROLE_TORSO     3

#define DEVICE_ROLE ROLE_RIGHT_LEG   // <-- CHANGE PER BOARD, then re-upload

#if DEVICE_ROLE == ROLE_RIGHT_LEG
  #define DEVICE_NAME "GAIT_C3_RLEG"
#elif DEVICE_ROLE == ROLE_LEFT_LEG
  #define DEVICE_NAME "GAIT_C3_LLEG"
#elif DEVICE_ROLE == ROLE_TORSO
  #define DEVICE_NAME "GAIT_C3_TORSO"
#else
  #error "Set DEVICE_ROLE to one of ROLE_RIGHT_LEG / ROLE_LEFT_LEG / ROLE_TORSO"
#endif

// --- Configuration ---
#define SDA_PIN 6
#define SCL_PIN 7
#define SAMPLING_RATE 200 // Hz
#define INTERVAL_MS (1000 / SAMPLING_RATE)

// If loop() ever falls behind schedule (slow BLE write, brief I2C hiccup),
// cap how many back-to-back "catch-up" samples we take in a single pass
// before yielding back to drainToBLE() — prevents one bad stall from
// turning into a runaway burst that starves the radio.
#define MAX_CATCHUP_SAMPLES 4

// --- BLE UUIDs ---
#define SERVICE_UUID          "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_CMD    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_DATA   "0000ffe1-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_STATUS "0000ffe2-0000-1000-8000-00805f9b34fb"

// Bytes per data packet: header(6) + BATCH_SIZE * 16.
// BATCH_SIZE=7 -> 118 bytes, comfortably under the 125-byte usable payload
// of the 128-byte MTU this sketch requests (matches the original sketch's
// MTU tuning, just repacked without the unused temperature field).
#define BATCH_SIZE 7
#define DATA_PACKET_MAX (6 + BATCH_SIZE * 16)

// Ring buffer: 2048 samples = ~10 s of buffering at 200 Hz, survives brief
// BLE stalls/reconnects without losing data (oldest is only overwritten if
// the host falls more than 10 s behind, which is logged via overflowCount).
#define RING_SIZE 2048

// --- MPU6050 Registers ---
#define MPU_ADDR      0x68
#define ACCEL_XOUT_H  0x3B
#define PWR_MGMT_1    0x6B
#define ACCEL_CONFIG  0x1C
#define GYRO_CONFIG   0x1B

// Full-scale ranges — explicitly set in setupMPU() below rather than relying
// on power-on defaults, so they're deterministic. MUST match the
// ACCEL_RANGE_G / GYRO_RANGE_DPS constants in index.html (used there to
// convert raw counts to g / dps for the CSV).
#define ACCEL_RANGE_G   2     // AFS_SEL = 0
#define GYRO_RANGE_DPS  250   // FS_SEL  = 0

// --- Watchdog ---
// If loop() ever stalls (I2C hang, unexpected deadlock, anything) for more
// than WATCHDOG_TIMEOUT_S seconds, the ESP32-C3 resets itself automatically
// instead of staying locked up until someone power-cycles it. On reset it
// re-advertises and the host app just needs to reconnect.
// NOTE: this uses the esp_task_wdt API shipped with ESP32 Arduino core 2.x.
// If your installed core is 3.x, esp_task_wdt_init() takes a
// esp_task_wdt_config_t* instead of (timeout_s, panic) — adjust startWatchdog()
// accordingly if the sketch fails to compile against your core version.
#define WATCHDOG_TIMEOUT_S 3

// --- Global Variables ---
BLEServer *pServer = NULL;
BLECharacteristic *pCmdCharacteristic = NULL;
BLECharacteristic *pDataCharacteristic = NULL;
BLECharacteristic *pStatusCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool isRecording = false;

struct __attribute__((packed)) ImuSample {
  uint32_t t;                 // millis() at capture
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

ImuSample ringBuf[RING_SIZE];
volatile uint16_t ringHead = 0;          // next write index
volatile uint16_t ringTail = 0;          // next read (unsent) index
volatile uint32_t seqNext  = 0;          // sequence number of the NEXT sample captured
uint32_t seqOut            = 0;          // sequence number of the next unsent sample
volatile uint32_t overflowCount = 0;
volatile uint32_t missedSampleCount = 0; // samples genuinely lost to a large scheduling stall
unsigned long nextSampleDueMs = 0;       // millis() at which the next sample is due

uint8_t txBuf[DATA_PACKET_MAX];

// rolling actual-rate estimate
uint32_t rateWindowStart = 0;
uint32_t rateWindowCount = 0;
float actualHz = 0;
unsigned long lastStatusMs = 0;

// time sync
uint64_t epochAtSync = 0;   // host epoch_ms captured at last time sync
uint32_t millisAtSync = 0;  // node millis() at that same instant
bool timeSynced = false;

// --- Forward declarations (used by the callback classes below) ---
void startRecording();
void stopRecording();
void resetData();
void handleTimeSync(uint64_t epoch);
void sendTimeSyncAck();
void sendHeartbeat();

// Command Callback
class CmdCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue();
      if (value.length() == 0) return;

      if (value == "START") {
        startRecording();
      } else if (value == "STOP") {
        stopRecording();
      } else if (value == "RESET") {
        resetData();
      } else if (value.startsWith("TIME:")) {
        uint64_t epoch = strtoull(value.substring(5).c_str(), NULL, 10);
        handleTimeSync(epoch);
      }
    }
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

void setupMPU() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Wake the sensor up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(100);

  // Explicitly set full-scale ranges (see ACCEL_RANGE_G / GYRO_RANGE_DPS above)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x00); // AFS_SEL=0 -> +/-2g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x00); // FS_SEL=0 -> +/-250 dps
  Wire.endTransmission(true);
  delay(10);
}

void readMPURaw(int16_t &ax, int16_t &ay, int16_t &az,
                 int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // discard temperature — not used downstream
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

// ---------------------------------------------------------------------------
// Sample capture -> ring buffer
// ---------------------------------------------------------------------------
void pushSample() {
  int16_t ax, ay, az, gx, gy, gz;
  readMPURaw(ax, ay, az, gx, gy, gz);

  uint16_t nextHead = (ringHead + 1) % RING_SIZE;
  if (nextHead == ringTail) {
    // buffer full: drop this newest sample, keep older data intact
    overflowCount++;
    seqNext++;   // sequence still advances so seq numbers stay meaningful
    return;
  }

  ImuSample &s = ringBuf[ringHead];
  s.t = millis();
  s.ax = ax; s.ay = ay; s.az = az;
  s.gx = gx; s.gy = gy; s.gz = gz;
  ringHead = nextHead;
  seqNext++;

  rateWindowCount++;
}

// ---------------------------------------------------------------------------
// Drain ring buffer into BLE notifications (called from loop())
// ---------------------------------------------------------------------------
void drainToBLE() {
  if (!deviceConnected) return;

  uint16_t available = (ringHead - ringTail + RING_SIZE) % RING_SIZE;
  if (available == 0) return;

  uint8_t n = (uint8_t)min((uint16_t)BATCH_SIZE, available);

  txBuf[0] = 0x01;
  txBuf[1] = n;
  memcpy(&txBuf[2], (const void*)&seqOut, 4);

  int off = 6;
  uint16_t idx = ringTail;
  for (uint8_t i = 0; i < n; i++) {
    ImuSample &s = ringBuf[idx];
    memcpy(&txBuf[off], &s.t, 4);   off += 4;
    memcpy(&txBuf[off], &s.ax, 12); off += 12;
    idx = (idx + 1) % RING_SIZE;
  }

  pDataCharacteristic->setValue(txBuf, off);
  pDataCharacteristic->notify();

  // ESP32's classic BLE stack doesn't hand back a per-call success/failure
  // status the way ArduinoBLE's writeValue() does, so unlike the nRF52
  // reference sketch we can't conditionally hold back ringTail/seqOut on
  // failure — we advance them here on the assumption the notify was queued.
  ringTail = idx;
  seqOut += n;
}

// ---------------------------------------------------------------------------
// Status / heartbeat
// ---------------------------------------------------------------------------
void sendHeartbeat() {
  if (!deviceConnected) return;

  uint16_t available = (ringHead - ringTail + RING_SIZE) % RING_SIZE;
  uint8_t usedPct = (uint8_t)((available * 100UL) / RING_SIZE);

  uint8_t buf[17];
  buf[0] = 0x03;
  buf[1] = isRecording ? 1 : 0;
  buf[2] = usedPct;
  uint32_t ofc = overflowCount;
  memcpy(&buf[3], &ofc, 4);
  uint16_t rateX10 = (uint16_t)(actualHz * 10.0f);
  memcpy(&buf[7], &rateX10, 2);
  uint32_t wfc = 0; // see STATUS packet note at top of file
  memcpy(&buf[9], &wfc, 4);
  uint32_t msc = missedSampleCount;
  memcpy(&buf[13], &msc, 4);

  pStatusCharacteristic->setValue(buf, 17);
  pStatusCharacteristic->notify();
}

void sendTimeSyncAck() {
  if (!deviceConnected) return;
  uint8_t buf[13];
  buf[0] = 0x02;
  memcpy(&buf[1], &epochAtSync, 8);
  memcpy(&buf[9], &millisAtSync, 4);
  pStatusCharacteristic->setValue(buf, 13);
  pStatusCharacteristic->notify();
}

// ---------------------------------------------------------------------------
// Recording control
// ---------------------------------------------------------------------------
void startRecording() {
  ringHead = 0;
  ringTail = 0;
  seqNext = 0;
  seqOut = 0;
  overflowCount = 0;
  missedSampleCount = 0;
  nextSampleDueMs = millis();
  rateWindowCount = 0;
  rateWindowStart = millis();
  isRecording = true;
  Serial.println("Recording Started");
}

void stopRecording() {
  isRecording = false; // loop() keeps draining any samples already buffered
  Serial.println("Recording Stopped");
}

void resetData() {
  bool wasRecording = isRecording;
  isRecording = false;
  ringHead = 0;
  ringTail = 0;
  seqNext = 0;
  seqOut = 0;
  overflowCount = 0;
  missedSampleCount = 0;
  nextSampleDueMs = millis();
  rateWindowCount = 0;
  rateWindowStart = millis();
  isRecording = wasRecording;
  Serial.println("Sequence & State Reset");
}

void handleTimeSync(uint64_t epoch) {
  epochAtSync = epoch;
  millisAtSync = millis();
  timeSynced = true;
  sendTimeSyncAck();
}

// ---------------------------------------------------------------------------
// Watchdog
// ---------------------------------------------------------------------------
void startWatchdog() {
#if ESP_IDF_VERSION_MAJOR >= 5
  // ESP32 Arduino core 3.x (IDF 5.x): struct-based config API.
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WATCHDOG_TIMEOUT_S * 1000,
    .idle_core_mask = 0,     // don't watch idle tasks, just ours below
    .trigger_panic = true    // reset the board on timeout
  };
  esp_task_wdt_init(&wdtConfig);
#else
  // ESP32 Arduino core 2.x (IDF 4.x): simple (timeout_s, panic) API.
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL); // watch the current (loop) task
}

void setup() {
  Serial.begin(115200);
  setupMPU();

  BLEDevice::init(DEVICE_NAME);

  // --- RANGE & THROUGHPUT OPTIMIZATIONS ---
  BLEDevice::setPower(ESP_PWR_LVL_P9); // Max TX Power (+9dBm)
  BLEDevice::setMTU(128);              // Request 128 MTU (Allows 125 bytes payload)
  // Note: Connection parameters are automatically negotiated by the central device (phone/PC)

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCmdCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_CMD, BLECharacteristic::PROPERTY_WRITE);
  pCmdCharacteristic->setCallbacks(new CmdCallback());

  pDataCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_DATA, BLECharacteristic::PROPERTY_NOTIFY);
  pDataCharacteristic->addDescriptor(new BLE2902());

  pStatusCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_STATUS, BLECharacteristic::PROPERTY_NOTIFY);
  pStatusCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();

  lastStatusMs = millis();
  rateWindowStart = millis();
  nextSampleDueMs = millis();

  startWatchdog();

  Serial.print(DEVICE_NAME);
  Serial.println(": waiting for Web Bluetooth connection...");
}

void loop() {
  esp_task_wdt_reset();

  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // --- 200 Hz sampling: plain millis() polling, no timer interrupt.
  //     nextSampleDueMs advances by exactly INTERVAL_MS each time (never
  //     reset to "now"), so if the loop briefly falls behind, the while()
  //     below genuinely catches up with fresh sensor reads on the next few
  //     passes rather than silently discarding samples. MAX_CATCHUP_SAMPLES
  //     bounds how many catch-up reads happen in one pass so a bad stall
  //     can't turn into a runaway burst that starves BLE. If we're still
  //     far behind after the cap, we resync to "now" and count the
  //     remainder as genuinely missed rather than trying to catch up forever.
  unsigned long nowMs = millis();
  if (isRecording) {
    uint8_t caughtUp = 0;
    while ((long)(nowMs - nextSampleDueMs) >= 0 && caughtUp < MAX_CATCHUP_SAMPLES) {
      pushSample();
      nextSampleDueMs += INTERVAL_MS;
      caughtUp++;
    }
    long stillBehindMs = (long)(nowMs - nextSampleDueMs);
    if (stillBehindMs > (long)(INTERVAL_MS * 20)) {
      missedSampleCount += (uint32_t)(stillBehindMs / INTERVAL_MS);
      nextSampleDueMs = nowMs;
    }
  }

  // --- stream buffered samples out ---
  drainToBLE();

  // --- rate estimate + heartbeat, ~1 Hz ---
  unsigned long n2 = millis();
  if (n2 - rateWindowStart >= 1000) {
    actualHz = rateWindowCount * 1000.0f / (n2 - rateWindowStart);
    rateWindowCount = 0;
    rateWindowStart = n2;
  }
  if (n2 - lastStatusMs >= 1000) {
    lastStatusMs = n2;
    sendHeartbeat();
  }
}
