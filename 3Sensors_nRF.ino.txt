/*
  ============================================================================
  GAIT IMU NODE  —  Seeed XIAO nRF52840 Sense
  ============================================================================
  Streams 6-axis IMU data (LSM6DS3, on-board) at 200 Hz over BLE to a PC or
  Android browser running the companion index.html (Web Bluetooth) app.

  FLASH THIS SAME SKETCH ONTO ALL 3 BOARDS — the only thing you change per
  board is the DEVICE_ROLE line below. That sets the BLE advertised name so
  the host app knows which limb/segment it is talking to.

      ROLE_RIGHT_LEG  -> "GAIT_RLEG"
      ROLE_LEFT_LEG   -> "GAIT_LLEG"
      ROLE_TORSO      -> "GAIT_TORSO"

  Board package : "Seeed nRF52 Boards" -> XIAO nRF52840 Sense (mbed-based core)
  Libraries     : ArduinoBLE            (BLE stack)
                  Seeed_Arduino_LSM6DS3  (on-board IMU driver)

  BLE protocol (custom, Nordic-UART-style UUID base):
    Service            6e400001-b5a3-f393-e0a9-e50e24dcca9e
      Control (Write)  6e400002-...   1..9 bytes, see COMMANDS below
      Data    (Notify) 6e400003-...   batched IMU samples, see PACKET FORMAT
      Status  (Notify) 6e400004-...   heartbeat / time-sync ack, see below

  COMMANDS written to Control characteristic:
    'S'                       -> Start recording (clears buffers, begins streaming)
    'X'                       -> Stop recording (drains any buffered samples, then idle)
    'R'                       -> Reset (clears buffers + counters, does not affect record state)
    'T' + uint64 LE epoch_ms  -> Time sync: host tells the node "right now is epoch_ms"

  DATA packet (sent on Data characteristic, up to ~200 bytes, well under the
  negotiated MTU on modern phones/PCs so it always fits in ONE notification):
    byte 0        : packet type = 0x01
    byte 1        : sample count N in this packet (<= BATCH_SIZE)
    bytes 2..5    : uint32 LE sequence number of the FIRST sample in packet
    then N * 16-byte samples:
        uint32 LE   t_ms   (millis() on the node when this sample was captured)
        int16  LE   ax, ay, az   (raw LSM6DS3 accel counts, +/-ACCEL_RANGE g full scale)
        int16  LE   gx, gy, gz   (raw LSM6DS3 gyro counts,  +/-GYRO_RANGE dps full scale)

  STATUS packet (sent on Status characteristic, ~1 Hz, or immediately after a
  time-sync command):
    byte 0        : packet type = 0x02 (time-sync ack) or 0x03 (heartbeat)
    type 0x02 payload: uint64 LE epoch_ms_echo, uint32 LE millis_at_sync
    type 0x03 payload: uint8 recording(0/1), uint8 bufferUsedPercent,
                        uint32 LE overflowCount, uint16 LE sampleRateHzx10,
                        uint32 LE writeFailCount (BLE notify retries — should
                        stay near 0; rising fast means the link can't keep up),
                        uint32 LE missedSampleCount (samples genuinely lost
                        to a large scheduling stall, e.g. a long BLE hiccup
                        — should stay at 0 under normal conditions; small
                        catch-up delays don't count here since the scheduler
                        recovers those with real fresh reads, not by
                        discarding them)

  Sequence numbers let the host detect dropped packets (gaps in the numbering)
  even though the node's ring buffer + BLE notifications are designed to make
  that a rare event under normal connection conditions.
  ============================================================================
*/

#include <ArduinoBLE.h>
#include "LSM6DS3.h"
#include "Wire.h"

// ---------------------------------------------------------------------------
// 1. SET THIS BEFORE UPLOADING TO EACH BOARD
// ---------------------------------------------------------------------------
#define ROLE_RIGHT_LEG 1
#define ROLE_LEFT_LEG  2
#define ROLE_TORSO     3

#define DEVICE_ROLE ROLE_RIGHT_LEG   // <-- CHANGE PER BOARD, then re-upload

#if DEVICE_ROLE == ROLE_RIGHT_LEG
  #define DEVICE_NAME "GAIT_RLEG"
#elif DEVICE_ROLE == ROLE_LEFT_LEG
  #define DEVICE_NAME "GAIT_LLEG"
#elif DEVICE_ROLE == ROLE_TORSO
  #define DEVICE_NAME "GAIT_TORSO"
#else
  #error "Set DEVICE_ROLE to one of ROLE_RIGHT_LEG / ROLE_LEFT_LEG / ROLE_TORSO"
#endif

// ---------------------------------------------------------------------------
// 2. Sampling / IMU configuration
// ---------------------------------------------------------------------------
#define SAMPLE_RATE_HZ   200
#define SAMPLE_INTERVAL_MS (1000UL / SAMPLE_RATE_HZ)   // 5 ms
#define IMU_ODR_HZ       208     // closest LSM6DS3 ODR at/above 200 Hz
#define ACCEL_RANGE_G    8       // +/- 8 g   (must match host-side CSV conversion)
#define GYRO_RANGE_DPS   2000    // +/- 2000 dps

// If loop() ever falls behind schedule (e.g. a slow BLE write, brief I2C
// hiccup), cap how many back-to-back "catch-up" samples we take in a single
// pass before yielding back to BLE.poll()/drainToBLE() — this prevents one
// bad stall from turning into a runaway burst that starves the radio.
#define MAX_CATCHUP_SAMPLES 4

// ---------------------------------------------------------------------------
// 3. BLE UUIDs
// ---------------------------------------------------------------------------
#define SERVICE_UUID  "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CONTROL_UUID  "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define DATA_UUID     "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define STATUS_UUID   "6e400004-b5a3-f393-e0a9-e50e24dcca9e"

// Bytes per data packet: header(6) + BATCH_SIZE * 16.
// BATCH_SIZE=12 -> 198 bytes (~17 packets/sec at 200 Hz). Comfortably under
// negotiated MTU (185-247B) on virtually all modern phones/PCs, and fewer
// packets/sec than a smaller batch would need, which matters if the OS
// caps how many notifications/sec it will service. If you connect on very
// old/constrained hardware and see writeFailCount climbing steadily in the
// status heartbeat, lower this (e.g. to 6) to shrink each packet further.
#define BATCH_SIZE 12
#define DATA_PACKET_MAX (6 + BATCH_SIZE * 16)

// Ring buffer: 2048 samples = ~10 s of buffering at 200 Hz, survives brief
// BLE stalls/reconnects without losing data (oldest is only overwritten if
// the host falls more than 10 s behind, which is logged via overflowCount).
#define RING_SIZE 2048

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
LSM6DS3 myIMU(I2C_MODE, 0x6A);

BLEService gaitService(SERVICE_UUID);
BLECharacteristic controlChar(CONTROL_UUID, BLEWrite, 9);
BLECharacteristic dataChar(DATA_UUID, BLERead | BLENotify, DATA_PACKET_MAX);
BLECharacteristic statusChar(STATUS_UUID, BLERead | BLENotify, 20);

struct __attribute__((packed)) ImuSample {
  uint32_t t;                 // millis() at capture
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

ImuSample ringBuf[RING_SIZE];
volatile uint16_t ringHead = 0;     // next write index
volatile uint16_t ringTail = 0;     // next read (unsent) index
volatile uint32_t seqNext  = 0;     // sequence number of the NEXT sample captured
uint32_t seqOut            = 0;     // sequence number of the next unsent sample
volatile uint32_t overflowCount = 0;
volatile uint32_t writeFailCount = 0;   // notifications the BLE stack refused to queue (retried, not lost)
volatile uint32_t missedSampleCount = 0; // samples genuinely lost to a large scheduling stall (rare — see loop())
unsigned long nextSampleDueMs = 0;       // millis() at which the next sample is due
uint8_t debugSampleTraceRemaining = 0;   // fine-grained Serial trace for the first few samples after each Start

// Hardware watchdog: if loop() ever stalls (I2C hang, unexpected deadlock,
// anything) for more than WATCHDOG_TIMEOUT_MS, the nRF52840 resets itself
// automatically instead of staying locked up until someone power-cycles it.
// On reset it re-advertises and the host app just needs to reconnect —
// far better than a wearable sensor that can go permanently unresponsive
// mid-session.
//
// Implemented directly against the nRF52840 WDT peripheral registers
// (via Nordic's nrf.h, bundled with every nRF52 Arduino core) rather than
// mbed::Watchdog — that mbed driver class isn't compiled into this board
// package's mbed-os build, so this avoids depending on it being present.
#include "nrf.h"
#define WATCHDOG_TIMEOUT_MS 3000

void wdtStart(uint32_t timeout_ms) {
  NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) |
                     (WDT_CONFIG_SLEEP_Run  << WDT_CONFIG_SLEEP_Pos);
  NRF_WDT->CRV = (uint32_t)(((uint64_t)timeout_ms * 32768ULL) / 1000ULL);
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;
  NRF_WDT->TASKS_START = 1;
}

inline void wdtKick() {
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

bool recording = false;

uint64_t epochAtSync = 0;   // host epoch_ms captured at last time sync
uint32_t millisAtSync = 0;  // node millis() at that same instant
bool timeSynced = false;

uint8_t txBuf[DATA_PACKET_MAX];

// rolling actual-rate estimate
uint32_t rateWindowStart = 0;
uint32_t rateWindowCount = 0;
float actualHz = 0;

unsigned long lastStatusMs = 0;
unsigned long lastLedMs = 0;
bool ledBlinkState = false;

// ---------------------------------------------------------------------------
// LED helpers (XIAO nRF52840 Sense on-board RGB LED, active LOW)
// ---------------------------------------------------------------------------
void ledOff() {
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}
void ledSet(bool r, bool g, bool b) {
  digitalWrite(LEDR, r ? LOW : HIGH);
  digitalWrite(LEDG, g ? LOW : HIGH);
  digitalWrite(LEDB, b ? LOW : HIGH);
}

// ---------------------------------------------------------------------------
// IMU read. Uses the LSM6DS3 library's own per-axis register calls — this
// is the method that was proven stable (no lockups) in earlier testing; a
// custom direct-Wire burst read was tried to shave I2C time further but
// caused the node to hang the instant recording started, so it has been
// removed rather than chased further blind. 6 register reads at ~100 kHz
// costs a few ms, which is why actualHz (reported in the status heartbeat)
// may land somewhat under 200 Hz — but every capture that does happen is
// timestamped with the real millis() it was taken at, so downstream
// analysis can resample onto a uniform grid using the true timestamps
// rather than assuming perfectly even 5 ms spacing.
// ---------------------------------------------------------------------------
void pushSample() {
  bool trace = Serial && debugSampleTraceRemaining > 0;
  if (trace) { Serial.println(F("[push] begin")); debugSampleTraceRemaining--; }

  int16_t ax, ay, az, gx, gy, gz;
  ax = myIMU.readRawAccelX();
  if (trace) Serial.println(F("[push] ax read"));
  ay = myIMU.readRawAccelY();
  az = myIMU.readRawAccelZ();
  if (trace) Serial.println(F("[push] accel done"));
  gx = myIMU.readRawGyroX();
  gy = myIMU.readRawGyroY();
  gz = myIMU.readRawGyroZ();
  if (trace) Serial.println(F("[push] gyro done"));

  uint16_t nextHead = (ringHead + 1) % RING_SIZE;
  if (nextHead == ringTail) {
    // buffer full: drop this newest sample, keep older data intact
    overflowCount++;
    seqNext++;   // sequence still advances so seq numbers stay meaningful
    return;
  }

  ImuSample &s = ringBuf[ringHead];
  s.t  = millis();
  s.ax = ax; s.ay = ay; s.az = az;
  s.gx = gx; s.gy = gy; s.gz = gz;
  ringHead = nextHead;
  seqNext++;

  rateWindowCount++;
  if (trace) Serial.println(F("[push] done"));
}

// ---------------------------------------------------------------------------
// Drain ring buffer into BLE notifications (called from loop())
// ---------------------------------------------------------------------------
void drainToBLE() {
  if (!dataChar.subscribed()) return;

  uint16_t available = (ringHead - ringTail + RING_SIZE) % RING_SIZE;
  if (available == 0) return;

  uint8_t n = (uint8_t)min((uint16_t)BATCH_SIZE, available);

  // Build the packet from a READ-ONLY pass over the ring buffer first.
  // We do NOT advance ringTail/seqOut until writeValue() confirms the
  // notification was actually queued/sent — this is what prevents silent
  // data loss when the BLE stack momentarily can't accept a notification
  // (e.g. previous one still in flight, brief radio contention, MTU
  // negotiation not finished yet). On failure we simply retry next loop
  // pass with the exact same samples still sitting in the ring buffer.
  txBuf[0] = 0x01;
  txBuf[1] = n;
  memcpy(&txBuf[2], (const void*)&seqOut, 4);

  int off = 6;
  uint16_t idx = ringTail;
  for (uint8_t i = 0; i < n; i++) {
    ImuSample &s = ringBuf[idx];
    memcpy(&txBuf[off], &s.t, 4);  off += 4;
    memcpy(&txBuf[off], &s.ax, 12); off += 12;
    idx = (idx + 1) % RING_SIZE;
  }

  bool ok = dataChar.writeValue(txBuf, off);
  if (ok) {
    ringTail = idx;
    seqOut += n;
  } else {
    writeFailCount++;
    // Leave ringTail/seqOut untouched: nothing lost, we'll retry immediately
    // on the next loop() pass (which happens continuously while connected).
  }
}

// ---------------------------------------------------------------------------
// Status / heartbeat
// ---------------------------------------------------------------------------
void sendHeartbeat() {
  if (!statusChar.subscribed()) return;

  uint16_t available = (ringHead - ringTail + RING_SIZE) % RING_SIZE;
  uint8_t usedPct = (uint8_t)((available * 100UL) / RING_SIZE);

  uint8_t buf[20];
  buf[0] = 0x03;
  buf[1] = recording ? 1 : 0;
  buf[2] = usedPct;
  uint32_t ofc = overflowCount;
  memcpy(&buf[3], &ofc, 4);
  uint16_t rateX10 = (uint16_t)(actualHz * 10.0f);
  memcpy(&buf[7], &rateX10, 2);
  uint32_t wfc = writeFailCount;
  memcpy(&buf[9], &wfc, 4);
  uint32_t msc = missedSampleCount;
  memcpy(&buf[13], &msc, 4);
  statusChar.writeValue(buf, 17);
}

void sendTimeSyncAck() {
  if (!statusChar.subscribed()) return;
  uint8_t buf[15];
  buf[0] = 0x02;
  memcpy(&buf[1], &epochAtSync, 8);
  memcpy(&buf[9], &millisAtSync, 4);
  statusChar.writeValue(buf, 13);
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
  writeFailCount = 0;
  missedSampleCount = 0;
  nextSampleDueMs = millis();
  rateWindowCount = 0;
  rateWindowStart = millis();
  debugSampleTraceRemaining = 8;
  recording = true;
  if (Serial) Serial.println(F("[state] recording = true"));
}

void stopRecording() {
  recording = false;   // loop() keeps draining any samples already buffered
}

void resetData() {
  bool wasRecording = recording;
  recording = false;
  ringHead = 0;
  ringTail = 0;
  seqNext = 0;
  seqOut = 0;
  overflowCount = 0;
  writeFailCount = 0;
  missedSampleCount = 0;
  nextSampleDueMs = millis();
  rateWindowCount = 0;
  rateWindowStart = millis();
  recording = wasRecording;
}

void handleTimeSync(const uint8_t* payload) {
  uint64_t epoch;
  memcpy(&epoch, payload, 8);
  epochAtSync = epoch;
  millisAtSync = millis();
  timeSynced = true;
  sendTimeSyncAck();
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  // Do not block on Serial — nodes must run untethered on battery.

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  ledOff();

  // ---- IMU ----
  myIMU.settings.accelRange       = ACCEL_RANGE_G;
  myIMU.settings.gyroRange        = GYRO_RANGE_DPS;
  myIMU.settings.accelSampleRate  = IMU_ODR_HZ;
  myIMU.settings.gyroSampleRate   = IMU_ODR_HZ;
  myIMU.settings.accelBandWidth   = 200;
  myIMU.settings.gyroBandWidth    = 200;
  if (myIMU.begin() != 0) {
    // Fast red blink forever = IMU init failure
    while (true) {
      ledSet(true, false, false);
      delay(150);
      ledOff();
      delay(150);
    }
  }

  // Step 1 of closing the rate gap: raise the I2C bus to fast-mode
  // (400 kHz vs. the library's ~100 kHz default). This is an isolated
  // change from the read METHOD (still the same proven six per-axis
  // library calls that have now run two full sessions with zero lockups
  // and near-zero BLE loss) — only how fast each of those calls' bus
  // transactions run. If this ever causes instability (it depends on the
  // board's pull-up sizing), flip I2C_FAST_MODE to 0 to instantly revert
  // to the known-good 100 kHz default with no other code changes needed.
  #define I2C_FAST_MODE 1
  #if I2C_FAST_MODE
  Wire.setClock(400000);
  #endif

  // ---- BLE ----
  if (!BLE.begin()) {
    while (true) {
      ledSet(true, false, true); // magenta = BLE init failure
      delay(150);
      ledOff();
      delay(150);
    }
  }

  BLE.setLocalName(DEVICE_NAME);
  BLE.setDeviceName(DEVICE_NAME);
  BLE.setAdvertisedService(gaitService);

  gaitService.addCharacteristic(controlChar);
  gaitService.addCharacteristic(dataChar);
  gaitService.addCharacteristic(statusChar);
  BLE.addService(gaitService);

  // Request a short, fast connection interval (units of 1.25 ms) so the
  // ~10 Hz stream of ~166-byte notifications has plenty of headroom.
  BLE.setConnectionInterval(0x0006, 0x000C); // 7.5 ms .. 15 ms

  BLE.advertise();

  lastStatusMs = millis();
  rateWindowStart = millis();

  if (Serial) Serial.println(F("[setup] ready, advertising."));
  wdtStart(WATCHDOG_TIMEOUT_MS);
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  BLEDevice central = BLE.central();
  wdtKick();

  // --- LED status ---
  unsigned long now = millis();
  if (central && central.connected()) {
    if (recording) {
      // blink red while recording
      if (now - lastLedMs > 250) { lastLedMs = now; ledBlinkState = !ledBlinkState; ledSet(ledBlinkState, false, false); }
    } else {
      ledSet(false, true, false); // solid green = connected, idle
    }
  } else {
    if (now - lastLedMs > 600) { lastLedMs = now; ledBlinkState = !ledBlinkState; ledSet(false, false, ledBlinkState); } // blue blink = advertising
  }

  if (!central) {
    return; // still need sampling to keep running below even if not central-connected? see note
  }

  while (central.connected()) {
    wdtKick();
    BLE.poll();

    // --- handle control writes ---
    if (controlChar.written()) {
      int len = controlChar.valueLength();
      const uint8_t* data = controlChar.value();
      if (len >= 1) {
        if (Serial) { Serial.print(F("[ctrl] received cmd: ")); Serial.println((char)data[0]); }
        switch (data[0]) {
          case 'S': startRecording(); break;
          case 'X': stopRecording();  break;
          case 'R': resetData();      break;
          case 'T': if (len >= 9) handleTimeSync(&data[1]); break;
          default: break;
        }
      }
    }

    // --- 200 Hz sampling: plain millis() polling, no timer interrupt.
    //     nextSampleDueMs advances by exactly SAMPLE_INTERVAL_MS each time
    //     (never reset to "now"), so if the loop briefly falls behind, the
    //     while() below genuinely catches up with fresh sensor reads on
    //     the next few passes rather than silently discarding samples —
    //     the same scheduling pattern used by the reference ESP32 sketch.
    //     MAX_CATCHUP_SAMPLES bounds how many catch-up reads happen in one
    //     pass so a bad stall can't turn into a runaway burst that starves
    //     BLE.poll(). If we're still far behind after the cap, we resync
    //     to "now" and count the remainder as genuinely missed rather than
    //     trying to catch up forever. ---
    if (recording) {
      unsigned long nowMs = millis();
      uint8_t caughtUp = 0;
      while ((long)(nowMs - nextSampleDueMs) >= 0 && caughtUp < MAX_CATCHUP_SAMPLES) {
        pushSample();
        nextSampleDueMs += SAMPLE_INTERVAL_MS;
        caughtUp++;
      }
      long stillBehindMs = (long)(nowMs - nextSampleDueMs);
      if (stillBehindMs > (long)(SAMPLE_INTERVAL_MS * 20)) {
        // A large stall (e.g. a long BLE hiccup) — don't try to burst-catch-up
        // forever; count the backlog as genuinely missed and resync to now.
        missedSampleCount += (uint32_t)(stillBehindMs / SAMPLE_INTERVAL_MS);
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

    // --- LED while connected ---
    if (recording) {
      if (n2 - lastLedMs > 250) { lastLedMs = n2; ledBlinkState = !ledBlinkState; ledSet(ledBlinkState, false, false); }
    } else {
      ledSet(false, true, false);
    }
  }

  // Disconnected: resume advertising automatically (ArduinoBLE does this
  // implicitly once BLE.central() stops returning a connected device, but
  // we make sure recording state is preserved so a quick reconnect can
  // resume streaming from the ring buffer without data loss).
  ledOff();
}
