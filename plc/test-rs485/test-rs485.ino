/*
  RS485 / MODBUS RTU TEST PROGRAM FOR THE GATE PLC
  Hardware: Controllino Maxi (SKU 100.100.00)
  Bus: RS485 on Serial3 (DE pin 75, /RE pin 76), Modbus RTU 19200 baud 8E1.
  Devices: Schneider ATV320 VFD gate 1 (address 1), gate 2 (address 2).

  PURPOSE
  Find out if intermittent Modbus RTU errors seen with the main program are
  caused by the RS485 bus (wiring, termination, noise, VFD settings) or by
  software (library behaviour, frame timing).

  The VFDs are polled with the same requests as the main program and every
  result is counted. Two ways of communicating can be selected:
  - LIB: ArduinoModbus library, used exactly as in the main program (same
         calls, 200 ms timeout, no pause between requests).
  - RAW: own Modbus RTU implementation directly on the RS485 port. Every
         received byte is recorded with timing. Errors are classified as:
         no answer, late answer, missing bytes, CRC error, wrong address,
         exception, echo, leftover bytes before a request, bus noise.
  Comparing LIB and RAW, with and without a pause between requests and with
  one or both VFDs, shows where the problem is. The AUTO command runs this
  comparison and prints a summary with observations.

  SAFETY
  The E-STOP relays (R8, R9) are kept de-energized: VFD Safe Torque Off is
  active and the motors cannot run. The program only reads VFD registers.
  With WRITES ON it also writes a stop command (LFR = 0, CMD = 8), the same
  as the main program sends when a gate is at rest.
  Ethernet / Modbus TCP is not used by this program.

  USAGE
  Connect USB, open a serial monitor at 115200 baud (line ending CR, LF or
  both). Polling starts automatically in LIB mode. Send HELP for commands.
  Recommended procedure:
  1. INFO        Check that both VFDs answer, show status and last fault.
  2. SCAN        Check that only addresses 1 and 2 answer (no duplicates).
  3. AUTO 1000   Run all test steps (about 2 x 1000 cycles per step), read
                 the summary table and observations.
  4. LISTEN 10   Listen to the idle bus. No bytes should be received.
  For long tests leave the program running and send STATS now and then.

  ALSO CHECK ON THE VFD DISPLAY
  ATV320 menu [Monitoring] > [Communication map] > [Modbus network diag]
  (names may differ by firmware): Modbus frame counter (M1Ct) and CRC error
  counter (M1EC). A rising CRC error counter on a VFD means the VFD receives
  corrupted requests: physical bus problem, or requests sent without the
  required bus silence between frames.

  INTERPRETATION (short)
  - Errors with no pause, gone with a 5 ms pause: frame timing (software).
  - Errors only in LIB, not in RAW: library behaviour (software).
  - Errors on one VFD also when it is polled alone with pause: bus segment,
    connector, termination or settings of that VFD (hardware).
  - Missing bytes / CRC errors / bus noise: physical layer (termination 120 Ohm
    at both bus ends, bias, shield and 0 V reference, cable routing, EMC).
  - LATE answers: VFD answers after the timeout, increase the timeout.
  Note: motors do not run during this test. Errors caused by motor EMC only
  show while the gates move.
*/

#include <errno.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <Controllino.h>  // Use CONTROLLINO_xx aliases

/* SETTINGS ------------------------------------------------------------------*/
#define SERIAL_BAUD 115200
#define MODBUS_BAUD 19200
#define MODBUS_CONFIG SERIAL_8E1
#define RELAY_G1_ESTOP CONTROLLINO_R8
#define RELAY_G2_ESTOP CONTROLLINO_R9

#define DEFAULT_TIMEOUT 200       // [ms] Response timeout (same as main program)
#define DEFAULT_CYCLE 100         // [ms] Time between start of poll cycles
#define DEFAULT_REPORT 60         // [s] Periodic statistics, 0 = off
#define DEFAULT_LATE_LISTEN 300   // [ms] RAW: keep listening after a timeout
#define FRAME_END_SILENCE_US 4000 // [us] RAW: bus silence ending a received frame
#define AUTO_GAP 5                // [ms] Pause before each request in AUTO steps
#define AUTO_DEFAULT_CYCLES 300
#define AUTO_MAX_CYCLES 60000
#define FAULT_CONSECUTIVE 5       // = VFD_COMM_ERROR_FAULT in the main program
#define SCAN_DEFAULT_MAX 10
#define SERIAL_CMD_MAX_LEN 32

// ATV320 registers (same as main program)
#define REG_ETA 3201  // Status word
#define REG_RFR 3202  // Output frequency [Hz/10]
#define REG_NCV 3011  // Drive rating (identification)
#define REG_LFT 7121  // Last fault code
#define REG_NM1 12741 // Communication scanner input 1 (main program: ETA, RFR)
#define REG_NC1 12761 // Communication scanner output 1 (main program: CMD, LFR)
#define REG_CMD 8501  // Command word
#define REG_LFR 8502  // Frequency reference [Hz/10]
#define CMD_STOP 8    // Stop, fast stop not active (bit 3 inverted)

const uint8_t vfdAddress[2] = {1, 2};

/* TYPES ---------------------------------------------------------------------*/

enum Result : uint8_t {
  R_OK = 0,
  R_TIMEOUT,      // No byte received within timeout
  R_LATE,         // Valid answer, but received after the timeout
  R_PARTIAL,      // Answer incomplete (bytes missing)
  R_CRC,          // Complete answer, CRC wrong
  R_WRONG_SLAVE,  // Answer from other address than requested
  R_EXCEPTION,    // VFD answered with Modbus exception
  R_BAD_DATA,     // Wrong function code, length or content
  R_ECHO,         // Own request received back
  R_OTHER,        // Other library error
  R_COUNT
};

enum Mode : uint8_t { MODE_LIB, MODE_RAW };
enum Pattern : uint8_t { PATTERN_MAIN, PATTERN_SIMPLE };

struct Settings {
  Mode mode = MODE_LIB;
  uint8_t targets = 3;        // Bit 0 = VFD 1, bit 1 = VFD 2
  bool order21 = false;       // Poll VFD 2 before VFD 1
  uint16_t gapMs = 0;         // Bus silence before each request (0 = as main program)
  uint16_t timeoutMs = DEFAULT_TIMEOUT;
  uint16_t cycleMs = DEFAULT_CYCLE;
  Pattern pattern = PATTERN_MAIN;
  bool writes = false;        // Also write stop command (LFR, CMD)
  uint8_t verbose = 1;        // 0 = statistics only, 1 = errors, 2 = all frames
  uint16_t reportS = DEFAULT_REPORT;
  uint16_t lateMs = DEFAULT_LATE_LISTEN;
  bool running = true;
};

#define HIST_BINS 8
const uint16_t histLimitMs[HIST_BINS - 1] = {10, 20, 50, 100, 200, 500, 1000};

struct Stats {
  uint32_t sent;
  uint32_t count[R_COUNT];
  uint32_t staleEvents, staleBytes;
  uint16_t consec, maxConsec;
  uint32_t faultEvents;          // Times FAULT_CONSECUTIVE errors in a row reached
  uint32_t durMin, durMax;       // [us] Transaction time (request start to answer end)
  uint64_t durSum;
  uint32_t turnMin, turnMax;     // [us] RAW: end of request to first answer byte
  uint64_t turnSum;
  uint32_t turnN;
  uint32_t hist[HIST_BINS];
};

#define RX_MAX 40
struct Txn {
  uint8_t vfd;          // 0 = VFD 1, 1 = VFD 2
  uint8_t addr;
  uint8_t func;         // 3 = read holding registers, 6 = write single register
  uint16_t reg;
  uint16_t qtyOrVal;
  uint16_t values[4];
  Result result;
  uint8_t exc;          // Exception code
  const char *libErr;   // Library error text (LIB)
  uint32_t durUs, turnUs;
  bool turnValid;
  uint8_t rx[RX_MAX];
  uint8_t n;            // Received bytes (may exceed RX_MAX, then truncated)
  uint8_t expected;
  uint8_t staleN;       // Bytes in receive buffer before the request
  uint8_t stale[8];
};

#define EVT_QUEUE 6
#define EVT_RX_MAX 24
struct Event {
  uint32_t ms;
  uint8_t vfd, func;
  uint16_t reg, qtyOrVal;
  Result result;
  uint8_t exc;
  Mode mode;
  const char *libErr;
  uint32_t durUs, turnUs;
  bool turnValid;
  uint8_t n, expected;
  uint8_t rx[EVT_RX_MAX];
  uint8_t staleN;
  uint8_t stale[8];
};

struct Summary {
  uint32_t sent, ok, timeout, late, corrupt, wrongSlave, other, staleEvents, faultEvents;
  uint16_t maxConsec;
  uint32_t avgDurUs, maxDurUs;
};

#define AUTO_STEPS 6
struct AutoStep {
  Mode mode;
  uint8_t gapMs;
  uint8_t targets;
};
const AutoStep autoSteps[AUTO_STEPS] = {
  {MODE_LIB, 0, 3},         // 1: as main program
  {MODE_RAW, 0, 3},         // 2: same timing, byte level diagnostics
  {MODE_RAW, AUTO_GAP, 3},  // 3: pause between frames
  {MODE_RAW, AUTO_GAP, 1},  // 4: VFD 1 alone
  {MODE_RAW, AUTO_GAP, 2},  // 5: VFD 2 alone
  {MODE_LIB, AUTO_GAP, 3},  // 6: library with pause between frames
};

/* GLOBAL VARIABLES ----------------------------------------------------------*/
Settings cfg;
Stats stats[2];
Txn txn;
Event evq[EVT_QUEUE];
uint8_t evCount = 0;
uint32_t evDropped = 0;
uint32_t unsolicitedBytes = 0;     // RAW: bytes received while bus idle
uint8_t unsolBuf[16];
uint8_t unsolNew = 0;              // Unsolicited bytes not yet printed
uint32_t statsResetMs = 0;
unsigned long lastBusUs = 0;       // End of last transaction
unsigned long lastCycleMs = 0;
unsigned long lastReportMs = 0;
Summary autoRes[AUTO_STEPS][2];
uint32_t autoUnsol[AUTO_STEPS];

/* SETUP ---------------------------------------------------------------------*/

void setup() {
  // Keep VFD Safe Torque Off active: motors cannot run
  pinMode(RELAY_G1_ESTOP, OUTPUT);
  pinMode(RELAY_G2_ESTOP, OUTPUT);
  digitalWrite(RELAY_G1_ESTOP, LOW);
  digitalWrite(RELAY_G2_ESTOP, LOW);

  Serial.begin(SERIAL_BAUD);
  Serial.println();
  Serial.println(F("=== RS485 / Modbus RTU test program, gate PLC ==="));
  Serial.println(F("E-STOP relays R8/R9 de-energized: Safe Torque Off active, motors cannot run."));

  ModbusRTUClient.setTimeout(cfg.timeoutMs);
  if (!ModbusRTUClient.begin(MODBUS_BAUD, MODBUS_CONFIG)) {
    Serial.println(F("Error starting Modbus RTU client! Program stopped."));
    while (1);
  }
  Serial.print(F("RS485: Serial3, 19200 baud 8E1, DE pin "));
  Serial.print(RS485_DEFAULT_DE_PIN);
  Serial.print(F(", /RE pin "));
  Serial.print(RS485_DEFAULT_RE_PIN);
  Serial.print(F(", DE pre/post delay [us]: "));
  Serial.print(RS485_DEFAULT_PRE_DELAY);
  Serial.print('/');
  Serial.println(RS485_DEFAULT_POST_DELAY);
  Serial.println(F("Polling started. Send HELP for commands, AUTO for the automatic test."));
  printConfig();
  resetStats();
}

/* LOOP ----------------------------------------------------------------------*/

void loop() {
  handleSerialCommands();

  if (cfg.mode == MODE_RAW) {
    monitorIdleBus();
  }

  if (cfg.running && millis() - lastCycleMs >= cfg.cycleMs) {
    lastCycleMs = millis();
    runCycle();
    printEvents();
  }

  if (cfg.reportS && millis() - lastReportMs >= cfg.reportS * 1000UL) {
    lastReportMs = millis();
    if (cfg.running) {
      printStats();
    }
  }
}

/* POLL CYCLE ----------------------------------------------------------------*/

/* One poll cycle: same requests as the main program, VFD 1 then VFD 2 (or
   reversed with ORDER 21). Main program: read NM1 (ETA, RFR), and only if
   that succeeded read NC1 (CMD, LFR). */
void runCycle() {
  for (uint8_t k = 0; k < 2; k++) {
    uint8_t v = cfg.order21 ? 1 - k : k;
    if (!(cfg.targets & (1 << v))) {
      continue;
    }
    if (cfg.pattern == PATTERN_SIMPLE) {
      transact(v, 3, REG_ETA, 1, true);
    } else if (transact(v, 3, REG_NM1, 2, true) == R_OK) {
      transact(v, 3, REG_NC1, 2, true);
    }
    if (cfg.writes) {
      transact(v, 6, REG_LFR, 0, true);
      transact(v, 6, REG_CMD, CMD_STOP, true);
    }
  }
}

/* MODBUS TRANSACTION --------------------------------------------------------*/

/* One Modbus request/answer. Function 3: qtyOrVal = number of registers
   (max 4). Function 6: qtyOrVal = value. */
Result transact(uint8_t v, uint8_t func, uint16_t reg, uint16_t qtyOrVal, bool record) {
  txn.vfd = v;
  txn.addr = vfdAddress[v];
  txn.func = func;
  txn.reg = reg;
  txn.qtyOrVal = qtyOrVal;
  return transactTxn(cfg.mode, record);
}

Result transactTxn(Mode mode, bool record) {
  // Optional bus silence before the request (Modbus RTU requires at least
  // 3.5 characters, about 2 ms at 19200 baud, between frames)
  if (cfg.gapMs) {
    unsigned long gapUs = cfg.gapMs * 1000UL;
    while (micros() - lastBusUs < gapUs) {
    }
  }

  // Bytes already in the receive buffer before the request. RAW removes them.
  // LIB leaves them, like the main program (the library would read them as
  // the start of the answer).
  txn.staleN = 0;
  int avail = RS485.available();
  if (avail > 0) {
    txn.staleN = (avail > 255) ? 255 : avail;
    if (mode == MODE_RAW) {
      uint8_t i = 0;
      while (RS485.available()) {
        uint8_t b = RS485.read();
        if (i < sizeof(txn.stale)) {
          txn.stale[i++] = b;
        }
      }
    }
  }

  txn.exc = 0;
  txn.libErr = NULL;
  txn.turnValid = false;
  txn.n = 0;
  if (mode == MODE_RAW) {
    rawTransact();
  } else {
    libTransact();
  }
  lastBusUs = micros();

  if (record) {
    recordTxn(mode);
  }
  return txn.result;
}

/* Transaction using the ArduinoModbus library, as in the main program */
void libTransact() {
  errno = 0;
  bool ok;
  unsigned long t0 = micros();
  if (txn.func == 3) {
    ModbusRTUClient.requestFrom(txn.addr, HOLDING_REGISTERS, txn.reg, txn.qtyOrVal);
    ok = (ModbusRTUClient.available() == (int)txn.qtyOrVal);
    if (ok) {
      for (uint8_t i = 0; i < txn.qtyOrVal && i < 4; i++) {
        txn.values[i] = (uint16_t)ModbusRTUClient.read();
      }
    }
    while (ModbusRTUClient.available()) {
      ModbusRTUClient.read();
    }
  } else {
    ok = ModbusRTUClient.holdingRegisterWrite(txn.addr, txn.reg, txn.qtyOrVal);
  }
  txn.durUs = micros() - t0;

  if (ok) {
    txn.result = R_OK;
    return;
  }
  const char *e = ModbusRTUClient.lastError();
  txn.libErr = e;
  if (e == NULL) {
    txn.result = R_OTHER;
  } else if (!strcmp(e, "Connection timed out")) {
    txn.result = R_TIMEOUT;
  } else if (!strcmp(e, "Invalid CRC")) {
    txn.result = R_CRC;
  } else if (!strcmp(e, "Response not from requested slave")) {
    txn.result = R_WRONG_SLAVE;
  } else if (!strcmp(e, "Invalid data") || !strcmp(e, "Too many data")) {
    txn.result = R_BAD_DATA;
  } else if (!strncmp(e, "Illegal", 7) || !strncmp(e, "Slave device", 12) ||
             !strcmp(e, "Acknowledge") || !strcmp(e, "Negative acknowledge") ||
             !strcmp(e, "Memory parity error") || !strncmp(e, "Gateway", 7) ||
             !strncmp(e, "Target device", 13)) {
    txn.result = R_EXCEPTION;
  } else {
    txn.result = R_OTHER;
  }
}

/* Transaction using own Modbus RTU implementation, recording all bytes */
void rawTransact() {
  uint8_t req[8];
  req[0] = txn.addr;
  req[1] = txn.func;
  req[2] = txn.reg >> 8;
  req[3] = txn.reg & 0xFF;
  req[4] = txn.qtyOrVal >> 8;
  req[5] = txn.qtyOrVal & 0xFF;
  uint16_t crc = crc16(req, 6);
  req[6] = crc & 0xFF;  // CRC low byte first
  req[7] = crc >> 8;
  uint8_t expected = (txn.func == 3) ? 5 + 2 * txn.qtyOrVal : 8;
  txn.expected = expected;

  // Send request, same sequence as the library
  unsigned long tStart = micros();
  RS485.noReceive();
  RS485.beginTransmission();
  RS485.write(req, 8);
  RS485.endTransmission();  // Waits until the last byte is sent
  RS485.receive();
  unsigned long tTx = micros();

  // Receive answer
  unsigned long timeoutUs = cfg.timeoutMs * 1000UL;
  unsigned long deadlineUs = timeoutUs + cfg.lateMs * 1000UL;  // Late listening
  unsigned long tFirst = 0, tLast = 0;
  while (true) {
    if (RS485.available()) {
      uint8_t b = RS485.read();
      unsigned long now = micros();
      if (txn.n == 0) {
        tFirst = now;
      }
      tLast = now;
      if (txn.n < RX_MAX) {
        txn.rx[txn.n] = b;
      }
      if (txn.n < 255) {
        txn.n++;
      }
      if (txn.n == 2 && (txn.rx[1] & 0x80)) {
        expected = 5;  // Exception answer
      }
      if (txn.n >= expected) {
        break;
      }
    } else {
      unsigned long now = micros();
      if (txn.n == 0 && now - tTx > deadlineUs) {
        break;  // No answer
      }
      if (txn.n > 0 && now - tLast > FRAME_END_SILENCE_US) {
        break;  // Answer ended before expected length
      }
    }
  }
  txn.expected = expected;
  txn.durUs = (txn.n ? tLast : micros()) - tStart;

  if (txn.n == 0) {
    txn.result = R_TIMEOUT;
    return;
  }
  txn.turnValid = true;
  txn.turnUs = tFirst - tTx;
  txn.result = checkAnswer(req, expected);
  if (txn.result == R_OK && txn.turnUs > timeoutUs) {
    txn.result = R_LATE;  // Valid, but the library would have given up
  }
}

Result checkAnswer(const uint8_t *req, uint8_t expected) {
  if (txn.func == 3 && txn.n >= 6 && memcmp(txn.rx, req, 6) == 0) {
    return R_ECHO;
  }
  if (txn.n < expected || expected > RX_MAX) {
    return R_PARTIAL;
  }
  uint16_t crc = crc16(txn.rx, expected - 2);
  uint16_t crcReceived = txn.rx[expected - 2] | ((uint16_t)txn.rx[expected - 1] << 8);
  if (crcReceived != crc) {
    return R_CRC;
  }
  if (txn.rx[0] != txn.addr) {
    return R_WRONG_SLAVE;
  }
  if (txn.rx[1] == (txn.func | 0x80)) {
    txn.exc = txn.rx[2];
    return R_EXCEPTION;
  }
  if (txn.rx[1] != txn.func) {
    return R_BAD_DATA;
  }
  if (txn.func == 3) {
    if (txn.rx[2] != 2 * txn.qtyOrVal) {
      return R_BAD_DATA;
    }
    for (uint8_t i = 0; i < txn.qtyOrVal && i < 4; i++) {
      txn.values[i] = ((uint16_t)txn.rx[3 + 2 * i] << 8) | txn.rx[4 + 2 * i];
    }
  } else if (memcmp(txn.rx, req, 6) != 0) {
    return R_BAD_DATA;
  }
  return R_OK;
}

uint16_t crc16(const uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

/* RAW: bytes received while no transaction is running. Only the master
   (PLC) starts communication, so these are noise or late answers. */
void monitorIdleBus() {
  while (RS485.available()) {
    uint8_t b = RS485.read();
    if (unsolNew < sizeof(unsolBuf)) {
      unsolBuf[unsolNew] = b;
    }
    if (unsolNew < 255) {
      unsolNew++;
    }
    unsolicitedBytes++;
  }
}

/* STATISTICS ----------------------------------------------------------------*/

void resetStats() {
  memset(stats, 0, sizeof(stats));
  for (uint8_t v = 0; v < 2; v++) {
    stats[v].durMin = 0xFFFFFFFF;
    stats[v].turnMin = 0xFFFFFFFF;
  }
  unsolicitedBytes = 0;
  unsolNew = 0;
  evCount = 0;
  evDropped = 0;
  statsResetMs = millis();
  lastReportMs = millis();
}

void recordTxn(Mode mode) {
  Stats &s = stats[txn.vfd];
  s.sent++;
  s.count[txn.result]++;
  if (txn.staleN) {
    s.staleEvents++;
    s.staleBytes += txn.staleN;
  }
  if (txn.result == R_OK) {
    s.consec = 0;
  } else {
    if (s.consec < 0xFFFF) {
      s.consec++;
    }
    if (s.consec > s.maxConsec) {
      s.maxConsec = s.consec;
    }
    if (s.consec == FAULT_CONSECUTIVE) {
      s.faultEvents++;
    }
  }
  if (txn.durUs < s.durMin) s.durMin = txn.durUs;
  if (txn.durUs > s.durMax) s.durMax = txn.durUs;
  s.durSum += txn.durUs;
  uint8_t bin = 0;
  while (bin < HIST_BINS - 1 && txn.durUs >= histLimitMs[bin] * 1000UL) {
    bin++;
  }
  s.hist[bin]++;
  if (txn.turnValid) {
    if (txn.turnUs < s.turnMin) s.turnMin = txn.turnUs;
    if (txn.turnUs > s.turnMax) s.turnMax = txn.turnUs;
    s.turnSum += txn.turnUs;
    s.turnN++;
  }

  // Queue event for printing after the cycle (printing during the cycle
  // would change the bus timing)
  bool show = (cfg.verbose >= 2) ||
              (cfg.verbose >= 1 && (txn.result != R_OK || txn.staleN));
  if (!show) {
    return;
  }
  if (evCount >= EVT_QUEUE) {
    evDropped++;
    return;
  }
  Event &e = evq[evCount++];
  e.ms = millis();
  e.vfd = txn.vfd;
  e.func = txn.func;
  e.reg = txn.reg;
  e.qtyOrVal = txn.qtyOrVal;
  e.result = txn.result;
  e.exc = txn.exc;
  e.mode = mode;
  e.libErr = txn.libErr;
  e.durUs = txn.durUs;
  e.turnUs = txn.turnUs;
  e.turnValid = txn.turnValid;
  e.n = txn.n;
  e.expected = txn.expected;
  memcpy(e.rx, txn.rx, min((uint8_t)EVT_RX_MAX, (uint8_t)min(txn.n, (uint8_t)RX_MAX)));
  e.staleN = txn.staleN;
  memcpy(e.stale, txn.stale, sizeof(e.stale));
}

uint32_t errorCount(const Stats &s) {
  return s.sent - s.count[R_OK];
}

void makeSummary(const Stats &s, Summary &r) {
  r.sent = s.sent;
  r.ok = s.count[R_OK];
  r.timeout = s.count[R_TIMEOUT];
  r.late = s.count[R_LATE];
  r.corrupt = s.count[R_PARTIAL] + s.count[R_CRC] + s.count[R_ECHO];
  r.wrongSlave = s.count[R_WRONG_SLAVE];
  r.other = s.count[R_EXCEPTION] + s.count[R_BAD_DATA] + s.count[R_OTHER];
  r.staleEvents = s.staleEvents;
  r.faultEvents = s.faultEvents;
  r.maxConsec = s.maxConsec;
  r.avgDurUs = s.sent ? (uint32_t)(s.durSum / s.sent) : 0;
  r.maxDurUs = s.durMax;
}

/* PRINTING ------------------------------------------------------------------*/

const __FlashStringHelper *resultName(Result r) {
  switch (r) {
    case R_OK:          return F("OK");
    case R_TIMEOUT:     return F("TIMEOUT (no answer)");
    case R_LATE:        return F("LATE (valid answer after timeout)");
    case R_PARTIAL:     return F("PARTIAL (bytes missing)");
    case R_CRC:         return F("CRC ERROR");
    case R_WRONG_SLAVE: return F("WRONG ADDRESS in answer");
    case R_EXCEPTION:   return F("EXCEPTION");
    case R_BAD_DATA:    return F("BAD DATA (function/length/content)");
    case R_ECHO:        return F("ECHO (own request received)");
    case R_OTHER:       return F("OTHER ERROR");
    default:            return F("?");
  }
}

void printHex2(uint8_t b) {
  if (b < 0x10) {
    Serial.print('0');
  }
  Serial.print(b, HEX);
}

void printBytes(const uint8_t *b, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    if (i) {
      Serial.print(' ');
    }
    printHex2(b[i]);
  }
}

void printMs(uint32_t us) {
  Serial.print(us / 1000.0, 1);
}

void printTimestamp(uint32_t ms) {
  Serial.print('[');
  Serial.print(ms / 1000);
  Serial.print('.');
  uint16_t frac = ms % 1000;
  if (frac < 100) Serial.print('0');
  if (frac < 10) Serial.print('0');
  Serial.print(frac);
  Serial.print(F(" s] "));
}

void printPercent(uint32_t part, uint32_t total) {
  if (!total) {
    Serial.print('-');
    return;
  }
  Serial.print(100.0 * part / total, 3);
  Serial.print('%');
}

void printEvents() {
  for (uint8_t i = 0; i < evCount; i++) {
    Event &e = evq[i];
    printTimestamp(e.ms);
    Serial.print(F("VFD "));
    Serial.print(e.vfd + 1);
    Serial.print(e.mode == MODE_RAW ? F(" RAW ") : F(" LIB "));
    if (e.func == 3) {
      Serial.print(F("read "));
      Serial.print(e.reg);
      Serial.print('x');
      Serial.print(e.qtyOrVal);
    } else {
      Serial.print(F("write "));
      Serial.print(e.reg);
      Serial.print('=');
      Serial.print(e.qtyOrVal);
    }
    Serial.print(F(": "));
    Serial.print(resultName(e.result));
    if (e.result == R_EXCEPTION && e.mode == MODE_RAW) {
      Serial.print(F(" code "));
      Serial.print(e.exc);
    }
    Serial.print(F(", time "));
    printMs(e.durUs);
    Serial.print(F(" ms"));
    if (e.turnValid) {
      Serial.print(F(", VFD response delay "));
      printMs(e.turnUs);
      Serial.print(F(" ms"));
    }
    if (e.libErr) {
      Serial.print(F(", library: \""));
      Serial.print(e.libErr);
      Serial.print('"');
    }
    Serial.println();
    if (e.mode == MODE_RAW && (e.result != R_OK || cfg.verbose >= 2)) {
      Serial.print(F("    received "));
      Serial.print(e.n);
      Serial.print('/');
      Serial.print(e.expected);
      Serial.print(F(" bytes: "));
      printBytes(e.rx, min(e.n, (uint8_t)EVT_RX_MAX));
      if (e.n > EVT_RX_MAX) {
        Serial.print(F(" ..."));
      }
      Serial.println();
    }
    if (e.staleN) {
      Serial.print(F("    "));
      Serial.print(e.staleN);
      Serial.print(F(" leftover byte(s) in receive buffer before request"));
      if (e.mode == MODE_RAW) {
        Serial.print(F(" (removed): "));
        printBytes(e.stale, min(e.staleN, (uint8_t)sizeof(e.stale)));
      } else {
        Serial.print(F(" (library reads them as answer)"));
      }
      Serial.println();
    }
  }
  evCount = 0;

  if (unsolNew && cfg.verbose >= 1) {
    printTimestamp(millis());
    Serial.print(F("BUS: "));
    Serial.print(unsolNew);
    Serial.print(F(" byte(s) received while bus idle (noise or late answer): "));
    printBytes(unsolBuf, min(unsolNew, (uint8_t)sizeof(unsolBuf)));
    Serial.println();
  }
  unsolNew = 0;
}

void printConfig() {
  Serial.print(F("Config: mode "));
  Serial.print(cfg.mode == MODE_RAW ? F("RAW") : F("LIB"));
  Serial.print(F(", VFD "));
  Serial.print(cfg.targets == 3 ? F("BOTH") : (cfg.targets == 1 ? F("1") : F("2")));
  Serial.print(F(", order "));
  Serial.print(cfg.order21 ? F("2-1") : F("1-2"));
  Serial.print(F(", pattern "));
  Serial.print(cfg.pattern == PATTERN_MAIN ? F("MAIN") : F("SIMPLE"));
  Serial.print(F(", writes "));
  Serial.print(cfg.writes ? F("ON") : F("OFF"));
  Serial.print(F(", gap "));
  Serial.print(cfg.gapMs);
  Serial.print(F(" ms, timeout "));
  Serial.print(cfg.timeoutMs);
  Serial.print(F(" ms, cycle "));
  Serial.print(cfg.cycleMs);
  Serial.print(F(" ms, late listen "));
  Serial.print(cfg.lateMs);
  Serial.print(F(" ms, verbose "));
  Serial.print(cfg.verbose);
  Serial.print(F(", report "));
  Serial.print(cfg.reportS);
  Serial.print(F(" s, "));
  Serial.println(cfg.running ? F("RUNNING") : F("PAUSED"));
}

void printStats() {
  Serial.println();
  Serial.print(F("=== Statistics, "));
  Serial.print((millis() - statsResetMs) / 1000);
  Serial.println(F(" s since reset ==="));
  printConfig();
  for (uint8_t v = 0; v < 2; v++) {
    const Stats &s = stats[v];
    Serial.print(F("VFD "));
    Serial.print(v + 1);
    Serial.print(F(" (address "));
    Serial.print(vfdAddress[v]);
    Serial.print(F("): sent "));
    Serial.print(s.sent);
    Serial.print(F(", OK "));
    Serial.print(s.count[R_OK]);
    Serial.print(F(", errors "));
    Serial.print(errorCount(s));
    Serial.print(F(" ("));
    printPercent(errorCount(s), s.sent);
    Serial.println(')');
    if (!s.sent) {
      continue;
    }
    for (uint8_t r = 1; r < R_COUNT; r++) {
      if (s.count[r]) {
        Serial.print(F("    "));
        Serial.print(resultName((Result)r));
        Serial.print(F(": "));
        Serial.println(s.count[r]);
      }
    }
    Serial.print(F("  Leftover bytes before request: "));
    Serial.print(s.staleEvents);
    Serial.print(F(" times, "));
    Serial.print(s.staleBytes);
    Serial.println(F(" bytes"));
    Serial.print(F("  Consecutive errors: max "));
    Serial.print(s.maxConsec);
    Serial.print(F(", reached "));
    Serial.print(FAULT_CONSECUTIVE);
    Serial.print(F(" in a row (= FAULT_VFD_COMM in main program): "));
    Serial.print(s.faultEvents);
    Serial.println(F(" times"));
    Serial.print(F("  Transaction time [ms]: min "));
    printMs(s.durMin);
    Serial.print(F(", avg "));
    printMs((uint32_t)(s.durSum / s.sent));
    Serial.print(F(", max "));
    printMs(s.durMax);
    Serial.println();
    if (s.turnN) {
      Serial.print(F("  VFD response delay [ms] (RAW): min "));
      printMs(s.turnMin);
      Serial.print(F(", avg "));
      printMs((uint32_t)(s.turnSum / s.turnN));
      Serial.print(F(", max "));
      printMs(s.turnMax);
      Serial.println();
    }
    Serial.print(F("  Time histogram [ms]:"));
    for (uint8_t b = 0; b < HIST_BINS; b++) {
      Serial.print(' ');
      if (b == 0) {
        Serial.print('<');
        Serial.print(histLimitMs[0]);
      } else if (b == HIST_BINS - 1) {
        Serial.print('>');
        Serial.print(histLimitMs[HIST_BINS - 2]);
      } else {
        Serial.print(histLimitMs[b - 1]);
        Serial.print('-');
        Serial.print(histLimitMs[b]);
      }
      Serial.print(':');
      Serial.print(s.hist[b]);
    }
    Serial.println();
  }
  Serial.print(F("Bytes received while bus idle (RAW only): "));
  Serial.println(unsolicitedBytes);
  if (evDropped) {
    Serial.print(F("Event messages not printed (queue full): "));
    Serial.println(evDropped);
  }
  Serial.println();
}

/* AUTOMATIC TEST ------------------------------------------------------------*/

const __FlashStringHelper *autoStepName(uint8_t i) {
  switch (i) {
    case 0: return F("LIB, no gap, both VFDs (= main program)");
    case 1: return F("RAW, no gap, both VFDs");
    case 2: return F("RAW, 5 ms gap, both VFDs");
    case 3: return F("RAW, 5 ms gap, VFD 1 only");
    case 4: return F("RAW, 5 ms gap, VFD 2 only");
    case 5: return F("LIB, 5 ms gap, both VFDs");
    default: return F("?");
  }
}

void runAuto(uint32_t cycles) {
  Settings saved = cfg;
  Serial.println();
  Serial.print(F("=== AUTO test: "));
  Serial.print(AUTO_STEPS);
  Serial.print(F(" steps, "));
  Serial.print(cycles);
  Serial.println(F(" cycles each. Send any character to abort. ==="));
  memset(autoRes, 0, sizeof(autoRes));
  memset(autoUnsol, 0, sizeof(autoUnsol));
  bool aborted = false;
  uint8_t stepsDone = 0;

  for (uint8_t st = 0; st < AUTO_STEPS && !aborted; st++) {
    cfg.mode = autoSteps[st].mode;
    cfg.gapMs = autoSteps[st].gapMs;
    cfg.targets = autoSteps[st].targets;
    cfg.order21 = false;
    resetStats();
    Serial.print(F("Step "));
    Serial.print(st + 1);
    Serial.print(F(": "));
    Serial.println(autoStepName(st));

    unsigned long lastMs = millis() - cfg.cycleMs;
    for (uint32_t c = 0; c < cycles; c++) {
      while (millis() - lastMs < cfg.cycleMs) {
        if (cfg.mode == MODE_RAW) {
          monitorIdleBus();
        }
      }
      lastMs = millis();
      if (Serial.available()) {
        aborted = true;
        break;
      }
      runCycle();
      printEvents();
    }
    if (cfg.mode == MODE_RAW) {
      monitorIdleBus();
      printEvents();
    }
    for (uint8_t v = 0; v < 2; v++) {
      makeSummary(stats[v], autoRes[st][v]);
    }
    autoUnsol[st] = unsolicitedBytes;
    stepsDone = st + 1;
    Serial.print(F("  Errors: VFD 1 "));
    Serial.print(errorCount(stats[0]));
    Serial.print('/');
    Serial.print(stats[0].sent);
    Serial.print(F(", VFD 2 "));
    Serial.print(errorCount(stats[1]));
    Serial.print('/');
    Serial.println(stats[1].sent);
  }
  while (Serial.available()) {
    Serial.read();
  }
  if (aborted) {
    Serial.println(F("AUTO test aborted."));
  }

  printAutoTable(stepsDone);
  if (!aborted) {
    printObservations();
  }
  cfg = saved;
  resetStats();
  lastCycleMs = millis();
}

void printAutoTable(uint8_t steps) {
  Serial.println();
  Serial.println(F("=== AUTO test summary ==="));
  Serial.println(F("Columns: sent, OK, error %, timeout, late, corrupt (partial/CRC/echo),"));
  Serial.println(F("wrong address, other, leftover-before-request, max errors in a row,"));
  Serial.println(F("times 5 in a row (main program FAULT), avg/max transaction time [ms]."));
  for (uint8_t st = 0; st < steps; st++) {
    Serial.print(F("Step "));
    Serial.print(st + 1);
    Serial.print(F(": "));
    Serial.print(autoStepName(st));
    if (autoSteps[st].mode == MODE_RAW) {
      Serial.print(F(", bus idle bytes "));
      Serial.print(autoUnsol[st]);
    }
    Serial.println();
    for (uint8_t v = 0; v < 2; v++) {
      const Summary &r = autoRes[st][v];
      if (!r.sent) {
        continue;
      }
      Serial.print(F("  VFD "));
      Serial.print(v + 1);
      Serial.print(F(": "));
      Serial.print(r.sent);
      Serial.print(F(", OK "));
      Serial.print(r.ok);
      Serial.print(F(", err "));
      printPercent(r.sent - r.ok, r.sent);
      Serial.print(F(", tmo "));
      Serial.print(r.timeout);
      Serial.print(F(", late "));
      Serial.print(r.late);
      Serial.print(F(", corrupt "));
      Serial.print(r.corrupt);
      Serial.print(F(", addr "));
      Serial.print(r.wrongSlave);
      Serial.print(F(", other "));
      Serial.print(r.other);
      Serial.print(F(", leftover "));
      Serial.print(r.staleEvents);
      Serial.print(F(", row "));
      Serial.print(r.maxConsec);
      Serial.print(F(", fault "));
      Serial.print(r.faultEvents);
      Serial.print(F(", t "));
      printMs(r.avgDurUs);
      Serial.print('/');
      printMs(r.maxDurUs);
      Serial.println();
    }
  }
}

uint32_t autoErr(uint8_t st, uint8_t v) {
  return autoRes[st][v].sent - autoRes[st][v].ok;
}

/* Observations from the AUTO results. These are indications, not proof:
   intermittent errors need enough cycles to show up. */
void printObservations() {
  Serial.println();
  Serial.println(F("=== Observations ==="));
  uint32_t total = 0, sent = 0;
  for (uint8_t st = 0; st < AUTO_STEPS; st++) {
    for (uint8_t v = 0; v < 2; v++) {
      total += autoErr(st, v);
      sent += autoRes[st][v].sent;
    }
  }
  if (total == 0) {
    Serial.print(F("- No errors in "));
    Serial.print(sent);
    Serial.println(F(" transactions. Bus and VFDs communicate correctly with motors"));
    Serial.println(F("  stopped. If errors occur with the main program, test longer (AUTO 5000),"));
    Serial.println(F("  and test while the gates move (motor EMC): run the main program with"));
    Serial.println(F("  DEBUG enabled and look for \"Failed to read Modbus RTU register\"."));
  }

  for (uint8_t v = 0; v < 2; v++) {
    uint8_t alone = (v == 0) ? 3 : 4;
    uint32_t noGap = autoErr(0, v) + autoErr(1, v);
    uint32_t withGap = autoErr(2, v) + autoErr(5, v) + autoErr(alone, v);
    uint32_t late = 0, corrupt = 0, timeout = 0, wrongSlave = 0, staleLib = 0;
    for (uint8_t st = 0; st < AUTO_STEPS; st++) {
      late += autoRes[st][v].late;
      corrupt += autoRes[st][v].corrupt;
      timeout += autoRes[st][v].timeout;
      wrongSlave += autoRes[st][v].wrongSlave;
      if (autoSteps[st].mode == MODE_LIB) {
        staleLib += autoRes[st][v].staleEvents;
      }
    }
    if (noGap + withGap == 0) {
      continue;
    }
    Serial.print(F("VFD "));
    Serial.print(v + 1);
    Serial.println(':');
    if (noGap > 0 && withGap == 0) {
      Serial.println(F("- Errors only without pause between requests. Indicates a frame timing"));
      Serial.println(F("  problem (software): a request sent too soon after the previous frame is"));
      Serial.println(F("  not recognized by the VFD. Fix in main program: wait at least 5 ms of"));
      Serial.println(F("  bus silence before each Modbus RTU request."));
    }
    if (autoErr(0, v) > 0 && autoErr(1, v) == 0 && autoErr(2, v) == 0) {
      Serial.println(F("- Errors in LIB but not in RAW mode. Indicates library/software behaviour"));
      Serial.println(F("  rather than the bus."));
    }
    if (autoErr(alone, v) > 0) {
      Serial.println(F("- Errors also when this VFD is polled alone with pause. Not caused by"));
      Serial.println(F("  traffic to the other VFD. Check this VFD's bus connection (connector,"));
      Serial.println(F("  terminals, termination, shield), its Modbus settings and nearby EMC."));
    }
    if (corrupt > 0) {
      Serial.print(F("- "));
      Serial.print(corrupt);
      Serial.println(F(" corrupted answers (bytes missing or CRC error). Missing bytes are"));
      Serial.println(F("  usually bytes the UART discarded due to parity/framing errors. Indicates"));
      Serial.println(F("  a physical layer problem: termination (120 Ohm at both bus ends only),"));
      Serial.println(F("  bias, shield / 0 V connection, cable routing near motor cables."));
    }
    if (timeout > 0) {
      Serial.print(F("- "));
      Serial.print(timeout);
      Serial.println(F(" requests without any answer byte. The VFD did not accept the request"));
      Serial.println(F("  (corrupted on the way to the VFD, or sent too soon after another frame)."));
      Serial.println(F("  Check the CRC error counter on the VFD display."));
    }
    if (late > 0) {
      Serial.print(F("- "));
      Serial.print(late);
      Serial.println(F(" valid answers after the timeout. The VFD answers slower than the"));
      Serial.println(F("  timeout allows: increase the Modbus RTU timeout in the main program."));
    }
    if (wrongSlave > 0) {
      Serial.println(F("- Answers with wrong address: leftover bytes of an earlier (late) answer,"));
      Serial.println(F("  or two devices with the same address (run SCAN)."));
    }
    if (staleLib > 0) {
      Serial.println(F("- Leftover bytes before requests in LIB mode. The library does not clear"));
      Serial.println(F("  the receive buffer after a timeout, so a late answer is read as the"));
      Serial.println(F("  answer to the next request (possibly to the other VFD)."));
    }
  }
  uint32_t unsol = autoUnsol[1] + autoUnsol[2] + autoUnsol[3] + autoUnsol[4];
  if (unsol > 0) {
    Serial.print(F("- "));
    Serial.print(unsol);
    Serial.println(F(" bytes received while the bus was idle (RAW steps). Noise on the bus"));
    Serial.println(F("  or late answers. Check termination, bias and shielding. Use LISTEN."));
  }
  Serial.println();
}

/* DIAGNOSTIC COMMANDS -------------------------------------------------------*/

/* Single RAW read, not counted in statistics */
bool rawRead(uint8_t addr, uint16_t reg, uint8_t qty) {
  txn.vfd = 0;
  txn.addr = addr;
  txn.func = 3;
  txn.reg = reg;
  txn.qtyOrVal = qty;
  return transactTxn(MODE_RAW, false) == R_OK;
}

void printRawFailure() {
  Serial.print(resultName(txn.result));
  if (txn.result == R_EXCEPTION) {
    Serial.print(F(" code "));
    Serial.print(txn.exc);
  }
  if (txn.n) {
    Serial.print(F(", received: "));
    printBytes(txn.rx, min(txn.n, (uint8_t)RX_MAX));
  }
}

void printRegLine(const __FlashStringHelper *name, uint16_t reg, uint8_t qty, uint8_t addr) {
  Serial.print(F("  "));
  Serial.print(name);
  Serial.print(F(" ("));
  Serial.print(reg);
  Serial.print(F("): "));
  if (rawRead(addr, reg, qty)) {
    for (uint8_t i = 0; i < qty; i++) {
      if (i) {
        Serial.print(F(", "));
      }
      Serial.print(txn.values[i]);
      Serial.print(F(" (0x"));
      Serial.print(txn.values[i], HEX);
      Serial.print(')');
    }
    Serial.print(F(", response delay "));
    printMs(txn.turnUs);
    Serial.print(F(" ms"));
  } else {
    printRawFailure();
  }
  Serial.println();
}

void cmdInfo() {
  for (uint8_t v = 0; v < 2; v++) {
    uint8_t addr = vfdAddress[v];
    Serial.print(F("VFD "));
    Serial.print(v + 1);
    Serial.print(F(" (address "));
    Serial.print(addr);
    Serial.println(F("):"));
    printRegLine(F("Drive rating NCV"), REG_NCV, 1, addr);
    printRegLine(F("Status ETA, frequency RFR"), REG_ETA, 2, addr);
    if (txn.result == R_OK) {
      uint16_t eta = txn.values[0];
      Serial.print(F("    ETA: ready "));
      Serial.print((eta >> 1) & 1);
      Serial.print(F(", running "));
      Serial.print((eta >> 2) & 1);
      Serial.print(F(", fault "));
      Serial.print((eta >> 3) & 1);
      Serial.print(F(", voltage "));
      Serial.print((eta >> 4) & 1);
      Serial.print(F(", quick stop not active "));
      Serial.print((eta >> 5) & 1);
      Serial.print(F(", switch on disabled "));
      Serial.print((eta >> 6) & 1);
      Serial.print(F(", warning "));
      Serial.print((eta >> 7) & 1);
      Serial.print(F(", remote "));
      Serial.println((eta >> 9) & 1);
    }
    printRegLine(F("Last fault LFt"), REG_LFT, 1, addr);
    printRegLine(F("Scanner in NM1..2 (main: ETA, RFR)"), REG_NM1, 2, addr);
    printRegLine(F("Scanner out NC1..2 (main: CMD, LFR)"), REG_NC1, 2, addr);
  }
  Serial.println(F("LFt: see ATV320 manual for fault codes (0 = no fault)."));
}

void cmdScan(uint8_t maxAddr) {
  Serial.print(F("Scanning addresses 1 to "));
  Serial.print(maxAddr);
  Serial.println(F(" (read ETA 3201)..."));
  uint8_t found = 0;
  for (uint8_t a = 1; a <= maxAddr; a++) {
    rawRead(a, REG_ETA, 1);
    if (txn.result != R_TIMEOUT) {
      Serial.print(F("  Address "));
      Serial.print(a);
      Serial.print(F(": "));
      if (txn.result == R_OK) {
        Serial.print(F("answers, ETA 0x"));
        Serial.print(txn.values[0], HEX);
        found++;
      } else {
        printRawFailure();
        Serial.print(F(" (corrupted answer: noise, or two devices answering)"));
      }
      Serial.println();
    }
  }
  Serial.print(F("Scan done, "));
  Serial.print(found);
  Serial.println(F(" device(s) answered. Expected: addresses 1 and 2 only."));
}

void cmdRead(uint8_t addr, uint16_t reg, uint8_t qty) {
  Serial.print(F("Address "));
  Serial.print(addr);
  Serial.print(':');
  Serial.println();
  printRegLine(F("Read"), reg, qty, addr);
}

/* Listen passively: only the PLC starts communication, so with polling
   paused nothing should be received. Prints received frames. */
void cmdListen(uint16_t seconds) {
  Serial.print(F("Listening for "));
  Serial.print(seconds);
  Serial.println(F(" s (polling paused, nothing should be received)..."));
  while (RS485.available()) {
    RS485.read();
  }
  unsigned long start = millis();
  unsigned long tLast = 0;
  uint32_t bytes = 0, frames = 0;
  uint8_t lineLen = 0;
  bool inFrame = false;
  while (millis() - start < seconds * 1000UL) {
    if (RS485.available()) {
      uint8_t b = RS485.read();
      unsigned long now = micros();
      if (!inFrame) {
        frames++;
        printTimestamp(millis());
        Serial.print(F("frame: "));
        inFrame = true;
        lineLen = 0;
      } else if (lineLen >= 32) {
        Serial.print(F("\n    "));
        lineLen = 0;
      }
      printHex2(b);
      Serial.print(' ');
      lineLen++;
      bytes++;
      tLast = now;
    } else if (inFrame && micros() - tLast > FRAME_END_SILENCE_US) {
      Serial.println();
      inFrame = false;
    }
    if (Serial.available()) {
      break;
    }
  }
  if (inFrame) {
    Serial.println();
  }
  Serial.print(F("Listen done: "));
  Serial.print(bytes);
  Serial.print(F(" bytes in "));
  Serial.print(frames);
  Serial.println(F(" frames."));
  if (bytes) {
    Serial.println(F("Bytes on an idle bus: noise (missing termination/bias, EMC), or another"));
    Serial.println(F("device transmitting (second master, wrong VFD setting)."));
  }
}

void printHelp() {
  Serial.println(F("Commands (not case sensitive):"));
  Serial.println(F("  HELP              this list"));
  Serial.println(F("  RUN / PAUSE       start / pause continuous polling"));
  Serial.println(F("  STATS             print statistics"));
  Serial.println(F("  RESET             reset statistics"));
  Serial.println(F("  CONFIG            print settings"));
  Serial.println(F("  MODE LIB|RAW      LIB = ArduinoModbus as main program, RAW = byte level"));
  Serial.println(F("  VFD 1|2|BOTH      VFDs to poll"));
  Serial.println(F("  ORDER 12|21       poll order"));
  Serial.println(F("  PATTERN MAIN|SIMPLE  MAIN = NM1 + NC1 reads as main program, SIMPLE = ETA"));
  Serial.println(F("  WRITES ON|OFF     also write stop command (LFR = 0, CMD = 8)"));
  Serial.println(F("  GAP <ms>          bus silence before each request (0 = as main program)"));
  Serial.println(F("  TIMEOUT <ms>      response timeout (main program: 200)"));
  Serial.println(F("  CYCLE <ms>        time between poll cycles"));
  Serial.println(F("  LATE <ms>         RAW: keep listening after timeout to detect late answers"));
  Serial.println(F("  DEDELAY <pre> <post>  RS485 driver enable delays [us] (default 50 50)"));
  Serial.println(F("  VERBOSE 0|1|2     0 = statistics only, 1 = errors, 2 = every frame"));
  Serial.println(F("  REPORT <s>        periodic statistics interval, 0 = off"));
  Serial.println(F("  AUTO [cycles]     automatic comparison test (default 300 cycles/step)"));
  Serial.println(F("  INFO              read status, last fault and scanner registers of both VFDs"));
  Serial.println(F("  SCAN [max]        find answering addresses 1..max (default 10)"));
  Serial.println(F("  READ <addr> <reg> [n]  read n (1-4) holding registers"));
  Serial.println(F("  LISTEN <s>        listen to the idle bus"));
}

/* SERIAL COMMANDS -----------------------------------------------------------*/

void handleSerialCommands() {
  static char buf[SERIAL_CMD_MAX_LEN + 1];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (len) {
        buf[len] = '\0';
        executeCommand(buf);
        len = 0;
      }
    } else if (len < SERIAL_CMD_MAX_LEN) {
      buf[len++] = toupper(c);
    }
  }
}

long argNum(const char *s, long def) {
  return (s && *s) ? atol(s) : def;
}

void executeCommand(char *line) {
  char *cmd = strtok(line, " ");
  char *a1 = strtok(NULL, " ");
  char *a2 = strtok(NULL, " ");
  char *a3 = strtok(NULL, " ");
  if (!cmd) {
    return;
  }
  bool paused = false;  // Diagnostic commands run with polling paused

  if (!strcmp(cmd, "HELP")) {
    printHelp();
  } else if (!strcmp(cmd, "RUN")) {
    cfg.running = true;
    Serial.println(F("Polling running."));
  } else if (!strcmp(cmd, "PAUSE")) {
    cfg.running = false;
    Serial.println(F("Polling paused."));
  } else if (!strcmp(cmd, "STATS")) {
    printStats();
  } else if (!strcmp(cmd, "RESET")) {
    resetStats();
    Serial.println(F("Statistics reset."));
  } else if (!strcmp(cmd, "CONFIG")) {
    printConfig();
  } else if (!strcmp(cmd, "MODE") && a1) {
    if (!strcmp(a1, "RAW")) {
      cfg.mode = MODE_RAW;
    } else if (!strcmp(a1, "LIB")) {
      cfg.mode = MODE_LIB;
    }
    resetStats();
    printConfig();
  } else if (!strcmp(cmd, "VFD") && a1) {
    if (!strcmp(a1, "1")) {
      cfg.targets = 1;
    } else if (!strcmp(a1, "2")) {
      cfg.targets = 2;
    } else {
      cfg.targets = 3;
    }
    resetStats();
    printConfig();
  } else if (!strcmp(cmd, "ORDER") && a1) {
    cfg.order21 = !strcmp(a1, "21");
    resetStats();
    printConfig();
  } else if (!strcmp(cmd, "PATTERN") && a1) {
    cfg.pattern = !strcmp(a1, "SIMPLE") ? PATTERN_SIMPLE : PATTERN_MAIN;
    resetStats();
    printConfig();
  } else if (!strcmp(cmd, "WRITES") && a1) {
    cfg.writes = !strcmp(a1, "ON");
    resetStats();
    printConfig();
  } else if (!strcmp(cmd, "GAP")) {
    cfg.gapMs = constrain(argNum(a1, 0), 0, 1000);
    resetStats();
    printConfig();
  } else if (!strcmp(cmd, "TIMEOUT")) {
    cfg.timeoutMs = constrain(argNum(a1, DEFAULT_TIMEOUT), 10, 5000);
    ModbusRTUClient.setTimeout(cfg.timeoutMs);
    resetStats();
    printConfig();
  } else if (!strcmp(cmd, "CYCLE")) {
    cfg.cycleMs = constrain(argNum(a1, DEFAULT_CYCLE), 0, 60000);
    printConfig();
  } else if (!strcmp(cmd, "LATE")) {
    cfg.lateMs = constrain(argNum(a1, DEFAULT_LATE_LISTEN), 0, 5000);
    printConfig();
  } else if (!strcmp(cmd, "DEDELAY")) {
    int pre = constrain(argNum(a1, RS485_DEFAULT_PRE_DELAY), 0, 5000);
    int post = constrain(argNum(a2, RS485_DEFAULT_POST_DELAY), 0, 5000);
    RS485.setDelays(pre, post);
    resetStats();
    Serial.print(F("RS485 DE pre/post delay [us]: "));
    Serial.print(pre);
    Serial.print('/');
    Serial.println(post);
  } else if (!strcmp(cmd, "VERBOSE")) {
    cfg.verbose = constrain(argNum(a1, 1), 0, 2);
    printConfig();
  } else if (!strcmp(cmd, "REPORT")) {
    cfg.reportS = constrain(argNum(a1, DEFAULT_REPORT), 0, 3600);
    printConfig();
  } else if (!strcmp(cmd, "AUTO")) {
    runAuto(constrain(argNum(a1, AUTO_DEFAULT_CYCLES), 1, AUTO_MAX_CYCLES));
  } else if (!strcmp(cmd, "INFO")) {
    paused = true;
    cmdInfo();
  } else if (!strcmp(cmd, "SCAN")) {
    paused = true;
    cmdScan(constrain(argNum(a1, SCAN_DEFAULT_MAX), 1, 247));
  } else if (!strcmp(cmd, "READ") && a1 && a2) {
    paused = true;
    cmdRead(constrain(argNum(a1, 1), 1, 247), (uint16_t)argNum(a2, REG_ETA),
            constrain(argNum(a3, 1), 1, 4));
  } else if (!strcmp(cmd, "LISTEN")) {
    paused = true;
    cmdListen(constrain(argNum(a1, 10), 1, 3600));
  } else {
    Serial.print(F("Unknown command: "));
    Serial.println(cmd);
    printHelp();
  }

  if (paused) {
    // Diagnostic commands use the bus directly; restart cycle timing
    lastCycleMs = millis();
  }
}
