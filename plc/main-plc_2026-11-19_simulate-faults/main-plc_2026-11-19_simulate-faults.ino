/*
  Hardware: Controllino Maxi (SKU 100.100.00) https://www.controllino.com/product/controllino-maxi/
  Arduino setup:
  Refer to https://github.com/CONTROLLINO-PLC/CONTROLLINO_Library for
  installation of CONTROLLINO boards to Arduino IDE.
  Select correct target board [Tools->Board->Controllino MAXI] before uploading
  Arduino sketch to CONTROLLINO MAXI.

  Communication with VFD Schneider Electric ATV320.. using RS485 Modbus RTU (see
  page 17 in document Modbus Serial Link Manual NVE41308.01).
  All Modbus RTU requests go through rtuRequestFrom() / rtuWrite(), which
  keep at least RTU_FRAME_GAP_US of bus silence before each request (see
  RTU_FRAME_GAP_US for the reason).
  Communication with [control device] via Modbus TCP/IP communication. See
  external document "modbus-tcpip-server-setup" for details. Up to
  MODBUS_TCP_MAX_CLIENTS (3) clients can be connected at the same time, e.g.
  one gate control client and one monitoring client.

  VFD motor direction: forward (positive speed values) = gate direction: open
  VFD motor direction: reverse (negative speed values) = gate direction: close

  Gate position indication system consists of 3 proximity detectors per gate:
  fully OPEN/CLOSED, pre-OPEN, pre-CLOSED
  The gate has 4 different positions:
  fully OPEN, pre-OPEN, pre-CLOSED and fully CLOSED.

  GATE CONTROL PRINCIPLE
  Each gate has a TARGET (none/open/closed) set by Modbus TCP commands or the
  emergency force open key box. The state machine moves the gate towards its
  target. A gate cycle (open -> stay open -> close) is a target OPEN with
  cycleActive set. When the gate is fully open in a cycle it waits in
  STATE_STAY_OPEN until the stay open countdown (holding register 0) reaches 0,
  or (if "close after passage" is set) an object has passed the light barrier.
  The countdown is shared by both gates and follows these rules:
  - Cycle start request (either gate): raised to TIME_STAY_OPEN_DEFAULT if lower.
  - A gate enters STATE_STAY_OPEN: raised to TIME_STAY_OPEN_DEFAULT if lower.
  - At least one gate in STATE_STAY_OPEN: counts down 1 per second.
  - Last active cycle ends (no gate has cycleActive any more): set to 0, once.
  - The Modbus master can modify it at any time (limited to TIME_STAY_OPEN_MAX).
  There is no minimum stay open time: a gate in STATE_STAY_OPEN starts closing
  as soon as the countdown is 0, also if the Modbus master has lowered it.
  "Close after passage" (coil 6) is one setting shared by both gates. It is
  read every loop and takes effect directly, in any state.
  Faults are latched and must be reset via coil 7 (or the force open key box).
  After a successful fault reset the gate continues towards its target (and
  gate cycle) after the normal warning delay. The target is kept while in
  fault; commands received while in fault update it as usual.
  Coil 8 (warning, gate is obstructed) is set by the PLC every
  TIME_OBSTRUCTED_WARNING while a gate is continuously obstructed, see that
  setting. The Modbus client clears it after reading. The PLC clears it when
  the gate(s) that set it have reached their target.
  At PLC startup (e.g. after a power outage) both gates get target CLOSED.
  Gates do not move before the VFD has answered.
  At startup the PLC polls both VFDs until they answer (no fixed delay), see
  VFD_STARTUP_TIMEOUT. A VFD reporting a fault at startup (e.g. Modbus
  communication interruption while the PLC restarted) is reset automatically,
  see VFD_STARTUP_RESET_ATTEMPTS. Faults after startup are latched as usual.

  SERIAL DEBUG
  Debug messages are off by default. Connect USB cable (115200 baud) and send
  one of the following commands (not case sensitive):
    DEBUG       enable debug messages
    DEBUG 2     enable debug messages, including VFD status registers
    DEBUG OFF   disable debug messages

  FAULT SIMULATION (serial terminal)
  All faults can be simulated for test purposes. The simulated fault cause is
  injected where the real cause is detected, so the gate detects the fault,
  enters STATE_FAULT (Safe Torque Off etc.) and is reset (coil 7 or force
  open) exactly as for a real fault. Commands (not case sensitive):
    FAULT                 show state, fault code and simulated causes
    FAULT <gate> <code>   simulate fault <code> (1-7) on gate <gate> (1-2)
    FAULT <gate> CLEAR    remove simulated fault causes of one gate
    FAULT CLEAR           remove simulated fault causes of both gates
  Example: "FAULT 2 5" simulates invalid position sensors on gate 2.
  Fault code -> simulated cause:
    1 VFD fault: the VFD status (ETA bit 3) reports fault. Like a real VFD
      trip, it is cleared by the VFD fault reset command sent at fault reset.
    2 VFD communication: all Modbus RTU requests to the VFD fail (no bus
      traffic) until FAULT CLEAR. The fault is detected after
      VFD_COMM_ERROR_FAULT failed requests. NOTE: The real VFD sees no Modbus
      traffic either and may trip on its own communication timeout. That VFD
      fault is cleared by the following fault reset.
    5 Position sensors: the sensor combination is treated as invalid until
      FAULT CLEAR (fault after SENSOR_INVALID_TIME). Sensor readings are not
      changed.
    3 movement timeout, 4 pre-zone timeout, 6 motor not stopping,
    7 repeated safety edge: these faults are events without a lasting cause
      and are triggered once at the next loop, in any state except
      STATE_FAULT.
  A fault reset fails while a simulated cause (1, 2, 5) is still active.

  NOTE: Opening the USB serial port may reset the PLC (Arduino auto-reset via
  DTR). Gates stop and VFD Safe Torque Off is activated during the restart.

  TODO: What happens if ethernet goes down during operation? Test: power up without cable, then plug in cable and test polling modbus server.
  TODO: Setup VFD parameter ECFG (modbus register 7010) to ramp stop on fault or fast stop!!
  TODO: Verify PRE_ZONE_TIMEOUT and MOVEMENT_TIMEOUT against measured travel times.
*/

/* INCLUDE -------------------------------------------------------------------*/

#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <Controllino.h>  // Use CONTROLLINO_xx aliases

/* DEFINITIONS ---------------------------------------------------------------*/

// DIGITAL INPUT PINS
#define PIN_G1_POS_OPEN_CLOSED  CONTROLLINO_A0
#define PIN_G1_POS_PRE_OPEN     CONTROLLINO_A1
#define PIN_G1_POS_PRE_CLOSED   CONTROLLINO_A2
#define PIN_G1_SE_INNER         CONTROLLINO_A3
#define PIN_G1_SE_OUTER         CONTROLLINO_A4
#define PIN_G2_POS_OPEN_CLOSED  CONTROLLINO_A5
#define PIN_G2_POS_PRE_OPEN     CONTROLLINO_A6
#define PIN_G2_POS_PRE_CLOSED   CONTROLLINO_A7
#define PIN_G2_SE_INNER         CONTROLLINO_A8
#define PIN_G2_SE_OUTER         CONTROLLINO_A9
#define PIN_LIGHT_BARRIER       CONTROLLINO_IN0
#define PIN_FORCE_OPEN_ENABLE   CONTROLLINO_PIN_HEADER_DIGITAL_IN_10
#define PIN_FORCE_OPEN_DISABLE  CONTROLLINO_PIN_HEADER_DIGITAL_IN_11

// DIGITAL OUTPUT PINS (no pins in use)

// RELAY OUTPUT
#define RELAY_LAMP_GREEN    CONTROLLINO_R0  // 2026-11-08: not connected
#define RELAY_LAMP_WARNING  CONTROLLINO_R1
#define RELAY_FLOODLIGHT    CONTROLLINO_R2  // 2026-11-08: not connected
#define RELAY_G1_ESTOP      CONTROLLINO_R8
#define RELAY_G2_ESTOP      CONTROLLINO_R9

// VFD ATV320 MODBUS RTU ADDRESSES
#define VFD_ADD_G1 1  // Gate 1 VFD Modbus RTU address
#define VFD_ADD_G2 2  // Gate 2 VFD Modbus RTU address

// VFD ATV320 MODBUS RTU REGISTERS
// Register category: Control parameters
#define VFD_REG_CMD 8501          // [R/W], CMD, Command register
#define VFD_CMD_BIT_RUN 0         // Bit address. 0 = stop, 1 = run (assignment cannot be modified)
#define VFD_CMD_BIT_REVERSE 1     // Bit address. 0 = forward), 1 = reverse
#define VFD_CMD_BIT_FAULT_RESET 2 // Bit address. 1 = reset fault
#define VFD_CMD_BIT_FAST_STOP 3   // Bit address. 0 = fast stop enabled

// Register category: Setpoint parameters
#define VFD_REG_LFR 8502 // [HZ/10] [R/W] LFR, Frequency reference register (0x2136)
// Positive values = motor forward, negative values = reverse direction

// Register category: Status parameters
#define VFD_REG_ETA 3201  // [R] VFD status register

// Register category: Actual values parameters
#define VFD_REG_RFR 3202  // [R] [Hz /10] Motor output frequency (current motor frequency)

// Register category: Fault parameters
#define VFD_REG_LFT 7121  // [R] Last fault code

// Register category: Identification parameters
#define VFD_REG_NCV 3011  // [R] Drive nominal rating (used for identification)

// Register category: Communication parameters
#define VFD_REG_NM1 12741 // [R(/W)] Communication scanner input 1 value
#define VFD_REG_NC1 12761 // [R(/W)] Communication scanner output 1 value
// For information about VFD "communication scanner", see page 34 in Modbus
// Serial Link Manual (NVE41308 04/2016).
// Com scan input 1: 3201 ETA (status register)
// Com scan input 2: 3202 RFR (current frequency)
// Com scan output 1: 8501 CMD (command)
// Com scan output 2: 8502 LFR (speed setpoint)

// MODBUS TCP SERVER MAP (see document "plc-modbus-tcpip-server-setup")
#define COIL_WAIT_TRAFFIC         6 // Close after passage, shared by both gates
#define COIL_RESET_FAULTS         7 // Reset all faults at PLC and VFD's
#define COIL_OBSTRUCTED           8 // Warning, gate is obstructed (set by PLC)
#define COIL_FLOODLIGHT           9 // Control floodlight
#define HREG_STAY_OPEN_COUNTDOWN  0 // [s]
#define HREG_TRAFFIC_COUNTER      1
#define HREG_VFD1_REQUEST         2
#define HREG_VFD2_REQUEST         3
#define IREG_STATUS_SUMMARY       0
#define IREG_G1_FAULT_CODE        8
#define IREG_G2_FAULT_CODE        9

/* USER SETTINGS -------------------------------------------------------------*/

#define SERIAL_BAUD 115200 // [baud] USB serial communication (debug)
#define SERIAL_CMD_MAX_LEN 16 // [characters] Maximum length of a serial command
#define SERIAL_CMD_TIMEOUT 200 // [ms]
/* A serial command is executed at end of line (CR or LF), or this long after
   the last received character (terminals sending no line ending).
*/
#define VFD_STARTUP_TIMEOUT 10000 // [ms]
/* Maximum time to wait at PLC startup for both VFDs to answer Modbus RTU.
   The VFDs are polled continuously, so startup continues as soon as both
   have answered (ATV320 startup time ≈ 2 s). A VFD that has not answered
   within this time gives FAULT_VFD_COMM.
*/
#define VFD_STARTUP_RESET_ATTEMPTS 3
/* Number of VFD fault reset attempts at PLC startup, for a VFD reporting a
   fault. If the fault remains, the gate enters fault state as usual.
*/
#define VFD_STARTUP_RESET_DELAY 200 // [ms]
/* Time after a VFD fault reset command before the VFD status is read again.
*/
#define TIME_RETRACT 3000 // [ms]
/* If safety edge is activated, related gate will travel [TIME_RETRACT] in
   reverse direction in order to free the pinched object.
*/
#define TIME_RETRACT_MAX 10000 // [ms]
/* Retracting continues past TIME_RETRACT while the activated safety edge is
   still pressed, but never longer than this.
*/
#define TIME_WAIT_AFTER_RETRACT 30000 // [ms]
/* Waiting delay after gate has retracted.
   The gate doors meet at the inner edge. When a safety edge activation makes
   one gate retract, the other gate is fast stopped if it is closing (and only
   then). It does not retract. It waits in STATE_WAIT_AFTER_RETRACT until the
   retracting gate has left STATE_RETRACTING and STATE_WAIT_AFTER_RETRACT,
   but at least TIME_WAIT_AFTER_RETRACT, then continues closing.
*/
#define SAFETY_EDGE_MAX_EVENTS 3
/* Consecutive safety edge activations (without reaching the target position)
   before the gate goes to fault state.
*/
#define TIME_DELAY_MOVE 5000 // [ms]
/* Delay after gate movement has been requested until gate will start moving.
   This delay is primarily used for alerting users via signal warning light
   that gate will soon start moving. If the light barrier is obstructed while
   a gate is pending a closing movement, the delay is restarted and runs again
   in full once the light barrier is cleared.
*/
#define TIME_DELAY_LIGHT_BARRIER_INTERRUPT 30000 // [ms]
/* If light barrier is activated during closing movement, moving gate(s) will
   stop and wait after light barrier is cleared before resuming closing.
*/
#define TIME_OBSTRUCTED_WARNING 60000 // [ms]
/* A gate that has been obstructed continuously this long sets coil 8 (warning,
   gate is obstructed). A gate is obstructed when it is prevented from moving
   towards its target: one of its safety edges is activated, or the light
   barrier is obstructed while the gate's target is CLOSED. Only counted in the
   states STATE_PENDING_MOVE, STATE_OPENING, STATE_CLOSING,
   STATE_WAIT_LIGHT_BARRIER, STATE_RETRACTING and STATE_WAIT_AFTER_RETRACT
   (not while stopped, in fault or at rest). The coil is set again every
   TIME_OBSTRUCTED_WARNING while the obstruction continues, so one obstruction
   can give several reports (e.g. set after 60 s, 120 s, 180 s ...).
*/
#define TIME_LIGHT_BARRIER_THRESHOLD 200 // [ms]
/* If light barrier is obstructed/blocked longer than this, an object (human or
  vehicle maybe?) has passed the light barrier.
*/
#define TIME_DELAY_START_CLOSING 10000 // [ms]
/* Additional time delay after an object has passed through gate, before gate
   starts closing ("close after passage" mode).
*/
#define TIME_STAY_OPEN_MAX 43200 // [s] (12 hours)
/* Holding register 0 is limited to this value.
*/
#define TIME_STAY_OPEN_DEFAULT 120 // [s]
/* Holding register 0 (stay open countdown) is raised to this value if lower,
   at every cycle start request and when a gate enters STATE_STAY_OPEN.
*/
#define STOP_AUTO_CLEAR_TIME 600000UL // [ms] (10 minutes)
/* Stop coil is cleared by PLC after this time.
*/
#define MOVEMENT_TIMEOUT 50000 // [ms]
/* If gate stays in opening/closing state longer than MOVEMENT_TIMEOUT, gate
   will change state to FAULT.
*/
#define PRE_ZONE_TIMEOUT 10000 // [ms]
/* If gate moves inside the pre-open/pre-closed zone longer than this without
   reaching the end position, gate will change state to FAULT (end position
   sensor failure). Estimate: 18 deg pre-zone at 5 Hz is approx. 6 s.
*/
#define SENSOR_INVALID_TIME 1000 // [ms]
/* An invalid combination of position sensors must be present this long before
   gate will change state to FAULT.
*/
#define MOTOR_STOP_TIMEOUT 5000 // [ms]
/* Maximum time allowed for motor to stop after a fast stop before retracting.
*/
#define VFD_READ_STATUS_INTERVAL 800  // [ms]
/* Maximum time interval for VFD status update.
*/
#define VFD_READ_RETRY_INTERVAL 200  // [ms]
/* Time interval between VFD status read attempts after a failed read.
*/
#define RTU_FRAME_GAP_US 5000UL // [us]
/* Minimum bus silence before each Modbus RTU request. Modbus RTU requires at
   least 3.5 characters of silence between frames (about 2 ms at 19200 baud
   8E1). The ArduinoModbus library sends a request directly after the previous
   answer (< 1 ms). The VFD that did not send that answer hears the answer
   and the request as one frame, discards it (CRC error) and does not answer.
   Verified 2026-11-15 with test-rs485_2026-11-08: without a pause, the VFD
   polled second never answered its first request (whichever VFD it was).
*/
#define RTU_BUS_IDLE_MAX_WAIT_US 50000UL // [us]
/* Maximum time to wait for bus silence before a request, if bytes keep
   arriving (noise, late answer). The request is then sent anyway.
*/
#define MODBUS_TCP_MAX_CLIENTS 3
/* Maximum number of concurrent Modbus TCP clients (e.g. one gate control
   client + one monitoring client). All clients share the same coils and
   registers. The W5100 has 4 sockets; one is always kept free for listening,
   so 3 is the maximum. Further connection attempts are closed directly.
*/
#define MODBUS_TCP_CLIENT_TIMEOUT 10000 // [ms]
/* Inactivity timeout before closing an idle Modbus TCP client socket, freeing
   the W5100 socket. Clients must poll more often than this.
*/
#define MODBUS_TCP_STOP_TIMEOUT 50 // [ms]
/* Maximum time the loop waits for a graceful socket close when a client is
   disconnected (Ethernet library default is 1000 ms).
*/
#define VFD_COMM_ERROR_FAULT 5 //
/* VFD Modbus RTU communication consecutive faults required to move gate to
   fault state.
*/
#define FAULT_TIMEOUT 5000 // [ms]
/* Minimum time gate will stay in fault state before a reset is accepted.
*/
#define FAULT_STO_DELAY 4000 // [ms]
/* Time after entering fault state before the E-STOP relay (VFD Safe Torque
   Off) is de-energized. Allows the VFD to ramp the motor to stop first.
   On VFD communication fault the relay is de-energized immediately.
*/
#define FAULT_RESET_CONFIRM_TIME 2000 // [ms]
/* After a reset request, the fault cause must be gone within this time.
*/
#define MOTOR_SPEED_LOW 50 // [Hz/10]
/* Speed setpoint when pre-open or pre-closed sensors are activated (gate is
   close to fully open or fully closed).
*/
#define MOTOR_SPEED_HIGH 200 // [Hz/10]
/* Speed setpoint for normal gate movement
*/
#define MOTOR_SPEED_RETRACT 60 // [Hz/10]
/* Speed setpoint for gate retracting (safety edge was activated).
   NOTE: values below VFD parameter LSP are raised to LSP by the VFD.
*/

/* STATE MACHINE ENUMERATIONS ------------------------------------------------*/
// Values are reported in input register 0. Do not renumber.
enum GateState : uint8_t {
  STATE_IDLE = 0,               // No movement requested or target reached
  STATE_PENDING_MOVE = 1,       // Warning light delay before movement
  STATE_OPENING = 2,
  STATE_CLOSING = 3,
  STATE_STAY_OPEN = 4,          // Fully open in a cycle, waiting to close
  STATE_WAIT_LIGHT_BARRIER = 5, // Closing interrupted by light barrier
  STATE_RETRACTING = 6,         // Safety edge activated, moving away
  STATE_WAIT_AFTER_RETRACT = 7,  // After retract, or stopped by other gate's retract
  STATE_FAULT = 8,              // Latched, reset via coil 7
  STATE_STOPPED = 9             // Movement paused by stop coil
};

enum GateTarget : uint8_t {
  TARGET_NONE,
  TARGET_OPEN,
  TARGET_CLOSED
};

// Values are reported in input registers 8 (gate 1) and 9 (gate 2).
enum FaultCode : uint8_t {
  FAULT_NONE = 0,
  FAULT_VFD = 1,                  // VFD reports fault
  FAULT_VFD_COMM = 2,             // Modbus RTU communication lost
  FAULT_MOVEMENT_TIMEOUT = 3,     // MOVEMENT_TIMEOUT exceeded
  FAULT_PRE_ZONE_TIMEOUT = 4,     // End position not reached (sensor failure?)
  FAULT_POSITION_SENSORS = 5,     // Invalid position sensor combination
  FAULT_MOTOR_NOT_STOPPING = 6,   // Motor did not stop after fast stop
  FAULT_SAFETY_EDGE_REPEATED = 7  // SAFETY_EDGE_MAX_EVENTS reached
};

// VFD status structure, used for improving code readability.
struct VfdStatus {
  int16_t regCMD; // Modbus register CMD
  int16_t regLFR; // Modbus register LFR
  int16_t regETA; // Modbus register ETA
  int16_t regRFR; // Modbus register RFR
  uint16_t regReq;      // Modbus, user requested register
  uint16_t regReqValue; // Value of requested Modbus register
  bool waitingPower;          // Register ETA, bit 0 = 1 ready to switch on, waiting for power
  bool ready;                 // Register ETA, bit 1 = 0 not ready, = 1 ready
  bool running;               // Register ETA, bit 2 = 0 not running, = 1 running
  bool fault;                 // Register ETA, bit 3 = 0 no error detected, 1 = error detected
  bool voltageEnabled;        // Register ETA, bit 4 = 0 power supply mains absent
  bool quickStop;             // Register ETA, bit 5 = 0 quick stop active (inverted)
  bool switchOnDisabled;      // Register ETA, bit 6 = 1 switch on disabled
  bool warning;               // Register ETA, bit 7 = 0 no warning, 1 = warning active
  bool remoteControl;         // Register ETA, bit 9 = 0 local control, 1 = remote control
  bool motorTargetRefReached; // Register ETA, bit 10 = 0 ref not reached, 1 = ref reached
  bool motorLimitReached;     // Register ETA, bit 11 = 0 ref within limits, 1 = ref outside limits
  bool stopKeyEnabled;        // Register ETA, bit 14 = 0 stop key not active, 1 = active
  bool direction;             // Register ETA, bit 15 = 0 forward rotation, 1 = reverse rotation
  // reserved bits (8,12-13) are ignored
};

/* GATE DATA STRUCTURE -------------------------------------------------------*/
struct Gate {
  Gate(int vfdAddress,
       int posOpenClosed, int posPreOpen, int posPreClosed,
       int seInner, int seOuter, int relayEstop,
       int coilStartCycle, int coilStop, int coilClose,
       int hregVfdRequest)
    : vfdModbusRtuAddress(vfdAddress),
      pinPosOpenClosed(posOpenClosed), pinPosPreOpen(posPreOpen),
      pinPosPreClosed(posPreClosed),
      pinSeInner(seInner), pinSeOuter(seOuter), relayEstop(relayEstop),
      coilStartCycle(coilStartCycle), coilStop(coilStop),
      coilClose(coilClose),
      hregVfdRequest(hregVfdRequest) {}

  // Configuration
  const int vfdModbusRtuAddress;  // Also used as gate number in debug messages
  const int pinPosOpenClosed, pinPosPreOpen, pinPosPreClosed;
  const int pinSeInner, pinSeOuter;
  const int relayEstop;
  const int coilStartCycle, coilStop, coilClose;
  const int hregVfdRequest;

  // Inputs, sampled once per loop
  bool isPreOpen = false, isPreClosed = false;
  bool isFullyOpen = false, isFullyClosed = false;
  bool seInnerHit = false, seOuterHit = false;
  bool sensorsInvalid = false;          // Invalid sensor combination (filtered)
  unsigned long sensorsValidTime = 0;   // Last time sensor combination was valid

  // Gate control
  GateState state = STATE_IDLE;
  GateTarget target = TARGET_NONE;
  bool cycleActive = false;     // Target OPEN is part of open -> stay open -> close cycle
  bool passageSeen = false;     // Object passed light barrier during STAY_OPEN
  FaultCode faultCode = FAULT_NONE;
  uint8_t safetyEdgeEvents = 0; // Consecutive safety edge activations
  bool retractTowardOpen = false;
  bool retractMoving = false;   // RETRACTING: false = waiting for motor stop
  bool heldByOtherGate = false; // WAIT_AFTER_RETRACT: stopped by other gate's retract
  bool resetPending = false;    // FAULT: reset requested, waiting for confirmation
  uint32_t resetStatusSeq = 0;  // VFD status sequence number at reset request
  bool stoReleased = true;      // E-STOP relay energized (Safe Torque Off released)
  unsigned long timerStart = 0;      // Time of last change of state machine state
  unsigned long timerPreZone = 0;    // Time gate entered pre-zone during movement
  unsigned long timerStopEn = 0;     // Time of stop coil enabled
  unsigned long timerReset = 0;      // Time of fault reset request

  // VFD
  VfdStatus vfd {};             // VFD status register parsed to struct for readability
  int currentSpeed = 0;         // Requested speed [Hz/10]
  int lastSpeedCmd = 0;         // Speed sent to VFD (after fast stop override)
  bool fastStop = false;        // Fast stop request, held until motor has stopped
  uint8_t vfdCommErrorCounter = 0;   // Count consecutive communication faults
  uint32_t vfdStatusSeq = 0;    // Incremented on every successful status read
  uint32_t stopCmdSeq = 0;      // vfdStatusSeq when last stop command was given
  unsigned long timerReadVfdStatus = 0; // Time of last VFD status read attempt
  bool forceReadVfdStatus = false;      // Force read VFD status on next opportunity

  // Obstruction warning (coil 8)
  unsigned long timerObstructed = 0;    // Start of obstruction, or time of last report
  bool obstructionWarning = false;      // Coil 8 set by this gate, cleared at target

  // Fault simulation, see FAULT SIMULATION in header
  bool simVfdFault = false;        // VFD status reports fault until VFD fault reset
  bool simCommLost = false;        // Modbus RTU requests fail until FAULT CLEAR
  bool simSensorsInvalid = false;  // Invalid sensor combination until FAULT CLEAR
  FaultCode simEventFault = FAULT_NONE; // Fault 3, 4, 6 or 7, triggered at next loop
};

// Initialize Gate 1 and Gate 2
Gate gate1(VFD_ADD_G1,
           PIN_G1_POS_OPEN_CLOSED, PIN_G1_POS_PRE_OPEN, PIN_G1_POS_PRE_CLOSED,
           PIN_G1_SE_INNER, PIN_G1_SE_OUTER, RELAY_G1_ESTOP,
           0, 1, 2,
           HREG_VFD1_REQUEST);
Gate gate2(VFD_ADD_G2,
           PIN_G2_POS_OPEN_CLOSED, PIN_G2_POS_PRE_OPEN, PIN_G2_POS_PRE_CLOSED,
           PIN_G2_SE_INNER, PIN_G2_SE_OUTER, RELAY_G2_ESTOP,
           3, 4, 5,
           HREG_VFD2_REQUEST);

/* MODBUS TCP/IP SETUP -------------------------------------------------------*/
/* Controllino PLC Wiznet W5100 controller does not have a factory-assigned or
   pre-programmed MAC address. */
byte mac[] = { 0x32, 0x71, 0x82, 0x73, 0x78, 0x68 };  // ASCII for " GRIND"
IPAddress ip(192, 168, 52, 3);
EthernetServer ethServer(502);  // IANA official port (502) for Modbus TCP/IP
ModbusTCPServer modbusTCPServer;

/* GLOBAL VARIABLES ----------------------------------------------------------*/
int debug = 1;                        // Debug level, default OFF TODO: Set to 0 once testing completed!
unsigned long lightBarrierTimerStart; // For gate traffic detection
bool lightBarrierPrevState = true;    // For gate traffic detection (true = not obstructed)
bool lightBarrierObstructed;          // Sampled once per loop
bool passageDetected;                 // True during the loop an object has passed
unsigned long lastPassageTime;        // Time of last detected passage
bool forceOpen = false;               // Emergency services force gate open
bool waitTraffic = false;             // Close after passage (coil 6), both gates
unsigned long rtuLastActivityUs = 0;  // Last Modbus RTU bus activity (end of transaction or received byte)

/* SETUP ---------------------------------------------------------------------*/

void setup() {
  pinMode(PIN_G1_POS_OPEN_CLOSED, INPUT);
  pinMode(PIN_G1_POS_PRE_OPEN, INPUT);
  pinMode(PIN_G1_POS_PRE_CLOSED, INPUT);
  pinMode(PIN_G1_SE_INNER, INPUT);
  pinMode(PIN_G1_SE_OUTER, INPUT);
  pinMode(PIN_G2_POS_OPEN_CLOSED, INPUT);
  pinMode(PIN_G2_POS_PRE_OPEN, INPUT);
  pinMode(PIN_G2_POS_PRE_CLOSED, INPUT);
  pinMode(PIN_G2_SE_INNER, INPUT);
  pinMode(PIN_G2_SE_OUTER, INPUT);
  pinMode(PIN_LIGHT_BARRIER, INPUT);
  pinMode(CONTROLLINO_IN1, INPUT);  // Not connected
  pinMode(PIN_FORCE_OPEN_ENABLE, INPUT);
  pinMode(PIN_FORCE_OPEN_DISABLE, INPUT);
  pinMode(CONTROLLINO_PIN_HEADER_DIGITAL_IN_12, INPUT); // Not connected
  pinMode(CONTROLLINO_PIN_HEADER_DIGITAL_IN_13, INPUT); // Not connected

  pinMode(RELAY_LAMP_GREEN, OUTPUT);
  pinMode(RELAY_LAMP_WARNING, OUTPUT);
  pinMode(RELAY_FLOODLIGHT, OUTPUT);
  pinMode(RELAY_G1_ESTOP, OUTPUT);
  pinMode(RELAY_G2_ESTOP, OUTPUT);

  // Serial is always started to receive debug commands, see handleSerialCommands()
  Serial.begin(SERIAL_BAUD);
  Serial.println(F("Gate PLC started. Send DEBUG to enable debug messages."));
  Serial.println(F("FAULT SIMULATION version. Send FAULT for fault simulation status."));

  // Initialize Modbus RTU Client
  // Set short Modbus RTU communication timeout to prevent hogging CPU when a
  // Modbus RTU bus device is not responding.
  ModbusRTUClient.setTimeout(200);  // Default timeout is 1000 ms
  if (!ModbusRTUClient.begin(19200, SERIAL_8E1)) {
    Serial.println(F("Error starting Modbus RTU Client!"));
    // Stop execution if Modbus client fails to initialize
    // Modbus communication is required for basic functionality
    while (1);
  }

  // Start the Ethernet connection
  Ethernet.begin(mac, ip);
  // Limit how long sending to an unresponsive client (e.g. a monitoring laptop
  // unplugged mid-request) can block the loop. Defaults: 200 ms, 8 retries.
  Ethernet.setRetransmissionTimeout(100); // [ms]
  Ethernet.setRetransmissionCount(4);

  // Start ethernet connection/server
  ethServer.begin();

  // Start Modbus TCP server
  if (!modbusTCPServer.begin()) {
    Serial.println(F("Failed to start Modbus TCP Server!"));
    while (1);
  }

  // Setup Modbus registers
  // configure...(address [DEC, 0-based index], number of coils/registers)
  // verified ok to use same address values [DEC] for coils + registers
  modbusTCPServer.configureCoils(0, 10);
  modbusTCPServer.configureInputRegisters(0, 10);
  modbusTCPServer.configureHoldingRegisters(0, 4);

  // Wait for auxiliary equipment (VFD's) to startup. Polls the VFDs, so
  // startup continues as soon as both have answered.
  waitForVfds();

  // Make sure VFDs have no run command (may remain from before a PLC restart)
  // before Safe Torque Off is released.
  writeVfdStop(&gate1);
  writeVfdStop(&gate2);

  // Reset VFD faults present at startup (e.g. Modbus communication
  // interruption detected by the VFD while the PLC restarted).
  resetVfdFaultAtStartup(&gate1);
  resetVfdFaultAtStartup(&gate2);

  // Relays de-energize automatically on PLC power loss or hardware failure.
  // From here on relays are controlled by the state machine (Gate.stoReleased).
  digitalWrite(RELAY_G1_ESTOP, HIGH); // Release VFD Safe Torque Off
  digitalWrite(RELAY_G2_ESTOP, HIGH); // Release VFD Safe Torque Off

  // Return gates to closed position (e.g. after power outage while open).
  // Fully closed gates are left alone, see dispatch().
  gate1.target = TARGET_CLOSED;
  gate2.target = TARGET_CLOSED;
}

/* LOOP ----------------------------------------------------------------------*/

void loop() {
  unsigned long currentMillis = millis();

  // Handle debug commands over USB serial
  handleSerialCommands(currentMillis);

  // Handle Modbus TCP/IP server
  handleModbusTCP();

  // Rescue services emergency access force gate open
  if (digitalRead(PIN_FORCE_OPEN_ENABLE)) {
    forceOpen = true;
  } else if (digitalRead(PIN_FORCE_OPEN_DISABLE)) {
    forceOpen = false;
  }

  // Shared inputs, must be updated before the state machines
  waitTraffic = modbusTCPServer.coilRead(COIL_WAIT_TRAFFIC);
  lightBarrierObstructed = !digitalRead(PIN_LIGHT_BARRIER);
  passageDetected = handleTrafficMonitor(currentMillis);
  updateStayOpenCountdown(currentMillis);

  runGateStateMachine(&gate1, currentMillis);
  runGateStateMachine(&gate2, currentMillis);

  // After the state machines
  resetStayOpenCountdownAtCycleEnd();
  handleObstructionWarning(currentMillis);

  // Reset eventual Modbus TCP fault reset request.
  // Both gates has already processed the request.
  modbusTCPServer.coilWrite(COIL_RESET_FAULTS, 0);

  handleIndicationLamps();
  handleFloodlight();

  updateModbusTCPRegisters();
}

/* MODBUS TCP/IP HANDLING ----------------------------------------------------*/
/* Serve up to MODBUS_TCP_MAX_CLIENTS concurrent Modbus TCP clients.
   ModbusTCPServer can only talk to one client at a time, so the server is
   pointed to each client with pending data in turn (accept() only stores a
   client pointer) and one request is answered per client and loop. */
void handleModbusTCP() {
  static EthernetClient clients[MODBUS_TCP_MAX_CLIENTS];
  static unsigned long lastActivity[MODBUS_TCP_MAX_CLIENTS];
  unsigned long currentMillis = millis();

  // 1. Serve connected clients, release disconnected or idle clients.
  // Done before accepting new clients, so closed sockets are released before
  // the Ethernet library can reuse them for a new connection.
  for (uint8_t i = 0; i < MODBUS_TCP_MAX_CLIENTS; i++) {
    if (!clients[i]) {
      continue; // Free slot
    }
    if (clients[i].available()) {
      lastActivity[i] = currentMillis;
      modbusTCPServer.accept(clients[i]); // Direct server replies to this client
      modbusTCPServer.poll();             // Read one request and send reply
    } else if (!clients[i].connected()) {
      if (debug) {
        printModbusClient(clients[i], i);
        Serial.println(F("disconnected. Socket freed."));
      }
      clients[i].stop();
    } else if (currentMillis - lastActivity[i] > MODBUS_TCP_CLIENT_TIMEOUT) {
      if (debug) {
        printModbusClient(clients[i], i);
        Serial.println(F("idle timeout. Socket freed."));
      }
      clients[i].stop();
    }
  }

  // 2. Accept new connection (returned once, also before any data is received)
  EthernetClient newClient = ethServer.accept();
  if (newClient) {
    newClient.setConnectionTimeout(MODBUS_TCP_STOP_TIMEOUT);
    // Use the slot already holding this socket (socket reused by the Ethernet
    // library), otherwise the first free slot.
    uint8_t slot = MODBUS_TCP_MAX_CLIENTS;
    for (uint8_t i = 0; i < MODBUS_TCP_MAX_CLIENTS; i++) {
      if (clients[i] &&
          clients[i].getSocketNumber() == newClient.getSocketNumber()) {
        slot = i;
        break;
      }
    }
    if (slot == MODBUS_TCP_MAX_CLIENTS) {
      for (uint8_t i = 0; i < MODBUS_TCP_MAX_CLIENTS; i++) {
        if (!clients[i]) {
          slot = i;
          break;
        }
      }
    }
    if (slot < MODBUS_TCP_MAX_CLIENTS) {
      clients[slot] = newClient;
      lastActivity[slot] = currentMillis;
      if (debug) {
        printModbusClient(newClient, slot);
        Serial.println(F("accepted."));
      }
    } else {
      // All slots in use: reject, keep existing clients
      if (debug) {
        printModbusClient(newClient, MODBUS_TCP_MAX_CLIENTS);
        Serial.println(F("rejected, too many clients."));
      }
      newClient.stop();
    }
  }
}

void printModbusClient(EthernetClient &client, uint8_t slot) {
  Serial.print(F("Modbus TCP: client "));
  if (slot < MODBUS_TCP_MAX_CLIENTS) {
    Serial.print(slot);
    Serial.print(F(" "));
  }
  Serial.print(client.remoteIP());
  Serial.print(F(":"));
  Serial.print(client.remotePort());
  Serial.print(F(" "));
}

/* Parse VFD status register into vfdStatus struct */
void parseVfdStatus(Gate *g) {
  int16_t regValue = g->vfd.regETA;
  g->vfd.waitingPower =           (regValue & (1 << 0))  != 0;
  g->vfd.ready =                  (regValue & (1 << 1))  != 0;
  g->vfd.running =                (regValue & (1 << 2))  != 0;
  g->vfd.fault =                  (regValue & (1 << 3))  != 0;
  g->vfd.voltageEnabled =         (regValue & (1 << 4))  != 0;
  g->vfd.quickStop =              (regValue & (1 << 5))  != 0;
  g->vfd.switchOnDisabled =       (regValue & (1 << 6))  != 0;
  g->vfd.warning =                (regValue & (1 << 7))  != 0;
  g->vfd.remoteControl =          (regValue & (1 << 9))  != 0;
  g->vfd.motorTargetRefReached =  (regValue & (1 << 10)) != 0;
  g->vfd.motorLimitReached =      (regValue & (1 << 11)) != 0;
  g->vfd.stopKeyEnabled =         (regValue & (1 << 14)) != 0;
  g->vfd.direction =              (regValue & (1 << 15)) != 0;
}

/* STATE MACHINE LOGIC -------------------------------------------------------*/

void runGateStateMachine(Gate *g, unsigned long currentMillis) {
  readGateInputs(g, currentMillis);

  // Read VFD device status. Note: Status is only read at time interval
  readVfdStatus(g, currentMillis);

  bool cmdStop = handleGateCommands(g, currentMillis);
  bool safetyEdgeHit = g->seInnerHit || g->seOuterHit;

  if (passageDetected && g->state == STATE_STAY_OPEN) {
    g->passageSeen = true;
  }

  // Fault detection (faults are latched, see STATE_FAULT)
  if (g->state != STATE_FAULT) {
    if (g->vfdCommErrorCounter >= VFD_COMM_ERROR_FAULT) {
      enterFault(g, FAULT_VFD_COMM, currentMillis);
    } else if (g->vfd.fault) {
      enterFault(g, FAULT_VFD, currentMillis);
    } else if (g->sensorsInvalid) {
      enterFault(g, FAULT_POSITION_SENSORS, currentMillis);
    } else if (g->simEventFault != FAULT_NONE) {
      enterFault(g, g->simEventFault, currentMillis);  // Simulated fault
    }
  }
  g->simEventFault = FAULT_NONE;  // Simulated event faults trigger only once

  // Immediate safety override
  if (safetyEdgeHit) {
    if (g->state == STATE_OPENING || g->state == STATE_CLOSING) {
      // Retract away from the activated safety edge. Inner edge activated =
      // retract towards OPEN. If both edges are activated, retract opposite
      // to the current movement.
      if (g->seInnerHit && g->seOuterHit) {
        g->retractTowardOpen = (g->state == STATE_CLOSING);
      } else {
        g->retractTowardOpen = g->seInnerHit;
      }
      g->retractMoving = false;
      g->safetyEdgeEvents++;
      requestFastStop(g);
      enterState(g, STATE_RETRACTING, currentMillis);
      stopOtherGateIfClosing(g, currentMillis);
    } else if (g->state != STATE_RETRACTING && g->state != STATE_FAULT &&
               !motorStopped(g)) {
      // Motor is still ramping down (e.g. just stopped by command)
      requestFastStop(g);
    }
  }

  switch (g->state) {
    case STATE_IDLE:
    case STATE_STOPPED:
      g->currentSpeed = 0;
      // Start, resume or keep resting depending on target
      dispatch(g, cmdStop, currentMillis);
      break;

    case STATE_PENDING_MOVE:
      g->currentSpeed = 0;
      if (!movementRequired(g) || cmdStop) {
        dispatch(g, cmdStop, currentMillis);
        break;
      }
      // Restart warning delay while something is in the way
      if (safetyEdgeHit) {
        g->timerStart = currentMillis;
      }
      if (g->target == TARGET_CLOSED && lightBarrierObstructed) {
        g->timerStart = currentMillis;
      }
      if (currentMillis - g->timerStart >= TIME_DELAY_MOVE) {
        enterState(g, (g->target == TARGET_OPEN) ? STATE_OPENING : STATE_CLOSING,
                   currentMillis);
      }
      break;

    case STATE_OPENING:
      if (g->target != TARGET_OPEN || g->isFullyOpen || cmdStop) {
        g->currentSpeed = 0;
        dispatch(g, cmdStop, currentMillis);
        break;
      }
      g->currentSpeed = (g->isPreOpen) ? MOTOR_SPEED_LOW : MOTOR_SPEED_HIGH;
      checkMovementTimeouts(g, g->isPreOpen, currentMillis);
      break;

    case STATE_CLOSING:
      if (lightBarrierObstructed) {
        requestFastStop(g);
        enterState(g, STATE_WAIT_LIGHT_BARRIER, currentMillis);
        break;
      }
      if (g->target != TARGET_CLOSED || g->isFullyClosed || cmdStop) {
        g->currentSpeed = 0;
        dispatch(g, cmdStop, currentMillis);
        break;
      }
      // Negative motor speed values due to reverse movement
      g->currentSpeed = (g->isPreClosed) ? -MOTOR_SPEED_LOW : -MOTOR_SPEED_HIGH;
      checkMovementTimeouts(g, g->isPreClosed, currentMillis);
      break;

    case STATE_STAY_OPEN:
      g->currentSpeed = 0;
      if (g->target != TARGET_OPEN || !g->cycleActive) {
        // Close command or force open received
        dispatch(g, cmdStop, currentMillis);
        break;
      }
      if (!cmdStop) {
        bool timeUp = (modbusTCPServer.holdingRegisterRead(HREG_STAY_OPEN_COUNTDOWN) == 0);
        bool passed = waitTraffic && g->passageSeen && !lightBarrierObstructed &&
                      (currentMillis - lastPassageTime >= TIME_DELAY_START_CLOSING);
        if (timeUp || passed) {
          if (debug) {
            printGateNumber(g->vfdModbusRtuAddress);
            Serial.println(timeUp ? "Stay open time is up." : "Object has passed through gate.");
          }
          g->target = TARGET_CLOSED;
          g->cycleActive = false;
          dispatch(g, cmdStop, currentMillis);
        }
      }
      break;

    case STATE_WAIT_LIGHT_BARRIER:
      // Stop gate movement
      g->currentSpeed = 0;
      if (lightBarrierObstructed) {
        // Reset timer as long as light barrier is active
        g->timerStart = currentMillis;
      }
      if (g->target != TARGET_CLOSED || cmdStop ||
          currentMillis - g->timerStart > TIME_DELAY_LIGHT_BARRIER_INTERRUPT) {
        dispatch(g, cmdStop, currentMillis);
      }
      break;

    case STATE_RETRACTING:
      if (!g->retractMoving) {
        // Wait until motor has stopped
        g->currentSpeed = 0;
        if (motorStopped(g)) {
          g->retractMoving = true;
          g->timerStart = currentMillis;
          if (debug) {
            printGateNumber(g->vfdModbusRtuAddress);
            Serial.println("Motor has stopped. Retracting.");
          }
        } else if (currentMillis - g->timerStart > MOTOR_STOP_TIMEOUT) {
          enterFault(g, FAULT_MOTOR_NOT_STOPPING, currentMillis);
          break;
        } else {
          break;
        }
      }
      {
        // Edge leading in retract direction: outer when opening, inner when closing
        bool leadingEdgeHit = g->retractTowardOpen ? g->seOuterHit : g->seInnerHit;
        bool triggerEdgeHit = g->retractTowardOpen ? g->seInnerHit : g->seOuterHit;
        bool endReached = g->retractTowardOpen ? g->isFullyOpen : g->isFullyClosed;
        bool barrierBlocked = !g->retractTowardOpen && lightBarrierObstructed;
        unsigned long elapsed = currentMillis - g->timerStart;

        if (endReached || leadingEdgeHit || barrierBlocked ||
            (elapsed > TIME_RETRACT && !triggerEdgeHit) ||
            elapsed > TIME_RETRACT_MAX) {
          if (leadingEdgeHit || barrierBlocked) {
            requestFastStop(g);
          }
          g->currentSpeed = 0;
          enterState(g, STATE_WAIT_AFTER_RETRACT, currentMillis);
        } else {
          g->currentSpeed = g->retractTowardOpen ? MOTOR_SPEED_RETRACT : -MOTOR_SPEED_RETRACT;
        }
      }
      break;

    case STATE_WAIT_AFTER_RETRACT:
      g->currentSpeed = 0;
      // Restart time counter if any safety edge is hit/activated
      if (safetyEdgeHit) {
        g->timerStart = currentMillis;
      }
      if (g->heldByOtherGate) {
        // Stopped by the other gate's retract: wait until it has finished
        Gate *other = otherGate(g);
        if (other->state == STATE_RETRACTING ||
            other->state == STATE_WAIT_AFTER_RETRACT) {
          break;
        }
      }
      if (currentMillis - g->timerStart > TIME_WAIT_AFTER_RETRACT) {
        if (g->safetyEdgeEvents >= SAFETY_EDGE_MAX_EVENTS) {
          enterFault(g, FAULT_SAFETY_EDGE_REPEATED, currentMillis);
        } else {
          dispatch(g, cmdStop, currentMillis);
        }
      }
      break;

    case STATE_FAULT:
      // The system is not safe to use or cannot be used (example: VFD in error)
      g->currentSpeed = 0;
      runFaultState(g, currentMillis);
      break;
  }

  digitalWrite(g->relayEstop, g->stoReleased ? HIGH : LOW);
  setVfdState(g);
}

/* Read position sensors and safety edges of one gate */
void readGateInputs(Gate *g, unsigned long currentMillis) {
  bool openClosed = digitalRead(g->pinPosOpenClosed);
  g->isPreOpen = digitalRead(g->pinPosPreOpen);
  g->isPreClosed = digitalRead(g->pinPosPreClosed);
  g->isFullyOpen = openClosed && g->isPreOpen && !g->isPreClosed;
  g->isFullyClosed = openClosed && g->isPreClosed && !g->isPreOpen;

  // Valid combinations, see wiring diagram "gate position sensor logic"
  bool valid = !g->simSensorsInvalid &&  // Simulated fault
               !(g->isPreOpen && g->isPreClosed) &&
               !(openClosed && !g->isPreOpen && !g->isPreClosed);
  if (valid) {
    g->sensorsValidTime = currentMillis;
  }
  g->sensorsInvalid = (currentMillis - g->sensorsValidTime > SENSOR_INVALID_TIME);

  // Safety edge interface output LOW = safety edge activated (fail-safe)
  g->seInnerHit = !digitalRead(g->pinSeInner);
  g->seOuterHit = !digitalRead(g->pinSeOuter);
}

/* Read and clear Modbus TCP command coils. Returns true if gate is stopped. */
bool handleGateCommands(Gate *g, unsigned long currentMillis) {
  if (modbusTCPServer.coilRead(g->coilStartCycle)) {
    modbusTCPServer.coilWrite(g->coilStartCycle, false);
    modbusTCPServer.coilWrite(g->coilStop, false);  // Cycle start clears stop
    g->target = TARGET_OPEN;
    g->cycleActive = true;
    g->passageSeen = false;
    // Last cycle start request (either gate) decides when the gate(s) close
    raiseStayOpenCountdown();
    if (debug) {
      printGateNumber(g->vfdModbusRtuAddress);
      Serial.println("Cycle start requested.");
    }
  }

  if (modbusTCPServer.coilRead(g->coilClose)) {
    modbusTCPServer.coilWrite(g->coilClose, false);
    modbusTCPServer.coilWrite(g->coilStop, false);  // Close command clears stop
    g->target = TARGET_CLOSED;
    g->cycleActive = false;
    if (debug) {
      printGateNumber(g->vfdModbusRtuAddress);
      Serial.println("Close requested.");
    }
  }

  bool cmdStop = modbusTCPServer.coilRead(g->coilStop);
  if (cmdStop) {
    if (currentMillis - g->timerStopEn >= STOP_AUTO_CLEAR_TIME) {
      modbusTCPServer.coilWrite(g->coilStop, false);
      cmdStop = false;
    }
  } else {
    g->timerStopEn = currentMillis; // Reset timer
  }

  // Force gate to open. Overrides stop and close requests.
  if (forceOpen) {
    g->target = TARGET_OPEN;
    g->cycleActive = false;
    cmdStop = false;
  }

  return cmdStop;
}

/* True if gate is not in its target position */
bool movementRequired(Gate *g) {
  return (g->target == TARGET_OPEN && !g->isFullyOpen) ||
         (g->target == TARGET_CLOSED && !g->isFullyClosed);
}

/* Select next state from target and gate position. Used by all states that
   end, interrupt or resume a movement. */
void dispatch(Gate *g, bool cmdStop, unsigned long currentMillis) {
  if (g->target == TARGET_OPEN && g->isFullyOpen) {
    g->safetyEdgeEvents = 0;
    if (g->cycleActive) {
      if (g->state != STATE_STAY_OPEN) {
        g->passageSeen = false;
        raiseStayOpenCountdown();
      }
      enterState(g, STATE_STAY_OPEN, currentMillis);
    } else {
      if (!forceOpen) {
        g->target = TARGET_NONE;
      }
      enterState(g, STATE_IDLE, currentMillis);
    }
  } else if (g->target == TARGET_CLOSED && g->isFullyClosed) {
    g->safetyEdgeEvents = 0;
    g->target = TARGET_NONE;
    enterState(g, STATE_IDLE, currentMillis);
  } else if (g->target == TARGET_NONE) {
    enterState(g, STATE_IDLE, currentMillis);
  } else if (g->vfdStatusSeq == 0) {
    // VFD has not answered yet since PLC startup, wait before moving
    enterState(g, STATE_IDLE, currentMillis);
  } else if (cmdStop) {
    enterState(g, STATE_STOPPED, currentMillis);
  } else if (g->state != STATE_PENDING_MOVE) {
    enterState(g, STATE_PENDING_MOVE, currentMillis);
  }
}

/* Fault if movement takes too long, or too long in pre-zone without reaching
   the end position. */
void checkMovementTimeouts(Gate *g, bool inPreZone, unsigned long currentMillis) {
  if (!inPreZone) {
    g->timerPreZone = currentMillis;
  }
  if (currentMillis - g->timerStart > MOVEMENT_TIMEOUT) {
    enterFault(g, FAULT_MOVEMENT_TIMEOUT, currentMillis);
  } else if (currentMillis - g->timerPreZone > PRE_ZONE_TIMEOUT) {
    enterFault(g, FAULT_PRE_ZONE_TIMEOUT, currentMillis);
  }
}

void runFaultState(Gate *g, unsigned long currentMillis) {
  // Remove motor torque via E-STOP relay (VFD Safe Torque Off). Immediately if
  // the VFD cannot be commanded, otherwise after the VFD has ramped down.
  if (g->faultCode == FAULT_VFD_COMM || currentMillis - g->timerStart > FAULT_STO_DELAY) {
    g->stoReleased = false;
  }

  bool causeActive = (g->vfdCommErrorCounter >= VFD_COMM_ERROR_FAULT) ||
                     g->vfd.fault || g->sensorsInvalid;

  if (g->resetPending) {
    // Require a VFD status read after the reset request
    if (g->vfdStatusSeq != g->resetStatusSeq && !causeActive) {
      g->resetPending = false;
      g->faultCode = FAULT_NONE;
      g->safetyEdgeEvents = 0;
      // Target and cycleActive are kept: the gate continues towards its
      // target via STATE_IDLE -> dispatch() -> STATE_PENDING_MOVE (warning
      // delay), or enters STATE_STAY_OPEN if fully open in a cycle.
      g->stoReleased = true;
      enterState(g, STATE_IDLE, currentMillis);
    } else if (currentMillis - g->timerReset > FAULT_RESET_CONFIRM_TIME) {
      g->resetPending = false;
      if (debug) {
        printGateNumber(g->vfdModbusRtuAddress);
        Serial.println("Fault reset failed, fault cause still active.");
      }
    }
  } else if (currentMillis - g->timerStart > FAULT_TIMEOUT &&
             (modbusTCPServer.coilRead(COIL_RESET_FAULTS) || forceOpen)) {
    // Attempt to clear eventual VFD internal fault (even if fault is not in VFD)
    clearVfdFault(g);
    g->resetPending = true;
    g->resetStatusSeq = g->vfdStatusSeq;
    g->timerReset = currentMillis;
    g->forceReadVfdStatus = true;
    if (debug) {
      printGateNumber(g->vfdModbusRtuAddress);
      Serial.println("Fault reset requested.");
    }
  }
}

void enterState(Gate *g, GateState newState, unsigned long currentMillis) {
  if (g->state == newState) {
    return;
  }
  g->state = newState;
  g->heldByOtherGate = false;  // Set again by stopOtherGateIfClosing() if needed
  g->timerStart = currentMillis;
  g->timerPreZone = currentMillis;
  if (debug) {
    printGateNumber(g->vfdModbusRtuAddress);
    Serial.print("State set to ");
    Serial.println(stateName(newState));
  }
}

void enterFault(Gate *g, FaultCode code, unsigned long currentMillis) {
  g->currentSpeed = 0;
  g->faultCode = code;
  g->resetPending = false;
  enterState(g, STATE_FAULT, currentMillis);
  if (debug) {
    printGateNumber(g->vfdModbusRtuAddress);
    Serial.print("Fault code: ");
    Serial.println(code);
  }
}

Gate *otherGate(Gate *g) {
  return (g == &gate1) ? &gate2 : &gate1;
}

/* Gate g has started a retract (safety edge activated). The gate doors meet
   at the inner edge, so the other gate is fast stopped if it is closing. Any
   other state of the other gate is not affected. */
void stopOtherGateIfClosing(Gate *g, unsigned long currentMillis) {
  Gate *other = otherGate(g);
  if (other->state != STATE_CLOSING) {
    return;
  }
  // Own safety edge of the other gate also activated (e.g. object between the
  // doors): leave it closing, it retracts by its own safety edge handling.
  if (!digitalRead(other->pinSeInner) || !digitalRead(other->pinSeOuter)) {
    return;
  }
  requestFastStop(other);
  enterState(other, STATE_WAIT_AFTER_RETRACT, currentMillis);
  other->heldByOtherGate = true;
  setVfdState(other);  // Send stop command now, not at the next loop
  if (debug) {
    printGateNumber(other->vfdModbusRtuAddress);
    Serial.println(F("Stopped, other gate safety edge activated."));
  }
}

/* Fast stop, held until motor has stopped (see setVfdState) */
void requestFastStop(Gate *g) {
  if (!g->fastStop) {
    g->fastStop = true;
    g->stopCmdSeq = g->vfdStatusSeq;
  }
  g->currentSpeed = 0;
}

/* True if a VFD status read after the last stop command reports 0 Hz */
bool motorStopped(Gate *g) {
  return g->lastSpeedCmd == 0 &&
         g->vfdStatusSeq != g->stopCmdSeq &&
         g->vfd.regRFR == 0;
}

/* VFD MODBUS RTU CONTROL ----------------------------------------------------*/

/* Wait until the bus has been silent for RTU_FRAME_GAP_US before a request.
   Bytes received meanwhile (late answer after a timeout, noise) are discarded
   and restart the silence time, so they are not read as the answer to the
   next request. Waits at most RTU_BUS_IDLE_MAX_WAIT_US. */
void rtuWaitBusIdle() {
  unsigned long start = micros();
  while (micros() - rtuLastActivityUs < RTU_FRAME_GAP_US &&
         micros() - start < RTU_BUS_IDLE_MAX_WAIT_US) {
    if (RS485.available()) {
      RS485.read();
      rtuLastActivityUs = micros();
    }
  }
  while (RS485.available()) {
    RS485.read();
  }
}

/* Read holding registers, with bus silence before the request. Returns the
   number of values received, read them with ModbusRTUClient.read(). */
int rtuRequestFrom(int address, int reg, int count) {
  if (isSimCommLost(address)) {
    clearModbusRtuBuffer();
    return 0;  // Simulated communication loss, request fails
  }
  rtuWaitBusIdle();
  int n = ModbusRTUClient.requestFrom(address, HOLDING_REGISTERS, reg, count);
  rtuLastActivityUs = micros();
  return n;
}

/* Write one holding register, with bus silence before the request. Returns
   true on success. */
bool rtuWrite(int address, int reg, uint16_t value) {
  if (isSimCommLost(address)) {
    return false;  // Simulated communication loss, request fails
  }
  rtuWaitBusIdle();
  bool ok = ModbusRTUClient.holdingRegisterWrite(address, reg, value);
  rtuLastActivityUs = micros();
  return ok;
}

void clearVfdFault(Gate* g) {
  // Speed register needs to be written to after fault. Value does not matter.
  rtuWrite(g->vfdModbusRtuAddress, VFD_REG_LFR, 0);
  // Reset VFD && disable fast stop function (must be done before start request can be sent)
  // Fast stop bit is inverted: 1 = fast stop NOT active.
  // Command word: bit 2 = fault reset, bit 3 = disable fast stop
  uint16_t resetCmd = (1 << VFD_CMD_BIT_FAULT_RESET) | (1 << VFD_CMD_BIT_FAST_STOP);
  if (rtuWrite(g->vfdModbusRtuAddress, VFD_REG_CMD, resetCmd)) {
    g->simVfdFault = false;  // Simulated VFD fault is reset like a real one
  }
  if (debug) {
    printGateNumber(g->vfdModbusRtuAddress);
    Serial.println("VFD reset performed!");
  }
}

/* PLC startup: poll both VFDs until they have answered a status read, or
   VFD_STARTUP_TIMEOUT. Continuous polling also prevents a long pause in
   Modbus RTU traffic (VFD communication monitoring). A failed read takes
   the ModbusRTUClient timeout, which paces the retries. */
void waitForVfds() {
  unsigned long start = millis();
  while (true) {
    if (gate1.vfdStatusSeq == 0) {
      gate1.forceReadVfdStatus = true;
      readVfdStatus(&gate1, millis());
    }
    if (gate2.vfdStatusSeq == 0) {
      gate2.forceReadVfdStatus = true;
      readVfdStatus(&gate2, millis());
    }
    if (gate1.vfdStatusSeq != 0 && gate2.vfdStatusSeq != 0) {
      Serial.print(F("Both VFDs answered after [ms]: "));
      Serial.println(millis() - start);
      return;
    }
    if (millis() - start > VFD_STARTUP_TIMEOUT) {
      // Gate without answer enters FAULT_VFD_COMM in the first loop
      gate1.vfdCommErrorCounter = gate1.vfdStatusSeq ? 0 : VFD_COMM_ERROR_FAULT;
      gate2.vfdCommErrorCounter = gate2.vfdStatusSeq ? 0 : VFD_COMM_ERROR_FAULT;
      Serial.println(F("VFD startup timeout. At least one VFD did not answer!"));
      return;
    }
    delay(50);
  }
}

/* PLC startup: reset a VFD reporting a fault, up to VFD_STARTUP_RESET_ATTEMPTS
   times. A remaining fault is handled by the state machine (FAULT_VFD). */
void resetVfdFaultAtStartup(Gate *g) {
  for (uint8_t attempt = 1;
       attempt <= VFD_STARTUP_RESET_ATTEMPTS && g->vfdStatusSeq != 0 && g->vfd.fault;
       attempt++) {
    printGateNumber(g->vfdModbusRtuAddress);
    Serial.print(F("VFD fault at startup, reset attempt "));
    Serial.println(attempt);
    clearVfdFault(g);         // Fault reset bit 0 -> 1 (writeVfdStop before)
    delay(VFD_STARTUP_RESET_DELAY);
    writeVfdStop(g);          // Fault reset bit back to 0 for next attempt
    uint32_t seq = g->vfdStatusSeq;
    g->forceReadVfdStatus = true;
    readVfdStatus(g, millis());
    if (g->vfdStatusSeq == seq) {
      return; // Read failed, handled by communication fault detection
    }
  }
  if (g->vfdStatusSeq != 0 && g->vfd.fault) {
    printGateNumber(g->vfdModbusRtuAddress);
    Serial.println(F("VFD fault remains after startup reset."));
  }
}

/* Unconditional stop command (no run, fast stop not active) */
void writeVfdStop(Gate* g) {
  rtuWrite(g->vfdModbusRtuAddress, VFD_REG_LFR, 0);
  rtuWrite(g->vfdModbusRtuAddress, VFD_REG_CMD, (1 << VFD_CMD_BIT_FAST_STOP));
}

void readVfdStatus(Gate* g, unsigned long currentMillis) {
  // Retry faster after a failed read, but never on every loop. A VFD that does
  // not respond costs ModbusRTUClient timeout for every attempt.
  unsigned long interval = g->vfdCommErrorCounter ? VFD_READ_RETRY_INTERVAL : VFD_READ_STATUS_INTERVAL;
  if (!(currentMillis - g->timerReadVfdStatus > interval || g->forceReadVfdStatus)) {
    return;
  }
  g->timerReadVfdStatus = currentMillis;
  g->forceReadVfdStatus = false;

  // Read VFD status
  bool readFailed = false;
  rtuRequestFrom(g->vfdModbusRtuAddress, VFD_REG_NM1, 2);
  if (ModbusRTUClient.available() == 2) {
    // Success! Safe to pull data from internal buffer
    g->vfd.regETA = (int16_t)ModbusRTUClient.read();
    g->vfd.regRFR = (int16_t)ModbusRTUClient.read();
    if (debug > 1) {
      printGateNumber(g->vfdModbusRtuAddress);
      Serial.print("ETA: ");
      Serial.print(g->vfd.regETA);
      Serial.print(" RFR: ");
      Serial.println(g->vfd.regRFR);
    }
  } else {
    // Request did not return the amount of requested values.
    readFailed = true;
    clearModbusRtuBuffer();
    if (debug) {
      printGateNumber(g->vfdModbusRtuAddress);
      Serial.print("Failed to read Modbus RTU register NM1! Error: ");
      Serial.println(ModbusRTUClient.lastError());
    }
  }

  if (!readFailed) {
    rtuRequestFrom(g->vfdModbusRtuAddress, VFD_REG_NC1, 2);
    if (ModbusRTUClient.available() == 2) {
      // Success! Safe to pull data from internal buffer
      g->vfd.regCMD = (int16_t)ModbusRTUClient.read();
      g->vfd.regLFR = (int16_t)ModbusRTUClient.read();
      if (debug > 1) {
        printGateNumber(g->vfdModbusRtuAddress);
        Serial.print("CMD: ");
        Serial.print(g->vfd.regCMD);
        Serial.print(" LFR: ");
        Serial.println(g->vfd.regLFR);
      }
    } else {
      // Request did not return the amount of requested values.
      readFailed = true;
      clearModbusRtuBuffer();
      if (debug) {
        printGateNumber(g->vfdModbusRtuAddress);
        Serial.print("Failed to read Modbus RTU register NC1! Error: ");
        Serial.println(ModbusRTUClient.lastError());
      }
    }
  }

  if (readFailed) {
    if (g->vfdCommErrorCounter < VFD_COMM_ERROR_FAULT) {
      g->vfdCommErrorCounter++;
    }
    return;
  }

  if (g->simVfdFault) {
    g->vfd.regETA |= (1 << 3);  // Simulated VFD fault (ETA bit 3)
  }
  parseVfdStatus(g);
  g->vfdCommErrorCounter = 0;
  g->vfdStatusSeq++;

  // If requested, read user requested register
  // Default register is 0. Requested register 0 = function disabled.
  // Eventual read fail does not contribute to vfdCommErrorCounter
  long reg = modbusTCPServer.holdingRegisterRead(g->hregVfdRequest);
  if (reg > 0) {
    rtuRequestFrom(g->vfdModbusRtuAddress, (int)reg, 1);
    if (ModbusRTUClient.available() == 1) {
      g->vfd.regReqValue = (uint16_t)ModbusRTUClient.read();
    } else {
      g->vfd.regReqValue = 0;
      clearModbusRtuBuffer();
      if (debug) {
        printGateNumber(g->vfdModbusRtuAddress);
        Serial.print("Failed to read Modbus RTU user requested register! Error: ");
        Serial.println(ModbusRTUClient.lastError());
      }
    }
  }
}

/* Discard values left in the library's buffer of decoded register values.
   Bytes left in the RS485 receive buffer are discarded by rtuWaitBusIdle(). */
void clearModbusRtuBuffer() {
  while (ModbusRTUClient.available()) {
    ModbusRTUClient.read();
  }
}

void setVfdState(Gate* g) {
  // VFD cannot be reached. Do not block the loop with write timeouts, motor
  // torque is removed via STO in STATE_FAULT.
  if (g->vfdCommErrorCounter >= VFD_COMM_ERROR_FAULT) {
    return;
  }

  // Release fast stop once motor has stopped
  if (g->fastStop && motorStopped(g)) {
    g->fastStop = false;
  }

  int speed = g->fastStop ? 0 : g->currentSpeed;

  // The VFD ignores a run command sent in the same write as the fast stop
  // release (CMD 0 -> 9). Release fast stop with a stop command first
  // (CMD 0 -> 8) and send the run command once a status read confirms the
  // VFD has fast stop released (CMD 8 -> 9 = new run command).
  // Happens when retracting after a safety edge activation.
  bool vfdFastStopActive = !(g->vfd.regCMD & (1 << VFD_CMD_BIT_FAST_STOP));
  if (speed != 0 && vfdFastStopActive) {
    speed = 0;
  }

  if (speed == 0 && g->lastSpeedCmd != 0) {
    g->stopCmdSeq = g->vfdStatusSeq;  // Motor stop is confirmed by a later status read
  }
  g->lastSpeedCmd = speed;

  uint16_t tmp = 0;
  if (speed == 0) {
    // Stop (fast stop or normal ramp stop)
    // Fast stop bit is inverted: 1 = fast stop NOT active
    tmp = ((!g->fastStop) << VFD_CMD_BIT_FAST_STOP);
  } else {
    // Run
    tmp = (1 << VFD_CMD_BIT_RUN) | (1 << VFD_CMD_BIT_FAST_STOP);
  }

  // Compare against currently set values
  if (speed != g->vfd.regLFR) {
    // Write new value
    if (!rtuWrite(g->vfdModbusRtuAddress, VFD_REG_LFR, speed)) {
      if (g->vfdCommErrorCounter < VFD_COMM_ERROR_FAULT) {
        g->vfdCommErrorCounter++;
      }
    }
    // Force VFD status update at next state machine parsing
    g->forceReadVfdStatus = true;
    if (debug) {
      printGateNumber(g->vfdModbusRtuAddress);
      Serial.print("New value: VFD speed set to [Hz/10]: ");
      Serial.println(speed);
    }
  }
  if (tmp != (uint16_t)g->vfd.regCMD) {
    // Write new value
    if (!rtuWrite(g->vfdModbusRtuAddress, VFD_REG_CMD, tmp)) {
      if (g->vfdCommErrorCounter < VFD_COMM_ERROR_FAULT) {
        g->vfdCommErrorCounter++;
      }
    }
    // Force VFD status update at next state machine parsing
    g->forceReadVfdStatus = true;
    if (debug) {
      printGateNumber(g->vfdModbusRtuAddress);
      Serial.print("New value: VFD CMD set to: ");
      Serial.println(tmp);
    }
  }
}

/* SERIAL DEBUG COMMANDS -----------------------------------------------------*/

/* Collect characters received over USB serial into a command line. Command is
   executed at end of line, or SERIAL_CMD_TIMEOUT after last character. */
void handleSerialCommands(unsigned long currentMillis) {
  static char buf[SERIAL_CMD_MAX_LEN + 1];
  static uint8_t len = 0;
  static bool overflow = false;  // Line too long, discard until end of line
  static unsigned long lastCharTime = 0;

  while (Serial.available()) {
    char c = Serial.read();
    lastCharTime = currentMillis;
    if (c == '\r' || c == '\n') {
      if (len > 0 && !overflow) {
        buf[len] = '\0';
        executeSerialCommand(buf);
      }
      len = 0;
      overflow = false;
    } else if (len < SERIAL_CMD_MAX_LEN) {
      buf[len++] = toupper(c);
    } else {
      overflow = true;
    }
  }

  // Terminal without line ending
  if (len > 0 && currentMillis - lastCharTime > SERIAL_CMD_TIMEOUT) {
    if (!overflow) {
      buf[len] = '\0';
      executeSerialCommand(buf);
    }
    len = 0;
    overflow = false;
  }
}

void executeSerialCommand(const char *cmd) {
  if (strcmp(cmd, "DEBUG") == 0 || strcmp(cmd, "DEBUG 1") == 0) {
    debug = 1;
  } else if (strcmp(cmd, "DEBUG 2") == 0) {
    debug = 2;
  } else if (strcmp(cmd, "DEBUG OFF") == 0 || strcmp(cmd, "DEBUG 0") == 0) {
    debug = 0;
  } else if (executeFaultCommand(cmd)) {
    return;
  } else {
    Serial.print(F("Unknown command: "));
    Serial.println(cmd);
    Serial.println(F("Commands: DEBUG, DEBUG 2, DEBUG OFF,"));
    printFaultHelp();
    return;
  }
  Serial.print(F("Debug level set to "));
  Serial.println(debug);
}

/* FAULT SIMULATION ----------------------------------------------------------*/
/* See FAULT SIMULATION in header. */

/* Execute a FAULT command. Returns false if cmd is not a FAULT command. */
bool executeFaultCommand(const char *cmd) {
  if (strncmp(cmd, "FAULT", 5) != 0 || (cmd[5] != '\0' && cmd[5] != ' ')) {
    return false;
  }
  const char *arg = cmd + 5;
  while (*arg == ' ') {
    arg++;
  }

  if (*arg == '\0') {
    printSimulationStatus(&gate1);
    printSimulationStatus(&gate2);
    return true;
  }
  if (strcmp(arg, "CLEAR") == 0) {
    clearSimulatedFaults(&gate1);
    clearSimulatedFaults(&gate2);
    return true;
  }

  // FAULT <gate> <code|CLEAR>
  Gate *g = (arg[0] == '1') ? &gate1 : (arg[0] == '2') ? &gate2 : nullptr;
  const char *val = arg + 1;
  if (g == nullptr || *val != ' ') {
    printFaultHelp();
    return true;
  }
  while (*val == ' ') {
    val++;
  }
  if (strcmp(val, "CLEAR") == 0) {
    clearSimulatedFaults(g);
  } else if (val[0] >= '1' && val[0] <= '7' && val[1] == '\0') {
    simulateFault(g, (FaultCode)(val[0] - '0'));
  } else {
    printFaultHelp();
  }
  return true;
}

void printFaultHelp() {
  Serial.println(F("FAULT, FAULT <gate 1-2> <code 1-7>, FAULT <gate 1-2> CLEAR, FAULT CLEAR"));
}

/* Inject the cause of a fault, see FAULT SIMULATION in header */
void simulateFault(Gate *g, FaultCode code) {
  printGateNumber(g->vfdModbusRtuAddress);
  switch (code) {
    case FAULT_VFD:
      g->simVfdFault = true;
      Serial.println(F("Simulating VFD fault (cleared by fault reset)."));
      break;
    case FAULT_VFD_COMM:
      g->simCommLost = true;
      Serial.println(F("Simulating VFD communication loss until FAULT CLEAR."));
      break;
    case FAULT_POSITION_SENSORS:
      g->simSensorsInvalid = true;
      Serial.println(F("Simulating invalid position sensors until FAULT CLEAR."));
      break;
    default:  // Event faults 3, 4, 6, 7
      if (g->state == STATE_FAULT) {
        Serial.println(F("Already in fault state, simulation ignored."));
        return;
      }
      g->simEventFault = code;
      Serial.print(F("Simulating fault "));
      Serial.println(code);
      break;
  }
}

/* Remove simulated fault causes. A latched fault still requires a reset. */
void clearSimulatedFaults(Gate *g) {
  g->simVfdFault = false;
  g->simCommLost = false;
  g->simSensorsInvalid = false;
  g->simEventFault = FAULT_NONE;
  printGateNumber(g->vfdModbusRtuAddress);
  Serial.println(F("Simulated fault causes cleared."));
}

void printSimulationStatus(Gate *g) {
  printGateNumber(g->vfdModbusRtuAddress);
  Serial.print(stateName(g->state));
  Serial.print(F(", fault code "));
  Serial.print(g->faultCode);
  Serial.print(F(", simulated causes:"));
  if (g->simVfdFault) {
    Serial.print(F(" 1 VFD"));
  }
  if (g->simCommLost) {
    Serial.print(F(" 2 COMM"));
  }
  if (g->simSensorsInvalid) {
    Serial.print(F(" 5 SENSORS"));
  }
  if (!g->simVfdFault && !g->simCommLost && !g->simSensorsInvalid) {
    Serial.print(F(" none"));
  }
  Serial.println();
}

/* True if communication loss is simulated for the VFD at Modbus RTU address */
bool isSimCommLost(int address) {
  return (address == gate1.vfdModbusRtuAddress && gate1.simCommLost) ||
         (address == gate2.vfdModbusRtuAddress && gate2.simCommLost);
}

/* UTILITY FUNCTIONS ---------------------------------------------------------*/
void handleIndicationLamps() {
  bool anyGateMoving = (
                         gate1.state == STATE_OPENING ||
                         gate1.state == STATE_CLOSING ||
                         gate1.state == STATE_RETRACTING ||
                         gate2.state == STATE_OPENING ||
                         gate2.state == STATE_CLOSING ||
                         gate2.state == STATE_RETRACTING);

  bool anyGatePending = (
                          gate1.state == STATE_PENDING_MOVE ||
                          gate2.state == STATE_PENDING_MOVE);

  /* Lamp(s) are currently controlled by relays.
    Relays and current type of lamps are not suitable for very frequent blinking. */
  if (anyGateMoving) {
    digitalWrite(RELAY_LAMP_WARNING, HIGH); // Permanently on
  } else if (anyGatePending) {
    digitalWrite(RELAY_LAMP_WARNING, (millis() / 1000) % 2); // Flash, 1 s on, 1 s off
  } else {
    digitalWrite(RELAY_LAMP_WARNING, LOW);
  }
}

/* Count objects passing the light barrier. Returns true during the loop an
   object has passed (light barrier released after being obstructed longer
   than TIME_LIGHT_BARRIER_THRESHOLD). */
bool handleTrafficMonitor(unsigned long currentMillis) {
  bool passed = false;
  // Light barrier signal LOW = light barrier blocked/obstructed
  bool lightBarrierCurrentState = !lightBarrierObstructed;
  if (lightBarrierCurrentState) {
    if (!lightBarrierPrevState) {
      if (currentMillis - lightBarrierTimerStart > TIME_LIGHT_BARRIER_THRESHOLD) {
        // An object has passed through the gate.
        passed = true;
        lastPassageTime = currentMillis;
        if (debug) {
          Serial.println("Light barrier NOT obstructed.");
        }
        // Increment traffic counter if at least one gate is partially open.
        if (!gate1.isFullyClosed || !gate2.isFullyClosed) {
          long tmp = modbusTCPServer.holdingRegisterRead(HREG_TRAFFIC_COUNTER);
          modbusTCPServer.holdingRegisterWrite(HREG_TRAFFIC_COUNTER, (uint16_t)++tmp);
          if (debug) {
            Serial.println("Incremented traffic counter.");
          }
        }
      }
    }
  } else {
    if (lightBarrierPrevState) {
      lightBarrierTimerStart = currentMillis;
      if (debug) {
        Serial.println("Light barrier obstructed.");
      }
    }
  }
  lightBarrierPrevState = lightBarrierCurrentState;
  return passed;
}

/* Count down holding register 0 [s] towards 0 while at least one gate is in
   STATE_STAY_OPEN (gate states from previous loop). Limited to
   TIME_STAY_OPEN_MAX. The value is kept unchanged while no gate is in
   STATE_STAY_OPEN. */
void updateStayOpenCountdown(unsigned long currentMillis) {
  static unsigned long lastTick = 0;
  bool counting = (gate1.state == STATE_STAY_OPEN || gate2.state == STATE_STAY_OPEN);
  if (!counting) {
    // First count down 1 s after a gate has entered STATE_STAY_OPEN
    lastTick = currentMillis;
    return;
  }
  if (currentMillis - lastTick < 1000) {
    return;
  }
  lastTick = currentMillis;
  long t = modbusTCPServer.holdingRegisterRead(HREG_STAY_OPEN_COUNTDOWN);
  if (t > TIME_STAY_OPEN_MAX) {
    t = TIME_STAY_OPEN_MAX;
  }
  if (t > 0) {
    t--;
  }
  modbusTCPServer.holdingRegisterWrite(HREG_STAY_OPEN_COUNTDOWN, (uint16_t)t);
}

/* Raise holding register 0 to TIME_STAY_OPEN_DEFAULT if lower. A higher value
   (e.g. written by the Modbus master) is kept. */
void raiseStayOpenCountdown() {
  long t = modbusTCPServer.holdingRegisterRead(HREG_STAY_OPEN_COUNTDOWN);
  if (t < TIME_STAY_OPEN_DEFAULT) {
    modbusTCPServer.holdingRegisterWrite(HREG_STAY_OPEN_COUNTDOWN, TIME_STAY_OPEN_DEFAULT);
    t = TIME_STAY_OPEN_DEFAULT;
  }
  if (debug) {
    Serial.print(F("Stay open countdown [s]: "));
    Serial.println(t);
  }
}

/* Reset holding register 0 to 0 once, when the last active gate cycle has
   ended (no gate has cycleActive any more). Must be called after the state
   machines. */
void resetStayOpenCountdownAtCycleEnd() {
  static bool anyCycleActivePrev = false;
  bool anyCycleActive = gate1.cycleActive || gate2.cycleActive;
  if (!anyCycleActive && anyCycleActivePrev) {
    modbusTCPServer.holdingRegisterWrite(HREG_STAY_OPEN_COUNTDOWN, 0);
    if (debug) {
      Serial.println(F("Last gate cycle ended. Stay open countdown reset."));
    }
  }
  anyCycleActivePrev = anyCycleActive;
}

/* True if the gate is prevented from moving towards its target by a safety
   edge or the light barrier, see TIME_OBSTRUCTED_WARNING. */
bool gateObstructed(Gate *g) {
  switch (g->state) {
    case STATE_PENDING_MOVE:
    case STATE_OPENING:
    case STATE_CLOSING:
    case STATE_WAIT_LIGHT_BARRIER:
    case STATE_RETRACTING:
    case STATE_WAIT_AFTER_RETRACT:
      return g->seInnerHit || g->seOuterHit ||
             (g->target == TARGET_CLOSED && lightBarrierObstructed);
    default:
      return false;
  }
}

/* Coil 8: set every TIME_OBSTRUCTED_WARNING while a gate is continuously
   obstructed. Cleared by the Modbus client after reading, or here when the
   gate(s) that set it have reached their target. Must be called after the
   state machines. */
void handleObstructionWarning(unsigned long currentMillis) {
  Gate *gates[] = { &gate1, &gate2 };
  bool targetReached = false;

  for (Gate *g : gates) {
    if (g->obstructionWarning && !movementRequired(g)) {
      g->obstructionWarning = false;
      targetReached = true;
    }

    if (!gateObstructed(g)) {
      g->timerObstructed = currentMillis;
    } else if (currentMillis - g->timerObstructed >= TIME_OBSTRUCTED_WARNING) {
      // Report again every TIME_OBSTRUCTED_WARNING while obstruction continues
      g->timerObstructed = currentMillis;
      g->obstructionWarning = true;
      modbusTCPServer.coilWrite(COIL_OBSTRUCTED, 1);
      if (debug) {
        printGateNumber(g->vfdModbusRtuAddress);
        Serial.println(F("Gate obstructed for a long time. Warning coil set."));
      }
    }
  }

  if (targetReached && !gate1.obstructionWarning && !gate2.obstructionWarning) {
    modbusTCPServer.coilWrite(COIL_OBSTRUCTED, 0);
    if (debug) {
      Serial.println(F("Target reached. Obstruction warning coil cleared."));
    }
  }
}

void updateModbusTCPRegisters() {
  // See document "plc-modbus-tcpip-server-setup.ods" for reference
  //int inputRegisterWrite(int address, uint16_t value)

  // PLC fault: any gate fault not caused by the VFD itself
  bool plcFault =
    (gate1.state == STATE_FAULT && gate1.faultCode != FAULT_VFD) ||
    (gate2.state == STATE_FAULT && gate2.faultCode != FAULT_VFD);

  // Gate status summary
  modbusTCPServer.inputRegisterWrite(IREG_STATUS_SUMMARY,
                                     ((uint16_t)(gate1.isFullyOpen != 0)   << 0)
                                     | ((uint16_t)(gate1.isFullyClosed != 0) << 1)
                                     | ((uint16_t)(gate1.vfd.fault != 0)     << 2)
                                     | ((uint16_t)(gate1.state & 0b00001111) << 3)
                                     | ((uint16_t)(gate2.isFullyOpen != 0)   << 7)
                                     | ((uint16_t)(gate2.isFullyClosed != 0) << 8)
                                     | ((uint16_t)(gate2.vfd.fault != 0)     << 9)
                                     | ((uint16_t)(gate2.state & 0b00001111) << 10)
                                     | ((uint16_t)(forceOpen != 0)           << 14)
                                     | ((uint16_t)plcFault                   << 15));

  // PLC input status
  modbusTCPServer.inputRegisterWrite(1,
                                     ((uint16_t)digitalRead(CONTROLLINO_A0) << 0)
                                     | ((uint16_t)digitalRead(CONTROLLINO_A1) << 1)
                                     | ((uint16_t)digitalRead(CONTROLLINO_A2) << 2)
                                     | ((uint16_t)digitalRead(CONTROLLINO_A3) << 3)
                                     | ((uint16_t)digitalRead(CONTROLLINO_A4) << 4)
                                     | ((uint16_t)digitalRead(CONTROLLINO_A5) << 5)
                                     | ((uint16_t)digitalRead(CONTROLLINO_A6) << 6)
                                     | ((uint16_t)digitalRead(CONTROLLINO_A7) << 7)
                                     | ((uint16_t)digitalRead(CONTROLLINO_A8) << 8)
                                     | ((uint16_t)digitalRead(CONTROLLINO_A9) << 9)
                                     | ((uint16_t)digitalRead(CONTROLLINO_IN0) << 10)
                                     | ((uint16_t)digitalRead(CONTROLLINO_IN1) << 11)
                                     | ((uint16_t)digitalRead(CONTROLLINO_PIN_HEADER_DIGITAL_IN_10) << 12)
                                     | ((uint16_t)digitalRead(CONTROLLINO_PIN_HEADER_DIGITAL_IN_11) << 13)
                                     | ((uint16_t)digitalRead(CONTROLLINO_PIN_HEADER_DIGITAL_IN_12) << 14)
                                     | ((uint16_t)digitalRead(CONTROLLINO_PIN_HEADER_DIGITAL_IN_13) << 15));

  // PLC output status
  modbusTCPServer.inputRegisterWrite(2, 0); // Currently not in use!!

  // PLC relay status
  modbusTCPServer.inputRegisterWrite(3,
                                     (digitalRead(CONTROLLINO_R0) << 0)
                                     | (digitalRead(CONTROLLINO_R1) << 1)
                                     | (digitalRead(CONTROLLINO_R2) << 2)
                                     | (digitalRead(CONTROLLINO_R3) << 3)
                                     | (digitalRead(CONTROLLINO_R4) << 4)
                                     | (digitalRead(CONTROLLINO_R5) << 5)
                                     | (digitalRead(CONTROLLINO_R6) << 6)
                                     | (digitalRead(CONTROLLINO_R7) << 7)
                                     | (digitalRead(CONTROLLINO_R8) << 8)
                                     | (digitalRead(CONTROLLINO_R9) << 9));

  modbusTCPServer.inputRegisterWrite(4, gate1.vfd.regETA);
  modbusTCPServer.inputRegisterWrite(5, gate2.vfd.regETA);
  modbusTCPServer.inputRegisterWrite(6, gate1.vfd.regReqValue);
  modbusTCPServer.inputRegisterWrite(7, gate2.vfd.regReqValue);

  // Latched fault code, see enum FaultCode
  modbusTCPServer.inputRegisterWrite(IREG_G1_FAULT_CODE, gate1.faultCode);
  modbusTCPServer.inputRegisterWrite(IREG_G2_FAULT_CODE, gate2.faultCode);
}

void handleFloodlight() {
  digitalWrite(RELAY_FLOODLIGHT, modbusTCPServer.coilRead(COIL_FLOODLIGHT));
  // TODO: Safety: Prevent from changing state too often
}

const __FlashStringHelper* stateName(GateState s) {
  switch (s) {
    case STATE_IDLE:               return F("STATE_IDLE");
    case STATE_PENDING_MOVE:       return F("STATE_PENDING_MOVE");
    case STATE_OPENING:            return F("STATE_OPENING");
    case STATE_CLOSING:            return F("STATE_CLOSING");
    case STATE_STAY_OPEN:          return F("STATE_STAY_OPEN");
    case STATE_WAIT_LIGHT_BARRIER: return F("STATE_WAIT_LIGHT_BARRIER");
    case STATE_RETRACTING:         return F("STATE_RETRACTING");
    case STATE_WAIT_AFTER_RETRACT: return F("STATE_WAIT_AFTER_RETRACT");
    case STATE_FAULT:              return F("STATE_FAULT");
    case STATE_STOPPED:            return F("STATE_STOPPED");
  }
  return F("UNKNOWN");
}

void printGateNumber(int number) {
  Serial.print("Gate "); Serial.print(number); Serial.print(": ");
}
