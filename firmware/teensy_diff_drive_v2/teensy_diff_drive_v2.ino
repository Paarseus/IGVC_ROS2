// ============================================================================
// Teensy 4.1 — AVROS diff-drive motor bridge v2d (REV SPARK MAX FW 26.1.5)
// ============================================================================
// Role: USB-Serial <-> CAN bridge for precise motor tuning. The Jetson
// (ROS2 actuator_node) owns the diff-drive kinematics and streams per-wheel
// RPM setpoints; this firmware forwards them to each SPARK MAX's on-board
// velocity PID, confirms every parameter write with the controller's own
// PARAMETER_WRITE_RESPONSE, decodes STATUS_0/1/2/7/8, and echoes feedback.
// v2d (2026-09-28): adapted to SPARK MAX FW 26.1.5 per
// research/evidence/firmware_can_review_2026_09_27/FW26_CHANGES_2026_09_28.md §8:
// STATUS_7/8 telemetry, CHK configuration check, motor-type interlock, kS from
// one source (Teensy arbFF), kA warning. See PROTOCOL.md "v2d changes".
//
// Spec sources (all in research/evidence/firmware_can_review_2026_09_27/
// references/, abbreviated below):
//   JSON  = rev_2026_spark_frames_2.1.0.json         (REV-Specs 2.1.0)
//   HDR   = rev_2026_revlib_driver_2026.0.5_CANSparkFrames.h (REVLib-driver
//           2026.0.5, kMinFirmwareVersion v26.0.0). Every frame ID and DLC
//           used here is identical in JSON and HDR (checked programmatically).
//   PARAM = rev_2026_revlib_java_2026.0.5_SparkParameters.java
//   RST   = wpilib_2026_frc_can_device_spec_can_addressing.rst
// Cited as FILE:line. See PROTOCOL.md for the full frame table + changelog.
//
// Hardware:
//   Teensy 4.1  (CAN1: CTX1=pin22, CRX1=pin23), 1 Mbit/s, 29-bit IDs only
//   SN65HVD230 or TJA1051T/3 transceiver, 120 ohm termination at each end
//   REV SPARK MAX x 2 -- CAN ID 1 = left, ID 2 = right, NEO hall sensor
//
// Host -> Teensy (USB CDC, newline-terminated). v1 commands, unchanged:
//   L<rpm> R<rpm>     VELOCITY setpoints (slew-ramped, MAX_RPM clamp)
//   UL<d> UR<d>       DUTY setpoints, clamped to the duty cap (default 0.30)
//   S                 stop: velocity mode at 0 RPM, bypasses the ramp
//   D                 one DIAG line (v1 fields first, new fields appended)
//   K<P|I|D|F|Z><v>   write kP(13)/kI(14)/kD(15)/kV(16)/kIZone(17) to BOTH
//                     SPARKs. F keeps its v1 letter/reply ("OK KF=") but
//                     param 16 = kV, V/RPM on FW 26 (was kF duty/RPM on FW 25).
//                     Prints "OK K<x>=<v>" when queued (v1 text);
//                     the SPARKs' confirmations follow as PWR lines.
//   M<v>              velocity slew, RPM per 20 ms tick (default 100)
//   BURN              safe persist (see below); "OK BURN result L=<c> R=<c>"
//   A1 / A0           IGVC §I.2 safety light flash / solid
// New in v2:
//   K<P|I|D|F|V|Z|A><L|R><v>  one side only (F and V both = kV id 16,
//                     A = kA id 205: MAXMotion only on FW 26, reply adds a
//                     "# KA:" warning). K<A><v> = both sides.
//   KS[L|R]<volts>    v2c static-friction kS, Teensy-side ARBITRARY_FEEDFORWARD
//                     (the ONLY kS source; native param 204 must stay 0)
//   CHK               v2d configuration check of both SPARKs (also runs ~2 s
//                     after boot); "CHK <L|R|B> ..." lines, then CHK OK|FAIL
//   PW <L|R|B> <id|name> [f|u|i|b] <value>   typed PARAMETER_WRITE
//   PR <L|R|B> <id|name>                     parameter READ (RTR)
//   PT                print the built-in parameter table
//   FV                GET_FIRMWARE_VERSION from both SPARKs
//   CF [L|R|B]        CLEAR_FAULTS (sticky faults)
//   ID <L|R|B>        IDENTIFY blink
//   X1 / X0           high-rate telemetry line on / off (default off)
//   UVL<V> UVR<V>     VOLTAGE setpoints, clamped to 12 V x duty cap
//   MD<d>             runtime duty cap (0..0.6, default 0.30; volt cap = 12*d)
//
// Teensy -> Host:
//   E L<rpm> <pos> R<rpm> <pos>   50 Hz wheel feedback (format unchanged)
//   OK / ERR / DIAG / #           as v1
//   PWR <L|R> id=.. type=.. val=.. res=.. <name> <result>  write confirmation
//   PWR <L|R> id=.. TIMEOUT
//   PRD <L|R> id=.. ...  |  PRD <L|R> id=.. TIMEOUT|UNSUPPORTED
//   FV <L|R> <maj>.<min>.<build> ...  |  FV <L|R> TIMEOUT
//   F <L|R> f=.. w=.. sf=.. sw=.. ...   printed whenever STATUS_1 bits change
//   SSE <L|R> res=.. mask=.. en=..      SET_STATUSES_ENABLED response (on change)
//   X <us> L ... R ... SP ... <mode> I7 .. SP8 ..   telemetry, one line per new STATUS_2
//   !! MOTOR TYPE <n> on <L|R>: motion blocked ...  motor-type interlock (v2d)
//
// Safety:
//   * 300 ms host watchdog: no L/R/S/U* in that window -> both wheels forced
//     to 0 (velocity/duty/voltage), unramped.
//   * SPARK 100 ms heartbeat timeout (RST:249) is the second layer. The
//     heartbeat is sent from the 20 ms tick, which nothing in loop() blocks.
//   * v2d motor-type interlock (EXPECT_BRUSHLESS): a SPARK whose param 2 reads
//     back != 1 (brushless) only ever gets duty 0. Unknown (no read yet) = allowed.
//   * MAX_RPM clamp on velocity setpoints; duty/voltage cap (hard max 0.6 /
//     7.2 V). MAX_RPM limits only the setpoint, not the PID's output.
//   * BURN: refused unless both wheels are ~stopped (fresh STATUS_2). Then the
//     universal and secondary heartbeats are sent as all-zero (disabled) frames,
//     the firmware waits for STATUS_0 applied output ~0 (min 120 ms dwell; not
//     confirmed by 300 ms -> BURN aborts, nothing persisted), sends PERSIST_PARAMETERS,
//     waits (non-blocking, up to 1.5 s) for PERSIST_PARAMETERS_RESPONSE, then
//     restores the enabled heartbeat. A REV engineer states persist is refused
//     while the robot is enabled (wpilib_2024_2025beta_issue69_persist_error.md:247-249).
//   * No delay() anywhere after setup(); all CAN request/response exchanges
//     are state machines serviced from loop().
// ============================================================================

#include <FlexCAN_T4.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

// ---------- Configuration ----------------------------------------------------
static constexpr uint8_t  LEFT_ID        = 1;
static constexpr uint8_t  RIGHT_ID       = 2;
static constexpr uint32_t CAN_BAUD       = 1000000;
static constexpr uint32_t CTRL_DT_MS     = 20;    // 50 Hz control + heartbeat tick
static constexpr uint32_t FEEDBACK_DT_MS = 20;    // 50 Hz E-line to host
static constexpr uint32_t ENC_CFG_DT_MS  = 1000;  // re-send SET_STATUSES_ENABLED
static constexpr uint32_t WATCHDOG_MS    = 300;
// 4600 RPM = 1.527 m/s ground speed (4600 x 0.01994 / 60), matches the
// actuator_node max_linear_mps = 1.5 m/s with a small margin; 19% below NEO
// free speed (5676 RPM).
static constexpr float    MAX_RPM        = 4600.0f;

// Duty / voltage cap. Default identical to v1 MAX_DUTY; raised at run time
// with MD<d> up to the hard ceiling. Voltage cap = NOMINAL_V * duty cap.
static constexpr float    DUTY_CAP_DEFAULT = 0.30f;
static constexpr float    DUTY_CAP_HARD    = 0.60f;
static constexpr float    NOMINAL_V        = 12.0f;   // 0.6 * 12 = 7.2 V hard max

// Hardware RX filter: accept only data frames whose arbitration ID carries
// device type 2 / manufacturer 5 (bits 28..16 = 0x0205). The software check
// in onCanRx() is always applied as well. Set to 0 to fall back to v1's
// ACCEPT_ALL hardware filter (software filter still active).
#define USE_HW_RX_FILTER 1

// v2d motor-type interlock. The FW 26.1.5 update left param 2 (kMotorType) = 0
// (brushed) on both NEO controllers; an 8 % duty command then drew ~30 A into
// stalled motors. true: a SPARK whose param 2 reads back as anything but 1
// (brushless; sparkcan SparkBase.hpp:205-208, PDEF id 2 default 1) only gets
// duty-0 setpoints until a later read shows 1. Before the first successful
// read after boot motion is allowed (as v1).
static constexpr bool     EXPECT_BRUSHLESS = true;
static constexpr uint32_t MT_CHECK_MS      = 5000;   // periodic param-2 re-read
static constexpr uint32_t BOOT_CHK_DELAY_MS = 2000;  // automatic CHK after boot

// ---------- IGVC §I.2 safety light (Adafruit NeoPixel Ring 16) ----------------
// Unchanged from v1. show() is called ONLY on a state change or flash edge —
// never every loop — so the ~480 us IRQ blackout it incurs stays a tiny
// fraction of time and never disturbs the 50 Hz CAN heartbeat.
static constexpr uint8_t  LED_PIN         = 20;   // NeoPixel DIN via 470Ω (+ level shift at 5V)
static constexpr uint8_t  LED_COUNT       = 16;
static constexpr uint8_t  LED_R           = 255;  // amber
static constexpr uint8_t  LED_G           = 140;
static constexpr uint8_t  LED_B           = 0;
static constexpr uint8_t  LED_BRIGHT      = 190;  // ~75% of 255
static constexpr uint32_t FLASH_HALF_MS   = 250;  // 2 Hz, 50% duty
static constexpr uint32_t AUTO_TIMEOUT_MS = 750;  // no A-line in this window -> SOLID

// ---------- SPARK CAN protocol constants -------------------------------------
// Arbitration ID = devType<<24 | mfg<<16 | apiClass<<10 | apiIndex<<6 | devNum
// (RST "Addressing"; JSON:4 deviceTypeNumber 2 / manufacturerNumber 5).
static constexpr uint8_t  SPARK_DEV_TYPE = 2;    // JSON:4 deviceTypeNumber
static constexpr uint8_t  SPARK_MFG      = 5;    // JSON:4 manufacturerNumber
static constexpr uint32_t SPARK_ID_MASK  = 0x1FFF0000u;  // devType+mfg bits of a 29-bit ID
static constexpr uint32_t SPARK_ID_MATCH = ((uint32_t)SPARK_DEV_TYPE << 24) | ((uint32_t)SPARK_MFG << 16);

// Setpoints: DLC 8; float SETPOINT bits 0-31, ARBITRARY_FEEDFORWARD int16
// bits 32-47, PID_SLOT bits 48-49, FF units bit 50. We send slot 0, arbFF 0.
static constexpr uint8_t  CLS_SETPOINT   = 0;
static constexpr uint8_t  IDX_VELOCITY   = 0;    // VELOCITY_SETPOINT     JSON:716  HDR:48 (0x2050000) len HDR:378 (8)
static constexpr uint8_t  IDX_DUTY       = 2;    // DUTY_CYCLE_SETPOINT   JSON:764  HDR:49 (0x2050080) len HDR:379 (8)
static constexpr uint8_t  IDX_VOLTAGE    = 5;    // VOLTAGE_SETPOINT      JSON:851  HDR:51 (0x2050140) len HDR:381 (8); versionImplemented 25.0.0
static constexpr uint8_t  SETPOINT_DLC   = 8;

// SET_STATUSES_ENABLED: DLC 4 = uint16 MASK (bits 0-15) + uint16
// ENABLED_BITFIELD (bits 16-31). JSON:1025 HDR:55 (0x2050400) len HDR:385 (4).
// Bit n = STATUS_n: confirmed from REVLib 2026.0.5, which builds the bitfield
// as `|= 1 << statusIndex` (SparkFrameManager.cpp:80-134, FW26_CHANGES §2) and
// matches the c_Spark_kStatus0..7 enum (CANSparkDriver.h:96-103). Defaults:
// STATUS_0 enabledByDefault true (JSON:205), STATUS_1 true (JSON:417),
// STATUS_2/7/8 FALSE (JSON:451, :647, :665). v2d asserts STATUS_0,1,2,7,8
// (mask = enable = 0x0187) so a SPARK reboot cannot leave any off.
static constexpr uint8_t  CLS_CFG        = 1;
static constexpr uint8_t  IDX_SSE        = 0;    // SET_STATUSES_ENABLED          JSON:1025 HDR:55
static constexpr uint8_t  SSE_DLC        = 4;    // HDR:385 SPARK_SET_STATUSES_ENABLED_LENGTH (4u)
static constexpr uint16_t SSE_BITS       = 0x0187;  // bits 0,1,2,7,8 = STATUS_0,1,2,7,8
// Response DLC 5: RESULT_CODE u8 (0 ok, 1 unavailable frame), SPECIFIED_MASK
// u16 bits 8-23, ENABLED_BITFIELD u16 bits 24-39.
static constexpr uint8_t  IDX_SSE_RESP   = 1;    // SET_STATUSES_ENABLED_RESPONSE JSON:1040 HDR:56 (0x2050440) len HDR:386 (5)
// PERSIST_PARAMETERS_RESPONSE: DLC 1, RESULT_CODE "0 on success" (no other
// codes are defined by the spec).
static constexpr uint8_t  IDX_PERSIST_RESP = 4;  // PERSIST_PARAMETERS_RESPONSE   JSON:1089 HDR:57 (0x2050500) len HDR:387 (1)

// CLEAR_FAULTS: class 6 index 14, DLC 0.
static constexpr uint8_t  CLS_CLEAR_FAULTS = 6;  // CLEAR_FAULTS JSON:1215 HDR:63 (0x2051b80) len HDR:393 (0)
static constexpr uint8_t  IDX_CLEAR_FAULTS = 14;
// IDENTIFY: class 7 index 7, DLC 0.
static constexpr uint8_t  CLS_IDENTIFY   = 7;    // IDENTIFY JSON:1228 HDR:65 (0x2051dc0) len HDR:395 (0)
static constexpr uint8_t  IDX_IDENTIFY   = 7;
// GET_FIRMWARE_VERSION: class 9 index 8, rtr true, lengthBytes 8. Reply:
// MAJOR u8, MINOR u8, BUILD u16 BIG-endian, DEBUG_BUILD u8, HW_REV u8.
static constexpr uint8_t  CLS_FW         = 9;    // GET_FIRMWARE_VERSION JSON:1303 HDR:70 (0x2052600) len HDR:400 (8)
static constexpr uint8_t  IDX_FW         = 8;
// SECONDARY_HEARTBEAT: class 11 index 2, device 0, 64-bit enable bitfield.
// "only gets respected when the SPARK is not locked to the Universal
// Heartbeat" (JSON:1527); lock state = STATUS_0 PRIMARY_HEARTBEAT_LOCK bit 53.
static constexpr uint8_t  CLS_HB         = 11;   // SECONDARY_HEARTBEAT JSON:1525 HDR:79 (0x2052c80) len HDR:409 (8)
static constexpr uint8_t  IDX_HB         = 2;
// PARAMETER_WRITE: DLC 5 = PARAMETER_ID u8 + 32-bit VALUE whose type
// "depends on the Parameter Type" (JSON:5018). No type byte (FW 24's class-48
// frame had one; not valid on FW 25+).
static constexpr uint8_t  CLS_PARAM      = 14;
static constexpr uint8_t  IDX_PARAM_WRITE = 0;   // PARAMETER_WRITE JSON:5006 HDR:100 (0x2053800) len HDR:430 (5)
static constexpr uint8_t  PARAM_WRITE_DLC = 5;
// PARAMETER_WRITE_RESPONSE: DLC 7 = PARAMETER_ID u8, PARAMETER_TYPE u8
// (0 unused, 1 int, 2 uint, 3 float, 4 bool; JSON:5044), VALUE u32 (current value; does
// not match the request if the write failed), RESULT_CODE u8 (0 success,
// 1 invalid ID, 2 mismatched type, 3 access mode, 4 invalid, 5 not implemented; JSON:5068).
static constexpr uint8_t  IDX_PARAM_WRITE_RESP = 1;  // PARAMETER_WRITE_RESPONSE JSON:5032 HDR:101 (0x2053840) len HDR:431 (7)
// READ_PARAMETER_<2k>_AND_<2k+1>: apiClass 15..22, apiIndex 0..15, rtr true,
// lengthBytes 8, FIRST value bits 0-31, SECOND bits 32-63 (JSON:5082 ..;
// HDR:102 READ_PARAMETER_0_AND_1 0x2053c00 .. HDR:229 _254_AND_255 0x2055bc0,
// len HDR:432..559 all 8). pair k = id/2 -> class 15 + k/16, index k%16.
// UNCONFIRMED: every read frame's description says "SPARK MAX does not
// currently support this in v25.0.0-prerelease.4" (e.g. JSON:5388). Whether a
// MAX answers was open; observed working on 26.1.5 (FW26_CHANGES §5). A Flex on 26.1.6 answers only a genuine
// remote frame with DLC 8 (l5vel_2026_sparklib_PROTOCOL.md:69-72, community).
static constexpr uint8_t  CLS_READ_BASE  = 15;
static constexpr uint8_t  READ_RTR_DLC   = 8;
// Periodic status frames, class 46.
static constexpr uint8_t  CLS_STATUS     = 46;
static constexpr uint8_t  IDX_STATUS_0   = 0;    // STATUS_0 JSON:81  HDR:364 (0x205b800) len HDR:694 (8), default 10 ms
static constexpr uint8_t  IDX_STATUS_1   = 1;    // STATUS_1 JSON:207 HDR:365 (0x205b840) len HDR:695 (8), default 250 ms
static constexpr uint8_t  IDX_STATUS_2   = 2;    // STATUS_2 JSON:419 HDR:366 (0x205b880) len HDR:696 (8), default 20 ms, OFF by default
// STATUS_7 JSON:633 HDR:371 (0x205b9c0) len HDR:701 (8), default 20 ms, OFF by default (JSON:647).
//   I_ACCUMULATION float LE bits 0-31 (JSON:641); RESERVED int32 bits 32-63 (JSON:642).
static constexpr uint8_t  IDX_STATUS_7   = 7;
// STATUS_8 JSON:649 HDR:372 (0x205ba00) len HDR:702 (8), default 20 ms, OFF by default (JSON:665).
//   SETPOINT float LE bits 0-31 (JSON:657; RPM in velocity mode), IS_AT_SETPOINT bool bit 32
//   (JSON:658), SELECTED_PID_SLOT uint bits 33-36 (JSON:659), RESERVED bits 37-63 (JSON:660).
//   HDR spark_status_8_t (CANSparkFrames.h:14740-) has the same fields.
static constexpr uint8_t  IDX_STATUS_8   = 8;
// PERSIST_PARAMETERS: class 63 index 15, DLC 2, MAGIC_NUMBER u16 LE = 15011
// (0x3AA3 -> bytes A3 3A). "may take up to a second" (JSON:14884).
static constexpr uint8_t  CLS_PERSIST    = 63;   // PERSIST_PARAMETERS JSON:14882 HDR:375 (0x205ffc0) len HDR:705 (2)
static constexpr uint8_t  IDX_PERSIST    = 15;
static constexpr uint16_t PERSIST_MAGIC  = 15011; // JSON:14898 decodedMin/Max 15011
// Universal heartbeat (roboRIO RobotState, RST:188-249). The RST byte table numbers
// the bytes 1..8 with match time in byte 8, i.e. BIG-ENDIAN on the wire:
// SystemWatchdog (bit 28, "motor controllers are enabled") = data[4] bit 4 (0x10),
// Enabled (bit 25) = data[4] bit 1. Community implementations agree (team 195
// `receivedData[4] & 0x10`, UTNuclearRobotics/FRCCan buf[4]=0x18; see
// COMMUNITY_LIBRARIES_2026_09_28.md). v1's enabled frame (below) has data[4]=0x59,
// which sets 0x10 (and data[3]=0x12 sets the bits under a little-endian reading),
// so it enables under either byte order. Devices disable 100 ms after the last one (RST:249).
static constexpr uint32_t UNIVERSAL_HB   = 0x01011840;   // RST:188

// STATUS_0 signals (JSON:81 block).
static constexpr float    S0_APPLIED_SCALE = 3.082369457075716e-05f; // APPLIED_OUTPUT int16 bits 0-15  JSON:89
static constexpr float    S0_VOLT_SCALE    = 0.0073260073260073f;    // VOLTAGE uint12 bits 16-27 (V)  JSON:102
static constexpr float    S0_CURR_SCALE    = 0.0366300366300366f;    // CURRENT uint12 bits 28-39 (A)  JSON:114
// MOTOR_TEMPERATURE uint8 bits 40-47 degC (JSON:126); limit flags bits 48-51
// (JSON:138..), INVERTED bit 52 (JSON:186), PRIMARY_HEARTBEAT_LOCK bit 53
// (JSON:187-189), SPARK_MODEL uint4 bits 54-57 (JSON:199).

// PARAMETER types as encoded in PARAMETER_WRITE_RESPONSE (JSON:5032 block).
enum PType : uint8_t { PT_UNUSED = 0, PT_INT = 1, PT_UINT = 2, PT_FLOAT = 3, PT_BOOL = 4 };

// ---------- Built-in parameter table ----------------------------------------
// id + type from PARAM (SparkParameters.java 2026.0.5; line numbers given).
// Values are the RAW wire units (REVLib converts some before writing).
struct ParamInfo { uint8_t id; uint8_t type; const char *name; };
static const ParamInfo PARAMS[] = {
    {  2, PT_UINT,  "motorType"       },  // PARAM:39  0 brushed, 1 brushless (NEO needs 1)
    {  6, PT_UINT,  "idleMode"        },  // PARAM:42  0 coast, 1 brake
    {  9, PT_UINT,  "feedbackSensor"  },  // PARAM:44  1 = primary encoder
    { 13, PT_FLOAT, "kP"              },  // PARAM:48
    { 14, PT_FLOAT, "kI"              },  // PARAM:49
    { 15, PT_FLOAT, "kD"              },  // PARAM:50
    { 16, PT_FLOAT, "kV"              },  // PARAM:51  kV_0: V/RPM on FW 26 (was kF duty/RPM on FW 25)
    { 17, PT_FLOAT, "kIZone"          },  // PARAM:52
    { 18, PT_FLOAT, "kDFilter"        },  // PARAM:53
    { 19, PT_FLOAT, "outputMin"       },  // PARAM:54
    { 20, PT_FLOAT, "outputMax"       },  // PARAM:55
    { 45, PT_BOOL,  "inverted"        },  // PARAM:80
    { 56, PT_FLOAT, "openLoopRamp"    },  // PARAM:87  raw = 1/seconds-to-full (0 = off)
    { 59, PT_UINT,  "smartStallA"     },  // PARAM:90
    { 60, PT_UINT,  "smartFreeA"      },  // PARAM:91
    { 61, PT_UINT,  "smartLimitRpm"   },  // PARAM:92
    { 74, PT_UINT,  "voltCompMode"    },  // PARAM:99  0 off, 2 on
    { 75, PT_FLOAT, "nominalVoltage"  },  // PARAM:100
    { 96, PT_FLOAT, "kIMaxAccum"      },  // PARAM:101
    { 97, PT_FLOAT, "allowedClErr"    },  // PARAM:102 PID tolerance (FW 26.1.0+); keep 0
    {112, PT_FLOAT, "posConvFactor"   },  // PARAM:109
    {113, PT_FLOAT, "velConvFactor"   },  // PARAM:110
    {114, PT_FLOAT, "closedLoopRamp"  },  // PARAM:111 raw = 1/seconds-to-full (0 = off)
    {136, PT_FLOAT, "hallSamplePeriod"},  // PARAM:127 seconds, 0.008..0.064
    {137, PT_UINT,  "hallAvgDepth"    },  // PARAM:128 index 0..3 = 1/2/4/8 samples
    {158, PT_UINT,  "status0Period"   },  // PARAM:145 ms
    {159, PT_UINT,  "status1Period"   },  // PARAM:146 ms
    {160, PT_UINT,  "status2Period"   },  // PARAM:147 ms
    {165, PT_UINT,  "status7Period"   },  // PARAM:152 ms
    {186, PT_BOOL,  "forceEnStatus0"  },  // PARAM:173
    {187, PT_BOOL,  "forceEnStatus1"  },  // PARAM:174
    {188, PT_BOOL,  "forceEnStatus2"  },  // PARAM:175
    {193, PT_BOOL,  "forceEnStatus7"  },  // PARAM:180
    {199, PT_UINT,  "status8Period"   },  // PARAM:186 ms
    {200, PT_BOOL,  "forceEnStatus8"  },  // PARAM:187
    {204, PT_FLOAT, "kS"              },  // PARAM:191 volts; v2d: must stay 0 (kS comes from KS arbFF)
    {205, PT_FLOAT, "kA"              },  // PARAM:192 V/(RPM/s); MAXMotion only on FW 26
};
static constexpr uint8_t N_PARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);

// ---------- Safety light state (v1) ------------------------------------------
Adafruit_NeoPixel light(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
enum LightMode { LIGHT_SOLID, LIGHT_FLASH };
static LightMode light_mode  = LIGHT_SOLID;   // fail-safe default
static bool      light_on    = true;
static uint32_t  t_flash     = 0;
static uint32_t  t_last_auto = 0;             // millis() of last valid A-line

// ---------- Types and core state ----------------------------------------------
// (All type definitions precede the first function so that the Arduino
// builder's auto-generated prototypes can reference them.)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;

// v2d: parameter values captured from read replies / successful write replies,
// used by CHK and the motor-type interlock. Index = position in CACHE_IDS.
static const uint8_t CACHE_IDS[] = { 2, 6, 9, 16, 74, 75, 112, 113, 136, 137, 204, 205 };
static constexpr uint8_t N_CACHE = sizeof(CACHE_IDS);
static_assert(N_CACHE <= 16, "pc_valid is a 16-bit mask");

struct Wheel {
    uint8_t  dev      = 0;
    char     tag      = '?';
    float    cmd_rpm  = 0.0f;   // raw host-requested setpoint
    float    ramp_rpm = 0.0f;   // slew-limited setpoint actually sent
    float    ks_volts = 0.0f;   // v2c static-friction feedforward (volts), sent as ARBITRARY_FEEDFORWARD
    float    sent_arb = 0.0f;   // arbitrary feedforward in the last velocity setpoint (volts)
    float    cmd_duty = 0.0f;
    float    cmd_volt = 0.0f;
    float    sent_sp  = 0.0f;   // last setpoint value put on the bus (any mode)
    uint32_t sp_tx_us = 0;      // micros() of that setpoint frame
    // STATUS_2
    float    meas_rpm = 0.0f;
    float    meas_pos = 0.0f;
    bool     got_enc  = false;
    uint32_t s2_rx_us = 0;
    uint32_t s2_rx_ms = 0;
    // STATUS_0
    float    applied  = 0.0f;
    float    volts    = 0.0f;
    float    amps     = 0.0f;
    uint8_t  temp_c   = 0;
    uint8_t  s0_flags = 0;      // bits 48..55 of STATUS_0 (limits, inverted, hb lock, model lsbs)
    uint8_t  model    = 0;
    bool     got_s0   = false;
    uint32_t s0_rx_ms = 0;
    // STATUS_1
    uint64_t s1_word  = 0;
    bool     got_s1   = false;
    // SET_STATUSES_ENABLED response
    int16_t  sse_res  = -1;
    uint16_t sse_en   = 0;
    // PERSIST response
    int16_t  burn_res = -1;
    // v2d STATUS_7 / STATUS_8 (NAN / -1 until the first frame arrives)
    float    iaccum   = NAN;    // STATUS_7 I_ACCUMULATION
    float    s8_sp    = NAN;    // STATUS_8 SETPOINT (RPM in velocity mode)
    int8_t   s8_atsp  = -1;     // STATUS_8 IS_AT_SETPOINT
    int8_t   s8_slot  = -1;     // STATUS_8 SELECTED_PID_SLOT
    // v2d parameter cache + firmware version (CHK)
    uint32_t pc_raw[N_CACHE] = {0};
    uint16_t pc_valid = 0;
    bool     fw_valid = false;
    uint8_t  fw_maj = 0, fw_min = 0;
    uint16_t fw_build = 0;
    // v2d motor-type interlock
    bool     mt_known   = false;   // param 2 read back at least once since boot
    uint32_t motor_type = 0;
    bool     mt_blocked = false;
    bool     mt_warned  = false;   // "!!" line printed for mt_warned_val
    uint32_t mt_warned_val = 0;
    uint32_t blk_frames = 0;       // setpoints replaced by duty 0 (DIAG)
};
static Wheel left, right;

enum ControlMode { MODE_VELOCITY, MODE_DUTY, MODE_VOLTAGE };
static ControlMode ctrl_mode = MODE_VELOCITY;

// ---------- Request/response engine (writes, reads, firmware version) --------
// One queue per SPARK; one request in flight per SPARK. Responses are matched
// in onCanRx(); timeouts and retries are handled in serviceJobs() from loop().
enum JobKind : uint8_t { JOB_WRITE, JOB_READ, JOB_FW };
static constexpr uint8_t JF_QUIET = 0x01;   // v2d: no PRD/FV/TIMEOUT line (CHK, periodic param-2 read)
static constexpr uint8_t JF_CHK   = 0x02;   // v2d: counted by the running CHK
struct Job {
    uint8_t  kind;
    uint8_t  flags;    // JF_*
    uint8_t  id;       // parameter id (WRITE/READ)
    uint8_t  type;     // PType of the value (WRITE) / expected type (READ)
    uint8_t  tries;
    uint8_t  rtr_dlc;  // FW: DLC of the RTR request
    uint32_t raw;      // WRITE value, encoded per type
};
static constexpr uint8_t  JOBQ_LEN        = 16;
static constexpr uint32_t WRITE_TIMEOUT_US = 60000;   // REVLib default CAN timeout is 20 ms (issue69:183); x3 margin
static constexpr uint32_t READ_TIMEOUT_US  = 60000;
static constexpr uint8_t  WRITE_TRIES      = 3;       // REVLib retries 5x by default (issue69:179)
static constexpr uint8_t  READ_TRIES       = 2;

struct JobQueue {
    Job      q[JOBQ_LEN];
    uint8_t  head = 0, count = 0;
    bool     busy = false;      // q[head] is in flight
    uint32_t t_sent_us = 0;
};
static JobQueue jq_left, jq_right;

enum BurnState { BURN_IDLE, BURN_DISABLING, BURN_PERSISTING };

static const ParamInfo *findParamById(uint8_t id) {
    for (uint8_t i = 0; i < N_PARAMS; i++) if (PARAMS[i].id == id) return &PARAMS[i];
    return nullptr;
}
static const ParamInfo *findParamByName(const char *n) {
    for (uint8_t i = 0; i < N_PARAMS; i++) if (strcasecmp(PARAMS[i].name, n) == 0) return &PARAMS[i];
    return nullptr;
}

// K-command letter -> parameter id (types come from PARAMS).
static int kLetterToParam(char c) {
    switch (c) {
        case 'P': return 13;
        case 'I': return 14;
        case 'D': return 15;
        case 'F': return 16;   // v1 letter kept ("OK KF="): param 16 = kV, V/RPM on FW 26 (was kF duty/RPM on FW 25)
        case 'V': return 16;
        case 'Z': return 17;
        // 'S' is the Teensy-side arbFF kS (cmdK); v2d: native param 204 is not reachable
        // from K, so kS has one source only (write 204 with PW, and keep it 0).
        case 'A': return 205;  // MAXMotion only on FW 26 (FW26_CHANGES §1); cmdK prints a warning
        default:  return -1;
    }
}


// Slew-rate limit on the velocity setpoint, RPM per 20 ms tick (M<val>).
static float max_rpm_step = 100.0f;
// S and the host watchdog stop to idle (duty 0 -> SPARK idle mode, Brake) instead of a
// velocity-0 setpoint. 0 restores the v1/v2 behaviour (velocity mode at 0 RPM).
static constexpr bool STOP_TO_IDLE = true;
static float duty_cap     = DUTY_CAP_DEFAULT;

static uint32_t t_ctrl = 0, t_fb = 0, t_enc_cfg = 0, t_last_host = 0, t_mt_check = 0;
static uint32_t tx_count = 0, tx_queued = 0, tx_fail = 0, rx_count = 0, rx_foreign = 0, ser_drop = 0;
static bool     wdt_tripped = false;
static float    bus_voltage = 0.0f;   // last STATUS_0 voltage from either SPARK (v1 DIAG field)
static bool     hb_enabled  = true;   // universal heartbeat Enabled bit
static uint32_t hb_test_until = 0;    // HB0 bench test: millis() when the enabled heartbeat returns (0 = off)
static constexpr uint32_t HB_TEST_MAX_MS = 5000;
static bool     x_enabled   = false;  // X telemetry
static bool     x_pending   = false;  // a STATUS_2 arrived since the last X line
static uint32_t max_rx_age_us = 0;    // largest CAN-timestamp age correction seen (DIAG)

// ---------- Serial output (never blocks the loop) ----------------------------
// Teensy's USB CDC write can wait up to 120 ms when its buffers are full.
// Every line goes through out(), which drops the line (counted in DIAG
// sdrop=) rather than stall the 20 ms heartbeat.
static void out(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void out(const char *fmt, ...) {
    char buf[768];   // v2d review: DIAG with the v2d fields reaches ~495 chars typical, ~590 worst case
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n >= (int)sizeof(buf)) { n = sizeof(buf) - 1; buf[n - 1] = '\n'; }  // keep line framing
    if (Serial.availableForWrite() >= n) Serial.write((const uint8_t *)buf, n);
    else ser_drop++;
}

// ---------- CAN helpers ------------------------------------------------------
static inline uint32_t sparkId(uint8_t cls, uint8_t idx, uint8_t dev) {
    return ((uint32_t)SPARK_DEV_TYPE << 24)
         | ((uint32_t)SPARK_MFG      << 16)
         | ((uint32_t)(cls & 0x3F)   << 10)
         | ((uint32_t)(idx & 0x0F)   <<  6)
         | ((uint32_t)(dev & 0x3F));
}

static bool canSendRaw(uint32_t id, const uint8_t *data, uint8_t len, bool rtr) {
    CAN_message_t m;
    m.flags.extended = 1;
    m.flags.remote   = rtr ? 1 : 0;
    m.id  = id;
    m.len = len;
    if (data && len) memcpy(m.buf, data, len);
    // write() returns 1 = in a mailbox, -1 = queued in software, 0 = failed.
    int r = can.write(m);
    if (r > 0) { tx_count++; return true; }
    if (r < 0) { tx_count++; tx_queued++; return true; }   // no free mailbox: library software queue
    tx_fail++;
    return false;
}
static inline bool canSend(uint32_t id, const uint8_t *data, uint8_t len) {
    return canSendRaw(id, data, len, false);
}

static inline void putU32(uint8_t *b, uint32_t v) { b[0] = v; b[1] = v >> 8; b[2] = v >> 16; b[3] = v >> 24; }
static inline uint32_t getU32(const uint8_t *b) {
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}
static inline uint64_t getU64(const uint8_t *b) {
    return (uint64_t)getU32(b) | ((uint64_t)getU32(b + 4) << 32);
}
static inline uint32_t f2u(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }
static inline float    u2f(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }

static inline Wheel *wheelForDev(uint8_t dev) {
    if (dev == LEFT_ID)  return &left;
    if (dev == RIGHT_ID) return &right;
    return nullptr;
}

// Format a raw 32-bit parameter value according to its type.
static void fmtValue(char *dst, size_t n, uint8_t type, uint32_t raw) {
    switch (type) {
        case PT_INT:   snprintf(dst, n, "%ld", (long)(int32_t)raw); break;
        case PT_UINT:  snprintf(dst, n, "%lu", (unsigned long)raw); break;
        case PT_FLOAT: snprintf(dst, n, "%.7g", (double)u2f(raw)); break;   // raw hex printed alongside where exactness matters
        case PT_BOOL:  snprintf(dst, n, "%lu", (unsigned long)(raw ? 1 : 0)); break;
        default:       snprintf(dst, n, "0x%08lx", (unsigned long)raw); break;
    }
}
static char typeChar(uint8_t t) {
    switch (t) { case PT_INT: return 'i'; case PT_UINT: return 'u'; case PT_FLOAT: return 'f'; case PT_BOOL: return 'b'; default: return '?'; }
}

// ---------- SPARK commands ---------------------------------------------------
static void sendHeartbeats() {
    // Universal heartbeat (RST:188-249). Enabled: byte-identical to v1 (enables under
    // either byte order, see the constant's comment). Disabled (BURN only): ALL ZERO, so
    // Enabled and SystemWatchdog are clear under either byte order. (The earlier v2 draft
    // cleared only data[3], which leaves data[4] bit 4 = SystemWatchdog set: still enabled.)
    static const uint8_t uni_en[8]  = {0x78, 0x01, 0x00, 0x12, 0x59, 0x04, 0x00, 0x60};
    static const uint8_t uni_dis[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canSend(UNIVERSAL_HB, hb_enabled ? uni_en : uni_dis, 8);
    // Secondary heartbeat (JSON:1525): a 64-bit per-device enable bitfield, ignored once
    // the SPARK is locked to the universal heartbeat (STATUS_0 bit 53). Enabled: all ones
    // as in v1. Disabled: all ZEROS (REV's own node-can-bridge disables this way rather
    // than going silent), so an unlocked SPARK is explicitly disabled too.
    static const uint8_t sec_en[8]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    static const uint8_t sec_dis[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canSend(sparkId(CLS_HB, IDX_HB, 0), hb_enabled ? sec_en : sec_dis, 8);
}

// v2c: VELOCITY_SETPOINT (JSON:716, versionImplemented 25.0.0) carries ARBITRARY_FEEDFORWARD,
// int16 at bits 32-47, scale 0.0009765923 (JSON signal), and ARBITRARY_FEEDFORWARD_UNITS at bit 50
// (0 = voltage, 1 = duty). REVLib exposes it as setReference(..., arbFeedforward, units). Used here
// for per-side static-friction feedforward kS because FW 25 has no kS parameter (write -> invalid_id).
static constexpr float ARB_FF_SCALE     = 0.0009765923f;   // JSON: decodeScaleFactor
static constexpr float KS_MAX_VOLTS     = 2.0f;            // safety clamp on the static-friction term
static constexpr float KS_DEADBAND_RPM  = 1.0f;            // |setpoint| below this -> no kS (zero stays zero)

// v2d motor-type interlock (EXPECT_BRUSHLESS): true once param 2 read back != 1.
// Review recommendation: block BOTH tracks if either SPARK is blocked, so the good track can
// never drive alone and pivot the chassis. (w is kept for the per-side frame counter.)
static inline bool motionBlocked(const Wheel &w) {
    (void)w;
    return EXPECT_BRUSHLESS && (left.mt_blocked || right.mt_blocked);
}

static void sendSetpoint(Wheel &w, uint8_t idx, float value, float arb_volts = 0.0f) {
    if (motionBlocked(w)) {
        // Wrong motor type: every setpoint to this SPARK becomes duty 0 (idle), whatever
        // the mode. The velocity ramp is held at 0 so an unblock does not step.
        idx = IDX_DUTY; value = 0.0f; arb_volts = 0.0f;
        w.ramp_rpm = 0.0f;
        w.blk_frames++;
    }
    uint8_t d[SETPOINT_DLC] = {0};          // slot 0 (bits 48-49), FF units 0 = volts (bit 50)
    putU32(d, f2u(value));
    long q = lroundf(arb_volts / ARB_FF_SCALE);
    if (q >  32767) q =  32767;
    if (q < -32768) q = -32768;
    int16_t a16 = (int16_t)q;
    d[4] = (uint8_t)(a16 & 0xFF);           // little-endian int16 at bits 32-47
    d[5] = (uint8_t)((uint16_t)a16 >> 8);
    w.sent_arb = a16 * ARB_FF_SCALE;
    canSend(sparkId(CLS_SETPOINT, idx, w.dev), d, SETPOINT_DLC);
    w.sent_sp  = value;
    w.sp_tx_us = micros();
}

static void setVelocity(Wheel &w, float rpm) {
    if (isnan(rpm)) rpm = 0.0f;               // REVIEW fix: NaN passes every compare
    if (rpm >  MAX_RPM) rpm =  MAX_RPM;
    if (rpm < -MAX_RPM) rpm = -MAX_RPM;
    // kS uses the sign of the (ramped) TARGET, not the measured speed, so it cannot chatter at
    // standstill; a zero target sends zero feedforward (C1 practice 6-7: friction FF from the reference).
    float arb = 0.0f;
    if (rpm >  KS_DEADBAND_RPM) arb =  w.ks_volts;
    if (rpm < -KS_DEADBAND_RPM) arb = -w.ks_volts;
    sendSetpoint(w, IDX_VELOCITY, rpm, arb);
}
static inline float clampDuty(float d) {
    if (isnan(d)) return 0.0f;                // REVIEW fix: NaN passes every compare
    if (d >  duty_cap) return  duty_cap;
    if (d < -duty_cap) return -duty_cap;
    return d;
}
static inline float voltCap() { return duty_cap * NOMINAL_V; }
static inline float clampVolt(float v) {
    float c = voltCap();
    if (isnan(v)) return 0.0f;                // REVIEW fix: NaN passes every compare
    if (v >  c) return  c;
    if (v < -c) return -c;
    return v;
}
static void setDuty(Wheel &w, float duty)   { sendSetpoint(w, IDX_DUTY,    clampDuty(duty)); }
static void setVoltage(Wheel &w, float v)   { sendSetpoint(w, IDX_VOLTAGE, clampVolt(v)); }

static void sendStatusesEnabled(uint8_t dev) {
    uint8_t d[SSE_DLC];
    d[0] = SSE_BITS & 0xFF; d[1] = SSE_BITS >> 8;   // MASK
    d[2] = SSE_BITS & 0xFF; d[3] = SSE_BITS >> 8;   // ENABLED_BITFIELD
    canSend(sparkId(CLS_CFG, IDX_SSE, dev), d, SSE_DLC);
}

static void sendPersist(uint8_t dev) {
    uint8_t d[2] = { (uint8_t)(PERSIST_MAGIC & 0xFF), (uint8_t)(PERSIST_MAGIC >> 8) };  // A3 3A
    canSend(sparkId(CLS_PERSIST, IDX_PERSIST, dev), d, 2);
}

static void sendClearFaults(uint8_t dev) { canSend(sparkId(CLS_CLEAR_FAULTS, IDX_CLEAR_FAULTS, dev), nullptr, 0); }
static void sendIdentify(uint8_t dev)    { canSend(sparkId(CLS_IDENTIFY, IDX_IDENTIFY, dev), nullptr, 0); }

// Move `current` toward `target` by at most `step` (per control tick).
static inline void slewToward(float &current, float target, float step) {
    float delta = target - current;
    if (delta >  step) delta =  step;
    if (delta < -step) delta = -step;
    current += delta;
}


static inline JobQueue &jqFor(const Wheel &w) { return (&w == &left) ? jq_left : jq_right; }

static uint8_t chk_outstanding = 0;     // v2d: JF_CHK jobs not yet answered / timed out

static bool jobPush(Wheel &w, const Job &j) {
    JobQueue &jq = jqFor(w);
    if (jq.count >= JOBQ_LEN) return false;
    jq.q[(jq.head + jq.count) % JOBQ_LEN] = j;
    jq.count++;
    if (j.flags & JF_CHK) chk_outstanding++;
    return true;
}
static void jobPop(JobQueue &jq) {
    if ((jq.q[jq.head].flags & JF_CHK) && chk_outstanding) chk_outstanding--;
    jq.head = (jq.head + 1) % JOBQ_LEN;
    jq.count--;
    jq.busy = false;
}
static inline uint32_t readPairId(uint8_t id, uint8_t dev) {
    uint8_t pair = id >> 1;
    return sparkId(CLS_READ_BASE + (pair >> 4), pair & 0x0F, dev);
}

static void jobTransmit(Wheel &w, JobQueue &jq) {
    Job &j = jq.q[jq.head];
    switch (j.kind) {
        case JOB_WRITE: {
            uint8_t d[PARAM_WRITE_DLC];
            d[0] = j.id;
            putU32(d + 1, j.raw);
            canSend(sparkId(CLS_PARAM, IDX_PARAM_WRITE, w.dev), d, PARAM_WRITE_DLC);
            break;
        }
        case JOB_READ:
            canSendRaw(readPairId(j.id, w.dev), nullptr, READ_RTR_DLC, true);
            break;
        case JOB_FW:
            canSendRaw(sparkId(CLS_FW, IDX_FW, w.dev), nullptr, j.rtr_dlc, true);
            break;
    }
    j.tries++;
    jq.busy = true;
    jq.t_sent_us = micros();
}

static bool burnActive();

static void serviceJobsFor(Wheel &w) {
    JobQueue &jq = jqFor(w);
    if (jq.busy) {
        Job &j = jq.q[jq.head];
        uint32_t tmo = (j.kind == JOB_WRITE) ? WRITE_TIMEOUT_US : READ_TIMEOUT_US;
        if ((uint32_t)(micros() - jq.t_sent_us) < tmo) return;
        uint8_t max_tries = (j.kind == JOB_WRITE) ? WRITE_TRIES : READ_TRIES;
        if (j.kind == JOB_FW && j.rtr_dlc == 8 && j.tries >= READ_TRIES) {
            // Spec form (rtr, DLC 8) got no answer: try the DLC-0 remote frame
            // that community measurements report the firmware answering.
            j.rtr_dlc = 0; j.tries = 0;
            jobTransmit(w, jq);
            return;
        }
        if (j.tries < max_tries) { jobTransmit(w, jq); return; }
        if (!(j.flags & JF_QUIET)) switch (j.kind) {
            case JOB_WRITE: out("PWR %c id=%u TIMEOUT\n", w.tag, j.id); break;
            case JOB_READ:  out("PRD %c id=%u TIMEOUT\n", w.tag, j.id); break;
            case JOB_FW:    out("FV %c TIMEOUT\n", w.tag); break;
        }
        jobPop(jq);
    }
    if (!jq.busy && jq.count > 0 && !burnActive()) jobTransmit(w, jq);
}

static void serviceJobs() { serviceJobsFor(left); serviceJobsFor(right); }
static inline bool jobsIdle() {
    return jq_left.count == 0 && jq_right.count == 0;
}

static bool queueWrite(Wheel &w, uint8_t id, uint8_t type, uint32_t raw) {
    Job j = {};
    j.kind = JOB_WRITE; j.id = id; j.type = type; j.raw = raw;
    if (!jobPush(w, j)) { out("ERR PW %c queue full\n", w.tag); return false; }
    return true;
}
static bool queueRead(Wheel &w, uint8_t id, uint8_t flags = 0) {
    const ParamInfo *pi = findParamById(id);
    Job j = {};
    j.kind = JOB_READ; j.id = id; j.type = pi ? pi->type : PT_UNUSED; j.flags = flags;
    if (!jobPush(w, j)) { if (!(flags & JF_QUIET)) out("ERR PR %c queue full\n", w.tag); return false; }
    return true;
}
static bool queueFw(Wheel &w, uint8_t flags = 0) {
    Job j = {};
    j.kind = JOB_FW; j.rtr_dlc = 8; j.flags = flags;   // HDR:400 SPARK_GET_FIRMWARE_VERSION_LENGTH (8u), JSON rtr true
    if (!jobPush(w, j)) { if (!(flags & JF_QUIET)) out("ERR FV %c queue full\n", w.tag); return false; }
    return true;
}
// v2d: is a read of `id`'s pair already queued or in flight on this SPARK?
static bool readQueued(const Wheel &w, uint8_t id) {
    const JobQueue &jq = jqFor(w);
    for (uint8_t i = 0; i < jq.count; i++) {
        const Job &j = jq.q[(jq.head + i) % JOBQ_LEN];
        if (j.kind == JOB_READ && (j.id >> 1) == (id >> 1)) return true;
    }
    return false;
}
// v2d: periodic / post-PW motor-type re-check (quiet; the interlock prints on change).
static void queueMotorTypeRead(Wheel &w, uint8_t flags) {
    if (!readQueued(w, 2)) queueRead(w, 2, flags);
}

// ---------- BURN state machine ----------------------------------------------
static BurnState burn_state = BURN_IDLE;
static uint32_t  burn_t0 = 0;               // millis() when the current phase started
static constexpr float    BURN_MAX_RPM        = 30.0f;  // refuse above this measured speed
static constexpr uint32_t BURN_ENC_FRESH_MS   = 200;    // STATUS_2 must be this fresh
static constexpr uint32_t BURN_MIN_DWELL_MS   = 120;    // >100 ms SPARK heartbeat timeout
static constexpr uint32_t BURN_MAX_DWELL_MS   = 300;
static constexpr float    BURN_APPLIED_EPS    = 0.01f;  // |applied output| considered 0
static constexpr uint32_t BURN_RESP_TIMEOUT_MS = 1500;  // spec: "may take up to a second"

static bool burnActive() { return burn_state != BURN_IDLE; }

// REVIEW S3: velocity mode at 0 RPM with the ramp reset, same as the S command.
static void zeroAllCommands() {
    left.cmd_rpm  = right.cmd_rpm  = 0.0f;
    left.ramp_rpm = right.ramp_rpm = 0.0f;
    left.cmd_duty = right.cmd_duty = 0.0f;
    left.cmd_volt = right.cmd_volt = 0.0f;
    ctrl_mode = MODE_VELOCITY;
}

static void burnStart(uint32_t now) {
    if (burnActive()) { out("ERR BURN busy\n"); return; }
    if (!jobsIdle())  { out("ERR BURN refused: parameter requests pending\n"); return; }
    bool fresh = left.got_enc && right.got_enc
              && (now - left.s2_rx_ms)  < BURN_ENC_FRESH_MS
              && (now - right.s2_rx_ms) < BURN_ENC_FRESH_MS;
    if (!fresh) { out("ERR BURN refused: no fresh STATUS_2 (cannot verify wheels stopped)\n"); return; }
    if (fabsf(left.meas_rpm) > BURN_MAX_RPM || fabsf(right.meas_rpm) > BURN_MAX_RPM) {
        out("ERR BURN refused: moving L=%.0f R=%.0f rpm (limit %.0f)\n",
            (double)left.meas_rpm, (double)right.meas_rpm, (double)BURN_MAX_RPM);
        return;
    }
    left.burn_res = right.burn_res = -1;
    zeroAllCommands();                   // REVIEW S3: nothing resumes when the heartbeat returns
    hb_enabled = false;                  // all-zero universal + secondary HB
    hb_test_until = 0;                   // FINAL CHECK: BURN supersedes a running HB0 test
    burn_state = BURN_DISABLING;
    burn_t0 = now;
    out("# BURN disabling (hb_lock L=%d R=%d); persists ALL current RAM parameters\n",
        (left.s0_flags >> 5) & 1, (right.s0_flags >> 5) & 1);
}

static void serviceBurn(uint32_t now) {
    if (burn_state == BURN_DISABLING) {
        uint32_t dt = now - burn_t0;
        bool l0 = left.got_s0  && (int32_t)(left.s0_rx_ms  - burn_t0) > 0 && fabsf(left.applied)  < BURN_APPLIED_EPS;
        bool r0 = right.got_s0 && (int32_t)(right.s0_rx_ms - burn_t0) > 0 && fabsf(right.applied) < BURN_APPLIED_EPS;
        if ((dt >= BURN_MIN_DWELL_MS && l0 && r0) || dt >= BURN_MAX_DWELL_MS) {
            if (!(l0 && r0)) {
                // REVIEW S2: the SPARK is not confirmed disabled; REV says persist only
                // while disabled, so abort instead of persisting.
                zeroAllCommands();
                hb_enabled = true;
                burn_state = BURN_IDLE;
                out("ERR BURN aborted: applied output not ~0 after %lu ms (L=%.3f R=%.3f); nothing persisted\n",
                    (unsigned long)dt, (double)left.applied, (double)right.applied);
                return;
            }
            sendPersist(LEFT_ID);
            sendPersist(RIGHT_ID);
            burn_state = BURN_PERSISTING;
            burn_t0 = now;
        }
    } else if (burn_state == BURN_PERSISTING) {
        bool done = (left.burn_res >= 0 && right.burn_res >= 0);
        if (done || (now - burn_t0) >= BURN_RESP_TIMEOUT_MS) {
            zeroAllCommands();           // REVIEW S3: drop any command sent during BURN
            hb_enabled = true;
            burn_state = BURN_IDLE;
            out("OK BURN result L=%d R=%d\n", left.burn_res, right.burn_res);
        }
    }
}

// ---------- v2d: parameter cache, motor-type interlock, CHK --------------------
static int cacheIndex(uint8_t id) {
    for (uint8_t i = 0; i < N_CACHE; i++) if (CACHE_IDS[i] == id) return i;
    return -1;
}
static bool cacheGet(const Wheel &w, uint8_t id, uint32_t &raw) {
    int i = cacheIndex(id);
    if (i < 0 || !(w.pc_valid & (1u << i))) return false;
    raw = w.pc_raw[i];
    return true;
}

// Called with every read-back of param 2 (read reply, or successful write reply).
static void motorTypeUpdate(Wheel &w, uint32_t mt) {
    w.mt_known   = true;
    w.motor_type = mt;
    bool block = EXPECT_BRUSHLESS && mt != 1;
    if (block && !(w.mt_warned && w.mt_warned_val == mt)) {
        out("!! MOTOR TYPE %lu on %c: motion blocked (NEO needs brushless=1)\n", (unsigned long)mt, w.tag);
        w.mt_warned = true;
        w.mt_warned_val = mt;
    }
    if (!block) {
        if (w.mt_blocked) out("# MOTOR TYPE 1 on %c: motion unblocked\n", w.tag);
        w.mt_warned = false;
    }
    w.mt_blocked = block;
}

static void cacheStore(Wheel &w, uint8_t id, uint32_t raw) {
    int i = cacheIndex(id);
    if (i >= 0) { w.pc_raw[i] = raw; w.pc_valid |= (uint16_t)(1u << i); }
    if (id == 2) motorTypeUpdate(w, raw);
}

// CHK: reads FV + 8 parameter pairs per SPARK (the pairs cover 2/3, 6/7, 8/9, 16/17,
// 74/75, 112/113, 136/137, 204/205), all quiet, then chkReport() prints the summary
// from loop() once every JF_CHK job has been answered or has timed out.
static const uint8_t CHK_READS[] = { 2, 6, 9, 16, 74, 112, 136, 204 };
static bool chk_active = false;
static bool chk_queue_fail = false;
static bool chk_boot_pending = true;
static uint32_t t_boot = 0;

static void chkStart(bool automatic) {
    if (chk_active) { out("ERR CHK busy\n"); return; }
    chk_active = true;
    chk_queue_fail = false;
    Wheel *ws[2] = { &left, &right };
    for (uint8_t k = 0; k < 2; k++) {
        Wheel &w = *ws[k];
        w.pc_valid = 0;
        w.fw_valid = false;
        if (!queueFw(w, JF_QUIET | JF_CHK)) chk_queue_fail = true;
        for (uint8_t i = 0; i < sizeof(CHK_READS); i++)
            if (!queueRead(w, CHK_READS[i], JF_QUIET | JF_CHK)) chk_queue_fail = true;
    }
    out("OK CHK%s requests=%u\n", automatic ? " (boot)" : "", chk_outstanding);
}

// Append " <text>" to the reason list (bounded).
static void addReason(char *dst, size_t n, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
static void addReason(char *dst, size_t n, const char *fmt, ...) {
    size_t l = strlen(dst);
    if (l + 2 >= n) return;
    dst[l++] = ' ';
    dst[l] = '\0';
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(dst + l, n - l, fmt, ap);
    va_end(ap);
}

static void chkWheel(const Wheel &w, char *why, size_t nwhy) {
    const char t = w.tag;
    uint32_t a, b;
    // 1. firmware version
    if (w.fw_valid) {
        bool exp = (w.fw_maj == 26 && w.fw_min == 1 && w.fw_build == 5);
        out("CHK %c fw %u.%u.%u%s\n", t, w.fw_maj, w.fw_min, w.fw_build, exp ? " ok" : " (expected 26.1.5)");
        if (w.fw_maj < 26) addReason(why, nwhy, "%c:fw<26", t);
    } else { out("CHK %c fw NOREAD\n", t); addReason(why, nwhy, "%c:fw?", t); }
    // 2. motor type (param 2) -- must be 1 = brushless for a NEO
    if (cacheGet(w, 2, a)) {
        out("CHK %c motorType(2) %lu %s\n", t, (unsigned long)a,
            a == 1 ? "brushless ok" : (a == 0 ? "BRUSHED: BAD, NEO needs brushless=1" : "BAD, NEO needs brushless=1"));
        if (a != 1) addReason(why, nwhy, "%c:motorType=%lu", t, (unsigned long)a);
    } else { out("CHK %c motorType(2) NOREAD\n", t); addReason(why, nwhy, "%c:noread(2)", t); }
    // 3. feedback sensor (param 9): printed only (FeedbackSensor.java: 0 none, 1 primary encoder)
    if (cacheGet(w, 9, a)) out("CHK %c feedbackSensor(9) %lu%s\n", t, (unsigned long)a, a == 1 ? " (primary encoder)" : " (note: 1 = primary encoder)");
    else { out("CHK %c feedbackSensor(9) NOREAD\n", t); addReason(why, nwhy, "%c:noread(9)", t); }
    // 4. idle mode (param 6); L/R comparison is printed by chkReport()
    if (cacheGet(w, 6, a)) out("CHK %c idleMode(6) %lu %s\n", t, (unsigned long)a, a == 1 ? "brake" : (a == 0 ? "coast" : "?"));
    else { out("CHK %c idleMode(6) NOREAD\n", t); addReason(why, nwhy, "%c:noread(6)", t); }
    // 5. kV (param 16): V/RPM on FW 26 (was kF duty/RPM on FW 25)
    if (cacheGet(w, 16, a)) {
        float kv = u2f(a);
        bool fw26 = w.fw_valid && w.fw_maj >= 26;
        bool small = fw26 && kv < 5e-4f;
        out("CHK %c kV(16) %.7g V/RPM raw=0x%08lx%s\n", t, (double)kv, (unsigned long)a,
            small ? " WARN kV looks like a duty/RPM value; FW 26 uses V/RPM"
                  : (fw26 ? " ok" : " (fw unknown or <26: unit check skipped)"));
        if (small) addReason(why, nwhy, "%c:kV<5e-4", t);
    } else { out("CHK %c kV(16) NOREAD\n", t); addReason(why, nwhy, "%c:noread(16)", t); }
    // 6. voltage compensation (74 mode, 75 nominal V): printed only
    if (cacheGet(w, 74, a) && cacheGet(w, 75, b))
        out("CHK %c voltComp(74/75) mode=%lu%s nominal=%.4g V\n", t, (unsigned long)a,
            a == 0 ? " (off)" : (a == 2 ? " (on)" : ""), (double)u2f(b));
    else { out("CHK %c voltComp(74/75) NOREAD\n", t); addReason(why, nwhy, "%c:noread(74)", t); }
    // 7. kS (204) must be 0: kS comes from the Teensy arbFF only (KS command)
    if (cacheGet(w, 204, a)) {
        float ks = u2f(a);
        out("CHK %c kS(204) %.7g V%s (Teensy arbFF KS=%.4f V)\n", t, (double)ks,
            ks == 0.0f ? " ok" : " WARN param 204 != 0: native kS adds to the KS arbFF and applies +kS at setpoint 0; set 204 = 0",
            (double)w.ks_volts);
        if (ks != 0.0f) addReason(why, nwhy, "%c:kS204!=0", t);
    } else { out("CHK %c kS(204) NOREAD\n", t); addReason(why, nwhy, "%c:noread(204)", t); }
    // 8. kA (205): printed only, ignored in plain velocity mode on FW 26
    if (cacheGet(w, 205, a))
        out("CHK %c kA(205) %.7g V/(RPM/s)%s\n", t, (double)u2f(a),
            u2f(a) == 0.0f ? "" : " (note: param 205 only affects MAXMotion on FW 26)");
    else out("CHK %c kA(205) NOREAD\n", t);   // same pair as 204: already counted
    // 9. hall filter: 136 period (float seconds, default 0x3D000000 = 0.03125) + 137 depth
    if (cacheGet(w, 136, a) && cacheGet(w, 137, b))
        out("CHK %c hall(136/137) period=%.7g s raw=0x%08lx depth=%lu (%u samples)\n", t,
            (double)u2f(a), (unsigned long)a, (unsigned long)b, b <= 3 ? (1u << b) : 0u);
    else { out("CHK %c hall(136/137) NOREAD\n", t); addReason(why, nwhy, "%c:noread(136)", t); }
    // 10. conversion factors 112/113 must be exactly 1.0 (MAX_RPM, E line, host odometry assume RPM/rot)
    if (cacheGet(w, 112, a) && cacheGet(w, 113, b)) {
        bool ok = (u2f(a) == 1.0f && u2f(b) == 1.0f);
        out("CHK %c conv(112/113) pos=%.7g vel=%.7g %s\n", t, (double)u2f(a), (double)u2f(b), ok ? "ok" : "BAD: must be 1.0");
        if (!ok) addReason(why, nwhy, "%c:conv!=1", t);
    } else { out("CHK %c conv(112/113) NOREAD\n", t); addReason(why, nwhy, "%c:noread(112)", t); }
}

static void chkReport() {
    chk_active = false;
    char why[240] = "";
    chkWheel(left,  why, sizeof(why));
    chkWheel(right, why, sizeof(why));
    uint32_t il, ir;
    if (cacheGet(left, 6, il) && cacheGet(right, 6, ir)) {
        out("CHK B idleMode L=%lu R=%lu %s\n", (unsigned long)il, (unsigned long)ir,
            il == ir ? "ok" : "WARN L/R differ (write both, BURN, power-cycle; FW26_CHANGES B12)");
        if (il != ir) addReason(why, sizeof(why), "idleMode_L!=R");
    }
    if (chk_queue_fail) addReason(why, sizeof(why), "queue_full");
    if (why[0]) out("CHK FAIL%s\n", why);
    else        out("CHK OK\n");
}

// ---------- CAN RX ---------------------------------------------------------
static const char *const FAULT_NAMES[8] = {
    "OTHER", "MOTOR_TYPE", "SENSOR", "CAN", "TEMPERATURE", "DRV", "ESC_EEPROM", "FIRMWARE"
};  // STATUS_1 bits 0-7 faults (JSON:215..), 24-31 sticky faults (JSON:253..)
static const char *const WARN_NAMES[8] = {
    "BROWNOUT", "OVERCURRENT", "ESC_EEPROM", "EXT_EEPROM", "SENSOR", "STALL", "HAS_RESET", "OTHER"
};  // STATUS_1 bits 16-23 warnings (JSON:235..), 40-47 sticky warnings (JSON:323..); IS_FOLLOWER bit 48 (JSON:411)
static constexpr uint64_t S1_MEANINGFUL = 0x0001FF00FFFF00FFull; // bits 0-7, 16-31, 40-48

static void appendNames(char *dst, size_t n, const char *prefix, uint8_t bits, const char *const names[8]) {
    for (uint8_t b = 0; b < 8; b++) {
        if (!(bits & (1u << b))) continue;
        size_t l = strlen(dst);
        if (l + 2 >= n) return;
        snprintf(dst + l, n - l, " %s%s", prefix, names[b]);
    }
}

static void printFaults(const Wheel &w) {
    uint8_t f  = (uint8_t)(w.s1_word);
    uint8_t wr = (uint8_t)(w.s1_word >> 16);
    uint8_t sf = (uint8_t)(w.s1_word >> 24);
    uint8_t sw = (uint8_t)(w.s1_word >> 40);
    uint8_t fol = (uint8_t)((w.s1_word >> 48) & 1);
    char names[200] = "";
    appendNames(names, sizeof(names), "F:",  f,  FAULT_NAMES);
    appendNames(names, sizeof(names), "W:",  wr, WARN_NAMES);
    appendNames(names, sizeof(names), "SF:", sf, FAULT_NAMES);
    appendNames(names, sizeof(names), "SW:", sw, WARN_NAMES);
    out("F %c f=0x%02x w=0x%02x sf=0x%02x sw=0x%02x follower=%u%s\n",
        w.tag, f, wr, sf, sw, fol, names[0] ? names : " none");
}

static const char *writeResultName(uint8_t r) {
    switch (r) {
        case 0: return "ok";
        case 1: return "invalid_id";
        case 2: return "mismatched_type";
        case 3: return "access_mode";
        case 4: return "invalid";
        case 5: return "not_implemented";
        default: return "unknown";
    }
}

// Receive time in Teensy micros(), corrected for the delay between frame
// reception and this callback (can.events() dispatches from loop()). The
// FlexCAN free-running TIMER ticks once per CAN bit time (1 us at 1 Mbit/s)
// and is latched into the frame's timestamp at reception. If the correction
// looks implausible the dispatch time is used unchanged.
static uint32_t rxMicros(const CAN_message_t &msg) {
    uint32_t now_us = micros();
    uint16_t age_ticks = (uint16_t)((uint16_t)FLEXCAN1_TIMER - msg.timestamp);
    uint32_t age_us = (uint32_t)age_ticks * (1000000UL / CAN_BAUD);
    if (age_us > 20000) return now_us;
    if (age_us > max_rx_age_us) max_rx_age_us = age_us;
    return now_us - age_us;
}

static void onCanRx(const CAN_message_t &msg) {
    if (!msg.flags.extended || msg.flags.remote) return;   // our own RTR requests / non-SPARK
    if ((msg.id & SPARK_ID_MASK) != SPARK_ID_MATCH) { rx_foreign++; return; }
    uint8_t dev = msg.id & 0x3F;
    uint8_t cls = (msg.id >> 10) & 0x3F;
    uint8_t idx = (msg.id >>  6) & 0x0F;
    Wheel *wp = wheelForDev(dev);
    if (!wp) { rx_foreign++; return; }
    Wheel &w = *wp;
    rx_count++;
    uint32_t now = millis();

    if (cls == CLS_STATUS) {
        if (idx == IDX_STATUS_2 && msg.len >= 8) {
            w.s2_rx_us = rxMicros(msg);
            w.s2_rx_ms = now;
            w.meas_rpm = u2f(getU32(msg.buf));
            w.meas_pos = u2f(getU32(msg.buf + 4));
            w.got_enc  = true;
            x_pending  = true;
        } else if (idx == IDX_STATUS_0 && msg.len >= 8) {
            uint64_t v = getU64(msg.buf);
            w.applied  = (int16_t)(v & 0xFFFF) * S0_APPLIED_SCALE;
            w.volts    = (uint16_t)((v >> 16) & 0xFFF) * S0_VOLT_SCALE;
            w.amps     = (uint16_t)((v >> 28) & 0xFFF) * S0_CURR_SCALE;
            w.temp_c   = (uint8_t)(v >> 40);
            w.s0_flags = (uint8_t)(v >> 48);             // b0 hardFwd b1 hardRev b2 softFwd b3 softRev b4 inverted b5 hbLock
            w.model    = (uint8_t)((v >> 54) & 0x0F);
            w.got_s0   = true;
            w.s0_rx_ms = now;
            bus_voltage = w.volts;
        } else if (idx == IDX_STATUS_7 && msg.len >= 8) {
            // STATUS_7 (JSON:633): I_ACCUMULATION float, little-endian, bits 0-31 (JSON:641).
            w.iaccum = u2f(getU32(msg.buf));
        } else if (idx == IDX_STATUS_8 && msg.len >= 8) {
            // STATUS_8 (JSON:649): SETPOINT float bits 0-31 (JSON:657), IS_AT_SETPOINT bit 32
            // (JSON:658), SELECTED_PID_SLOT uint4 bits 33-36 (JSON:659), all little-endian.
            uint64_t v = getU64(msg.buf);
            w.s8_sp   = u2f((uint32_t)(v & 0xFFFFFFFFu));
            w.s8_atsp = (int8_t)((v >> 32) & 0x1);
            w.s8_slot = (int8_t)((v >> 33) & 0xF);
        } else if (idx == IDX_STATUS_1 && msg.len >= 8) {
            uint64_t v = getU64(msg.buf) & S1_MEANINGFUL;
            if (!w.got_s1 || v != w.s1_word) {
                w.s1_word = v;
                w.got_s1  = true;
                printFaults(w);
            }
        }
        return;
    }

    if (cls == CLS_CFG) {
        if (idx == IDX_SSE_RESP && msg.len >= 5) {
            uint8_t  res  = msg.buf[0];
            uint16_t mask = msg.buf[1] | ((uint16_t)msg.buf[2] << 8);
            uint16_t en   = msg.buf[3] | ((uint16_t)msg.buf[4] << 8);
            if (res != w.sse_res || en != w.sse_en) {
                out("SSE %c res=%u mask=0x%04x en=0x%04x\n", w.tag, res, mask, en);
            }
            w.sse_res = res; w.sse_en = en;
        } else if (idx == IDX_PERSIST_RESP && msg.len >= 1) {
            w.burn_res = msg.buf[0];
            if (!burnActive()) out("# PERSIST response %c res=%u (unsolicited)\n", w.tag, msg.buf[0]);
        }
        return;
    }

    JobQueue &jq = jqFor(w);
    Job *cur = (jq.busy && jq.count) ? &jq.q[jq.head] : nullptr;

    if (cls == CLS_PARAM && idx == IDX_PARAM_WRITE_RESP && msg.len >= 7) {
        uint8_t  pid  = msg.buf[0];
        uint8_t  ptyp = msg.buf[1];
        uint32_t raw  = getU32(msg.buf + 2);
        uint8_t  res  = msg.buf[6];
        char val[24];
        fmtValue(val, sizeof(val), ptyp, raw);
        const ParamInfo *pi = findParamById(pid);
        bool solicited = cur && cur->kind == JOB_WRITE && cur->id == pid;
        out("PWR %c id=%u type=%u val=%s res=%u %s %s raw=0x%08lx%s\n", w.tag, pid, ptyp, val, res,
            pi ? pi->name : "-", writeResultName(res), (unsigned long)raw, solicited ? "" : " unsolicited");
        if (res == 0) cacheStore(w, pid, raw);   // v2d: VALUE = controller's current value (JSON:5032)
        if (solicited && res == 0 && raw != cur->raw)
            out("# PWR %c id=%u readback 0x%08lx != sent 0x%08lx\n", w.tag, pid, (unsigned long)raw, (unsigned long)cur->raw);
        if (solicited) jobPop(jq);
        return;
    }

    if (cls == CLS_FW && idx == IDX_FW) {
        if (cur && cur->kind == JOB_FW) {
            bool quiet = cur->flags & JF_QUIET;
            if (msg.len >= 6) {
                uint16_t build = ((uint16_t)msg.buf[2] << 8) | msg.buf[3];   // BUILD is big-endian (JSON:1312)
                w.fw_maj = msg.buf[0]; w.fw_min = msg.buf[1]; w.fw_build = build; w.fw_valid = true;
                if (!quiet)
                    out("FV %c %u.%u.%u debug=%u hw=%u dlc_req=%u%s\n", w.tag, msg.buf[0], msg.buf[1], build,
                        msg.buf[4], msg.buf[5], cur->rtr_dlc,
                        (msg.buf[0] == 26 && msg.buf[1] == 1 && build == 5) ? "" : " (expected 26.1.5)");
            } else if (!quiet) {
                out("FV %c UNSUPPORTED dlc=%u\n", w.tag, msg.len);
            }
            jobPop(jq);
        }
        return;
    }

    if (cls >= CLS_READ_BASE && cls < CLS_READ_BASE + 8) {
        uint8_t pair = (uint8_t)(((cls - CLS_READ_BASE) << 4) | idx);
        if (cur && cur->kind == JOB_READ && (cur->id >> 1) == pair) {
            if (msg.len >= 8) {
                uint8_t id_a = pair << 1, id_b = id_a + 1;
                const ParamInfo *pa = findParamById(id_a), *pb = findParamById(id_b);
                uint32_t ra = getU32(msg.buf), rb = getU32(msg.buf + 4);
                cacheStore(w, id_a, ra);          // v2d: CHK cache + motor-type interlock
                cacheStore(w, id_b, rb);
                if (cur->flags & JF_QUIET) { jobPop(jq); return; }
                char va[24], vb[24];
                fmtValue(va, sizeof(va), pa ? pa->type : PT_UNUSED, ra);
                fmtValue(vb, sizeof(vb), pb ? pb->type : PT_UNUSED, rb);
                // Requested id first, its pair partner after.
                bool first = (cur->id == id_a);
                out("PRD %c id=%u type=%c val=%s raw=0x%08lx %s | pair id=%u val=%s\n", w.tag,
                    cur->id, typeChar(first ? (pa ? pa->type : 0) : (pb ? pb->type : 0)),
                    first ? va : vb, (unsigned long)(first ? ra : rb),
                    (first ? pa : pb) ? (first ? pa : pb)->name : "-",
                    first ? id_b : id_a, first ? vb : va);
            } else if (!(cur->flags & JF_QUIET)) {
                out("PRD %c id=%u UNSUPPORTED dlc=%u\n", w.tag, cur->id, msg.len);
            }
            jobPop(jq);
        }
        return;
    }
}

// ---------- Safety light (IGVC §I.2) — unchanged from v1 ---------------------
static void lightFill(bool on) {
    uint32_t c = on ? light.Color(LED_R, LED_G, LED_B) : 0;
    for (uint8_t i = 0; i < LED_COUNT; i++) light.setPixelColor(i, c);
    light.show();                       // only called on transitions / flash edges
}

static void setLightMode(LightMode m) {
    if (m == light_mode) return;
    light_mode = m;
    light_on   = true;
    t_flash    = millis();
    lightFill(true);
}

// Service the light in loop(). show() runs at most on a 2 Hz flash edge, and we
// hold it off if a 50 Hz control tick is imminent so the ~480 us blackout never
// delays a CAN heartbeat.
static void serviceLight(uint32_t now) {
    // Fail-safe: A-line silence (host dead / not autonomous) -> SOLID.
    if (light_mode == LIGHT_FLASH && (now - t_last_auto) > AUTO_TIMEOUT_MS) {
        setLightMode(LIGHT_SOLID);
        return;
    }
    if (light_mode == LIGHT_FLASH
        && (now - t_flash) >= FLASH_HALF_MS
        && (now - t_ctrl)  <  (CTRL_DT_MS - 2)) {   // stay clear of the heartbeat tick
        t_flash  = now;
        light_on = !light_on;
        lightFill(light_on);
    }
}

// ---------- Serial parser helpers --------------------------------------------
// Side token: L, R or B (both). Returns 0 on error.
static char parseSide(const char *tok) {
    if (!tok) return 0;
    char c = toupper((unsigned char)tok[0]);
    if ((c == 'L' || c == 'R' || c == 'B') && tok[1] == '\0') return c;
    return 0;
}

// Parameter token: numeric id or a name from PARAMS. Returns -1 on error.
static int parseParamId(const char *tok) {
    if (!tok || !tok[0]) return -1;
    if (isdigit((unsigned char)tok[0])) {
        char *end;
        long v = strtol(tok, &end, 0);
        if (*end || v < 0 || v > 255) return -1;
        return (int)v;
    }
    const ParamInfo *pi = findParamByName(tok);
    return pi ? pi->id : -1;
}

// Encode a value string per type into the 32-bit wire representation.
static bool encodeValue(uint8_t type, const char *s, uint32_t &raw) {
    if (!s || !s[0]) return false;
    char *end;
    switch (type) {
        case PT_FLOAT: { float f = strtof(s, &end); if (*end) return false; raw = f2u(f); return true; }
        case PT_UINT:  { unsigned long u = strtoul(s, &end, 0); if (*end || s[0] == '-') return false; raw = (uint32_t)u; return true; }
        case PT_INT:   { long i = strtol(s, &end, 0); if (*end) return false; raw = (uint32_t)(int32_t)i; return true; }
        case PT_BOOL:
            if (!strcasecmp(s, "1") || !strcasecmp(s, "true"))  { raw = 1; return true; }
            if (!strcasecmp(s, "0") || !strcasecmp(s, "false")) { raw = 0; return true; }
            return false;
        default: return false;
    }
}
static int typeFromChar(char c) {
    switch (tolower((unsigned char)c)) {
        case 'f': return PT_FLOAT;
        case 'u': return PT_UINT;
        case 'i': return PT_INT;
        case 'b': return PT_BOOL;
        default:  return -1;
    }
}

// Wheels addressed by a side token (L, R or B). Returns the count.
static uint8_t sideWheels(char side, Wheel **ws) {
    uint8_t n = 0;
    if (side == 'L' || side == 'B') ws[n++] = &left;
    if (side == 'R' || side == 'B') ws[n++] = &right;
    return n;
}

// Split into at most `max` whitespace-separated tokens (in place).
static uint8_t tokenize(char *s, char **tok, uint8_t max) {
    uint8_t n = 0;
    char *save = nullptr;
    for (char *t = strtok_r(s, " \t", &save); t && n < max; t = strtok_r(nullptr, " \t", &save)) tok[n++] = t;
    return n;
}

static void markHostCommand(uint32_t now) {
    t_last_host = now;
    wdt_tripped = false;
}

// ---------- Command handlers -------------------------------------------------
static void cmdK(char *line) {
    if (!line[1]) { out("ERR K?\r\n"); return; }
    char which = toupper((unsigned char)line[1]);
    if (which == 'S') {
        // v2c: KS[L|R]<volts> = static-friction feedforward sent in every velocity setpoint's
        // ARBITRARY_FEEDFORWARD field (volts). FW 25 has no kS parameter (param 204 -> invalid_id).
        // v2d (FW 26): this stays the ONLY kS source. Native param 204 would ADD to it (SIM,
        // FW26_CHANGES §1) and applies +kS even at setpoint 0.0 (no deadband), whereas this
        // path is 0 for |setpoint| <= 1 RPM. CHK fails if 204 != 0.
        char sd = 'B'; const char *v = line + 2;
        char c2 = toupper((unsigned char)line[2]);
        if (c2 == 'L' || c2 == 'R') { sd = c2; v = line + 3; }
        float ks = atof(v);
        if (!isfinite(ks) || ks < 0.0f) { out("ERR K?\r\n"); return; }
        if (ks > KS_MAX_VOLTS) ks = KS_MAX_VOLTS;
        if (sd != 'R') left.ks_volts  = ks;
        if (sd != 'L') right.ks_volts = ks;
        if (sd == 'B') out("OK KS=%.4f V (arbFF)\n", (double)ks);
        else           out("OK KS%c=%.4f V (arbFF)\n", sd, (double)ks);
        return;
    }
    int pid = kLetterToParam(which);
    if (pid < 0) { out("ERR K?\r\n"); return; }
    char side = 'B';
    const char *vs = line + 2;
    char s2 = toupper((unsigned char)line[2]);
    if (s2 == 'L' || s2 == 'R') { side = s2; vs = line + 3; }
    float val = atof(vs);
    if (!isfinite(val)) { out("ERR K?\r\n"); return; }   // REVIEW fix: never write a NaN/inf gain
    const ParamInfo *pi = findParamById((uint8_t)pid);
    uint8_t type = pi ? pi->type : PT_FLOAT;      // all K params are FLOAT in PARAM
    uint32_t raw = (type == PT_FLOAT) ? f2u(val) : (uint32_t)(int32_t)val;
    { Wheel *ws[2]; uint8_t nw = sideWheels(side, ws); for (uint8_t i = 0; i < nw; i++) queueWrite(*ws[i], (uint8_t)pid, type, raw); }
    if (side == 'B') out("OK K%c=%.8f\n", which, (double)val);           // v1 byte-identical
    else             out("OK K%c%c=%.8f\n", which, side, (double)val);
    // v2d: kA (205) is applied only in MAXMotion modes on FW 26 (docs table + SIM
    // {s=1,v=1,a=0} in velocity mode, FW26_CHANGES §1); the write still goes out.
    if (which == 'A') out("# KA: param 205 only affects MAXMotion on FW 26\n");
}

// REVIEW S1: parameters whose accidental write can re-address a SPARK, change the motor
// or sensor type, or enable follower/limit behaviour. Such a device keeps running beyond
// the Teensy watchdog's reach. Writing them needs a trailing FORCE token.
// IDs from SparkParameters.java (REVLib 2026.0.5) lines 37-47, 81-95, 112-121, 181-190.
static const uint8_t PW_DENY[] = { 0, 1, 2, 3, 10, 11, 12, 50, 51, 52, 53, 54, 55, 57, 58, 62, 63, 69,
                                   115, 116, 127, 128, 194, 195, 201, 202, 203 };

static bool pwDenied(int pid) {
    for (uint8_t i = 0; i < sizeof(PW_DENY); i++) if (PW_DENY[i] == pid) return true;
    return false;
}

static void cmdPW(char *args) {
    char *tok[6];
    uint8_t n = tokenize(args, tok, 6);
    bool force = (n >= 4 && strcasecmp(tok[n - 1], "FORCE") == 0);
    if (force) n--;
    if (n != 3 && n != 4) { out("ERR PW usage: PW <L|R|B> <id|name> [f|u|i|b] <value> [FORCE]\n"); return; }
    char side = parseSide(tok[0]);
    int pid = parseParamId(tok[1]);
    if (!side || pid < 0) { out("ERR PW side/id?\n"); return; }
    if (pwDenied(pid) && !force) {
        out("ERR PW id=%d is protected (CAN ID / motor / sensor / follower / limits); append FORCE to write\n", pid);
        return;
    }
    const ParamInfo *pi = findParamById((uint8_t)pid);
    int type;
    const char *vs;
    if (n == 4) {
        type = typeFromChar(tok[2][0]);
        if (type < 0 || tok[2][1]) { out("ERR PW type? (f|u|i|b)\n"); return; }
        vs = tok[3];
    } else {
        if (!pi) { out("ERR PW id %d not in table: give a type (f|u|i|b)\n", pid); return; }
        type = pi->type;
        vs = tok[2];
    }
    uint32_t raw;
    if (!encodeValue((uint8_t)type, vs, raw)) { out("ERR PW value?\n"); return; }
    if (pi && pi->type != type) out("# PW id=%d table type is %c, sending as %c\n", pid, typeChar(pi->type), typeChar(type));
    {
        Wheel *ws[2]; uint8_t nw = sideWheels(side, ws);
        for (uint8_t i = 0; i < nw; i++) {
            queueWrite(*ws[i], (uint8_t)pid, (uint8_t)type, raw);
            if (pid == 2) queueMotorTypeRead(*ws[i], 0);   // v2d: re-check the interlock after the write
        }
    }
    char val[24];
    fmtValue(val, sizeof(val), (uint8_t)type, raw);
    out("OK PW %c id=%d type=%c val=%s raw=0x%08lx\n", side, pid, typeChar(type), val, (unsigned long)raw);
    // v2d FW 26 notes (FW26_CHANGES §1)
    if (pid == 204 && raw != 0)
        out("# PW id=204: native kS adds to the KS arbFF and applies +kS at setpoint 0; keep 204 = 0 and use KS\n");
    if (pid == 205) out("# KA: param 205 only affects MAXMotion on FW 26\n");
    if (pid == 16 && type == PT_FLOAT && u2f(raw) != 0.0f && fabsf(u2f(raw)) < 5e-4f)
        out("# PW id=16: kV looks like a duty/RPM value; FW 26 uses V/RPM\n");
}

static void cmdPR(char *args) {
    char *tok[3];
    uint8_t n = tokenize(args, tok, 3);
    if (n != 2) { out("ERR PR usage: PR <L|R|B> <id|name>\n"); return; }
    char side = parseSide(tok[0]);
    int pid = parseParamId(tok[1]);
    if (!side || pid < 0) { out("ERR PR side/id?\n"); return; }
    { Wheel *ws[2]; uint8_t nw = sideWheels(side, ws); for (uint8_t i = 0; i < nw; i++) queueRead(*ws[i], (uint8_t)pid); }
    out("OK PR %c id=%d\n", side, pid);
}

static void cmdPT() {
    for (uint8_t i = 0; i < N_PARAMS; i++) {
        out("PT %u %c %s\n", PARAMS[i].id, typeChar(PARAMS[i].type), PARAMS[i].name);
    }
}

static void cmdDiag() {
    // v1 fields first, byte-identical in order and format; v2 fields appended.
    out("DIAG tx=%lu rx=%lu wdt=%d mode=%s L=%.0f/%.0f/%.0f R=%.0f/%.0f/%.0f duty L=%.3f R=%.3f V=%.2f M=%.2f burn=%d/%d"
        " | volt L=%.2f R=%.2f cap=%.2f vcap=%.2f txq=%lu txfail=%lu foreign=%lu sdrop=%lu hb=%d burnst=%d"
        " hblock=%d/%d inv=%d/%d model=%u/%u app=%.3f/%.3f I=%.1f/%.1f T=%u/%u sse=%d:0x%04x/%d:0x%04x"
        " f=0x%02x/0x%02x sf=0x%02x/0x%02x jq=%u/%u x=%d rxage=%lu ks=%.3f/%.3f arb=%.3f/%.3f"
        " iacc=%.6g/%.6g sp8=%.2f/%.2f atsp=%d/%d slot=%d/%d mt=%ld/%ld blk=%d/%d blkn=%lu/%lu chk=%d\n",
        (unsigned long)tx_count, (unsigned long)rx_count, wdt_tripped ? 1 : 0,
        ctrl_mode == MODE_DUTY ? "DUTY" : (ctrl_mode == MODE_VOLTAGE ? "VOLT" : "VEL"),
        (double)left.meas_rpm, (double)left.ramp_rpm, (double)left.cmd_rpm,
        (double)right.meas_rpm, (double)right.ramp_rpm, (double)right.cmd_rpm,
        (double)left.cmd_duty, (double)right.cmd_duty,
        (double)bus_voltage, (double)max_rpm_step,
        left.burn_res, right.burn_res,
        (double)left.cmd_volt, (double)right.cmd_volt, (double)duty_cap, (double)voltCap(),
        (unsigned long)tx_queued, (unsigned long)tx_fail, (unsigned long)rx_foreign, (unsigned long)ser_drop,
        hb_enabled ? 1 : 0, (int)burn_state,
        (left.s0_flags >> 5) & 1, (right.s0_flags >> 5) & 1,
        (left.s0_flags >> 4) & 1, (right.s0_flags >> 4) & 1,
        left.model, right.model,
        (double)left.applied, (double)right.applied, (double)left.amps, (double)right.amps,
        left.temp_c, right.temp_c,
        left.sse_res, left.sse_en, right.sse_res, right.sse_en,
        (uint8_t)left.s1_word, (uint8_t)right.s1_word,
        (uint8_t)(left.s1_word >> 24), (uint8_t)(right.s1_word >> 24),
        jq_left.count, jq_right.count, x_enabled ? 1 : 0, (unsigned long)max_rx_age_us,
        (double)left.ks_volts, (double)right.ks_volts, (double)left.sent_arb, (double)right.sent_arb,
        (double)left.iaccum, (double)right.iaccum, (double)left.s8_sp, (double)right.s8_sp,
        left.s8_atsp, right.s8_atsp, left.s8_slot, right.s8_slot,
        left.mt_known ? (long)left.motor_type : -1L, right.mt_known ? (long)right.motor_type : -1L,
        motionBlocked(left) ? 1 : 0, motionBlocked(right) ? 1 : 0,
        (unsigned long)left.blk_frames, (unsigned long)right.blk_frames, chk_active ? 1 : 0);
}

static void emitTelemetry() {
    // X <teensy_us> L <vel_rpm> <pos_rot> <status2_rx_us> <applied> <current_A> <bus_V> <temp_C>
    //               R <...same...> SP <l_setpoint> <r_setpoint> <sp_tx_us_L> <sp_tx_us_R> <mode>
    //   v2d, appended after <mode> (tools parse by index; fields 0-23 unchanged):
    //               I7 <L_iaccum> <R_iaccum> SP8 <L_setpoint> <R_setpoint> <L_atsp> <R_atsp>
    //   STATUS_7/8 values; nan / -1 until the first frame arrives.
    out("X %lu L %.2f %.5f %lu %.4f %.2f %.2f %u R %.2f %.5f %lu %.4f %.2f %.2f %u SP %.2f %.2f %lu %lu %c"
        " I7 %.6g %.6g SP8 %.2f %.2f %d %d\n",
        (unsigned long)micros(),
        (double)left.meas_rpm, (double)left.meas_pos, (unsigned long)left.s2_rx_us,
        (double)left.applied, (double)left.amps, (double)left.volts, left.temp_c,
        (double)right.meas_rpm, (double)right.meas_pos, (unsigned long)right.s2_rx_us,
        (double)right.applied, (double)right.amps, (double)right.volts, right.temp_c,
        (double)left.sent_sp, (double)right.sent_sp,
        (unsigned long)left.sp_tx_us, (unsigned long)right.sp_tx_us,
        ctrl_mode == MODE_DUTY ? 'D' : (ctrl_mode == MODE_VOLTAGE ? 'U' : 'V'),
        (double)left.iaccum, (double)right.iaccum, (double)left.s8_sp, (double)right.s8_sp,
        left.s8_atsp, right.s8_atsp);
}

// ---------- Serial parser ---------------------------------------------------
static void handleLine(char *line) {
    if (!line[0]) return;
    char cmd = toupper((unsigned char)line[0]);
    char c1  = toupper((unsigned char)line[1]);
    uint32_t now = millis();

    switch (cmd) {
        case 'S':
            // Stop both wheels. STOP_TO_IDLE (default): duty 0, so the SPARK's idle mode
            // (Brake = windings shorted, "quick stop", C3-S48) stops the motor. Bench
            // 2026-09-28: a velocity-0 stop slammed the PID to full reverse (46-57 A, bus
            // sag to 8 V, DRV faults, >2 s of ringing); duty 0 + Brake stopped in ~0.3 s
            // with no overshoot at 3-12 A. The host already ramps speed down before it
            // sends S, so a normal stop = controlled decel then brake at standstill (SS1
            // pattern, C3 section 7). The next L/R command resumes velocity mode from 0.
            left.cmd_rpm  = right.cmd_rpm  = 0.0f;
            left.ramp_rpm = right.ramp_rpm = 0.0f;
            left.cmd_duty = right.cmd_duty = 0.0f;
            left.cmd_volt = right.cmd_volt = 0.0f;
            ctrl_mode = STOP_TO_IDLE ? MODE_DUTY : MODE_VELOCITY;
            markHostCommand(now);
            out("OK S\r\n");
            return;

        case 'D':
            cmdDiag();
            return;

        case 'U': {
            if (c1 == 'V') {
                // Voltage command: "UVL1.2 UVR-1.2" / "UVL1.2" / "UVR-1.2"
                char *lp = strchr(line + 2, 'L'); if (!lp) lp = strchr(line + 2, 'l');
                char *rp = strchr(line + 2, 'R'); if (!rp) rp = strchr(line + 2, 'r');
                if (!lp && !rp) { out("ERR UV?\n"); return; }
                if (lp) left.cmd_volt  = clampVolt(atof(lp + 1));
                if (rp) right.cmd_volt = clampVolt(atof(rp + 1));
                ctrl_mode = MODE_VOLTAGE;
                markHostCommand(now);
                out("OK UVL=%.3f UVR=%.3f\n", (double)left.cmd_volt, (double)right.cmd_volt);
                return;
            }
            // Duty-cycle command:  "UL0.05 UR-0.05" / "UL0.05" / "UR-0.05"
            char *lp = strchr(line + 1, 'L'); if (!lp) lp = strchr(line + 1, 'l');
            char *rp = strchr(line + 1, 'R'); if (!rp) rp = strchr(line + 1, 'r');
            if (!lp && !rp) { out("ERR U?\r\n"); return; }
            if (lp) left.cmd_duty  = clampDuty(atof(lp + 1));
            if (rp) right.cmd_duty = clampDuty(atof(rp + 1));
            ctrl_mode = MODE_DUTY;
            markHostCommand(now);
            out("OK UL=%.3f UR=%.3f\n", (double)left.cmd_duty, (double)right.cmd_duty);
            return;
        }

        case 'B':
            // Safe persist; completes asynchronously with "OK BURN result L= R=".
            burnStart(now);
            return;

        case 'A':
            // IGVC §I.2 safety-light mode (v1). Refreshes the light heartbeat
            // ONLY (not the motor watchdog). Acks only on a state change.
            if (line[1] == '1' || line[1] == '0') {
                t_last_auto = now;
                LightMode m = (line[1] == '1') ? LIGHT_FLASH : LIGHT_SOLID;
                if (m != light_mode) {
                    setLightMode(m);
                    out("OK A%c\r\n", line[1]);   // v1: print + println
                }
            } else {
                out("ERR A?\r\n");
            }
            return;

        case 'M':
            if (c1 == 'D') {
                // Runtime duty/voltage cap. Hard ceiling DUTY_CAP_HARD / 7.2 V.
                if (!line[2]) { out("ERR MD?\n"); return; }
                float d = atof(line + 2);
                if (!(d >= 0.0f)) d = 0.0f;           // REVIEW fix: also catches NaN (NaN cap disabled every clamp)
                if (d > DUTY_CAP_HARD) d = DUTY_CAP_HARD;
                duty_cap = d;
                left.cmd_duty  = clampDuty(left.cmd_duty);  right.cmd_duty = clampDuty(right.cmd_duty);
                left.cmd_volt  = clampVolt(left.cmd_volt);  right.cmd_volt = clampVolt(right.cmd_volt);
                out("OK MD=%.3f VCAP=%.2f\n", (double)duty_cap, (double)voltCap());
                return;
            }
            // v1: velocity slew rate, RPM per 20 ms control tick.
            if (!line[1]) { out("ERR M?\r\n"); return; }
            max_rpm_step = atof(line + 1);
            if (max_rpm_step < 0.0f) max_rpm_step = 0.0f;
            out("OK M=%.2f\n", max_rpm_step);
            return;

        case 'K':
            cmdK(line);
            return;

        case 'P':
            if (c1 == 'W' && (line[2] == ' ' || line[2] == '\t')) { cmdPW(line + 2); return; }
            if (c1 == 'R' && (line[2] == ' ' || line[2] == '\t')) { cmdPR(line + 2); return; }
            if (c1 == 'T' && !line[2]) { cmdPT(); return; }
            out("ERR P?\n");
            return;

        case 'F':
            if (c1 == 'V' && !line[2]) {
                queueFw(left); queueFw(right);
                out("OK FV\n");
                return;
            }
            out("ERR F?\n");
            return;

        case 'C':
            if (c1 == 'H' && toupper((unsigned char)line[2]) == 'K' && !line[3]) {
                chkStart(false);   // v2d configuration check; result arrives as CHK lines
                return;
            }
            if (c1 == 'F') {
                char *tok[2];
                uint8_t n = tokenize(line + 2, tok, 2);
                char side = (n == 0) ? 'B' : parseSide(tok[0]);
                if (!side) { out("ERR CF side?\n"); return; }
                { Wheel *ws[2]; uint8_t nw = sideWheels(side, ws); for (uint8_t i = 0; i < nw; i++) sendClearFaults(ws[i]->dev); }
                out("OK CF %c\n", side);
                return;
            }
            out("ERR C?\n");
            return;

        case 'I':
            if (c1 == 'D') {
                char *tok[2];
                uint8_t n = tokenize(line + 2, tok, 2);
                char side = (n == 0) ? 'B' : parseSide(tok[0]);
                if (!side) { out("ERR ID side?\n"); return; }
                { Wheel *ws[2]; uint8_t nw = sideWheels(side, ws); for (uint8_t i = 0; i < nw; i++) sendIdentify(ws[i]->dev); }
                out("OK ID %c\n", side);
                return;
            }
            out("ERR I?\n");
            return;

        case 'H':
            // Bench test of the DISABLED heartbeat (the one BURN uses): HB0 sends the
            // disabled universal + secondary heartbeat for up to 5 s (auto-restores); HB1
            // restores now. While disabled, a streamed command (e.g. UL0.05) must NOT move
            // the tracks and STATUS_0 applied output must read 0. Refused during BURN.
            if (line[1] == 'B' || line[1] == 'b') {
                if (burnActive()) { out("ERR HB busy (BURN)\n"); return; }
                if (line[2] == '0') {
                    hb_enabled = false;
                    hb_test_until = now + HB_TEST_MAX_MS;
                    if (hb_test_until == 0) hb_test_until = 1;
                    out("OK HB0 (disabled heartbeat for up to %lu ms)\n", (unsigned long)HB_TEST_MAX_MS);
                    return;
                }
                if (line[2] == '1') {
                    hb_enabled = true;
                    hb_test_until = 0;
                    out("OK HB1\n");
                    return;
                }
            }
            out("ERR HB?\n");
            return;

        case 'X':
            if (line[1] == '1' || line[1] == '0') {
                x_enabled = (line[1] == '1');
                x_pending = false;
                out("OK X%c\n", line[1]);
            } else {
                out("ERR X?\n");
            }
            return;

        default: {
            // Velocity command: "L500 R-500" / "L500" / "R-500"  (v1)
            char *lp = strchr(line, 'L'); if (!lp) lp = strchr(line, 'l');
            char *rp = strchr(line, 'R'); if (!rp) rp = strchr(line, 'r');
            if (!lp && !rp) { out("ERR unknown\r\n"); return; }
            auto clamp = [](float v) {
                if (isnan(v)) return 0.0f;            // REVIEW fix: "Lnan" must not reach the ramp
                if (v >  MAX_RPM) return  MAX_RPM;
                if (v < -MAX_RPM) return -MAX_RPM;
                return v;
            };
            if (lp) left.cmd_rpm  = clamp(atof(lp + 1));
            if (rp) right.cmd_rpm = clamp(atof(rp + 1));
            ctrl_mode = MODE_VELOCITY;
            markHostCommand(now);
            out("OK L=%.0f R=%.0f\n", (double)left.cmd_rpm, (double)right.cmd_rpm);
            return;
        }
    }
}

static void processSerial() {
    static char buf[128];
    static uint8_t n = 0;
    // Bounded per loop pass so a serial flood cannot delay the 20 ms tick.
    for (uint16_t budget = 256; budget && Serial.available(); budget--) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (n > 0) { buf[n] = '\0'; handleLine(buf); n = 0; }
        } else if (n < sizeof(buf) - 1) {
            buf[n++] = c;
        }
    }
}

// ---------- setup / loop ----------------------------------------------------
void setup() {
    left.dev  = LEFT_ID;  left.tag  = 'L';
    right.dev = RIGHT_ID; right.tag = 'R';

    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println("# avros diff-drive bridge v2d ready (SPARK MAX FW 26.1.5)");
    Serial.println("# proto: L<rpm> R<rpm> | UL<d> UR<d> | UVL<V> UVR<V> | S | D | K[PIDFVZSA][L|R]<v> | M<v> | MD<d> | BURN | A0/A1"
                   " | PW <L|R|B> <id|name> [f|u|i|b] <v> | PR <L|R|B> <id|name> | PT | FV | CF [L|R|B] | ID [L|R|B] | X0/X1 | CHK");

    // IGVC §I.2 safety light: SOLID amber the instant the Teensy boots, before
    // CAN bring-up, so it is on whenever the vehicle has power (rule fail-safe).
    light.begin();
    light.setBrightness(LED_BRIGHT);
    lightFill(true);
    t_last_auto = millis();

    can.begin();
    can.setBaudRate(CAN_BAUD);
    can.setMaxMB(16);
    can.enableFIFO();
    can.enableFIFOInterrupt();
#if USE_HW_RX_FILTER
    // Data frames only (RTR bit must be 0), extended, devType 2 + mfg 5.
    can.setFIFOFilter(REJECT_ALL);
    can.setFIFOUserFilter(0, SPARK_ID_MATCH, SPARK_ID_MASK, EXT);
#else
    can.setFIFOFilter(ACCEPT_ALL);
#endif
    can.onReceive(onCanRx);

    // NOTE: intentionally no parameter pushes here -- SPARK flash is
    // authoritative; actuator_node pushes its gains at startup.
    t_last_host = millis();
    t_ctrl      = millis() - CTRL_DT_MS;     // first heartbeat on the first loop pass
    t_enc_cfg   = millis() - ENC_CFG_DT_MS;  // first SET_STATUSES_ENABLED immediately
    t_boot      = millis();                  // v2d: automatic CHK BOOT_CHK_DELAY_MS later
    t_mt_check  = millis() - MT_CHECK_MS;    // review: first motor-type read on the first loop pass
}

void loop() {
    // Drain the RX queue (can.events() dispatches one frame per call).
    for (uint8_t i = 0; i < 32; i++) {
        if (((can.events() >> 12) & 0xFFFF) == 0) break;
    }
    processSerial();

    uint32_t now = millis();

    // No !Serial guard (v1): bool(Serial) can flicker under heavy CDC traffic.
    // The 300 ms host watchdog stops the motors on USB disconnect.

    // 50 Hz control + heartbeat tick
    if (now - t_ctrl >= CTRL_DT_MS) {
        t_ctrl = now;

        if (now - t_last_host > WATCHDOG_MS) {
            if (!wdt_tripped) {
                out("# WDT host-timeout stop\r\n");
                wdt_tripped = true;
            }
            left.cmd_rpm  = right.cmd_rpm  = 0.0f;
            left.ramp_rpm = right.ramp_rpm = 0.0f;   // immediate, unramped stop
            left.cmd_duty = right.cmd_duty = 0.0f;
            left.cmd_volt = right.cmd_volt = 0.0f;
            if (STOP_TO_IDLE) ctrl_mode = MODE_DUTY;  // host lost: duty 0 -> idle (Brake) stop, see case 'S'
        }

        sendHeartbeats();
        if (burnActive()) {
            // Output held at zero while the heartbeat says disabled; the ramp
            // restarts from 0 once the enabled heartbeat is restored.
            left.ramp_rpm = right.ramp_rpm = 0.0f;
            setVelocity(left,  0.0f);
            setVelocity(right, 0.0f);
        } else if (ctrl_mode == MODE_DUTY) {
            // REVIEW fix: a stale ramp_rpm from an earlier velocity run would
            // otherwise be sent as a step the moment L/R resumes (e.g. L1000,
            // UL0, L0 -> first velocity frame 900 RPM to a stopped wheel).
            left.ramp_rpm = right.ramp_rpm = 0.0f;
            setDuty(left,  left.cmd_duty);
            setDuty(right, right.cmd_duty);
        } else if (ctrl_mode == MODE_VOLTAGE) {
            left.ramp_rpm = right.ramp_rpm = 0.0f;   // REVIEW fix: see MODE_DUTY
            setVoltage(left,  left.cmd_volt);
            setVoltage(right, right.cmd_volt);
        } else {
            // FINAL CHECK: while HB0 holds the SPARKs disabled, keep the ramp at 0 so
            // re-enable (HB1 / 5 s auto-restore) does not step straight to cmd_rpm.
            if (!hb_enabled) left.ramp_rpm = right.ramp_rpm = 0.0f;
            slewToward(left.ramp_rpm,  left.cmd_rpm,  max_rpm_step);
            slewToward(right.ramp_rpm, right.cmd_rpm, max_rpm_step);
            setVelocity(left,  left.ramp_rpm);
            setVelocity(right, right.ramp_rpm);
        }
    }

    // Status-enable keepalive: a SPARK reboot clears the STATUS_2/7/8 enables, so
    // re-assert STATUS_0,1,2,7,8 every ENC_CFG_DT_MS (harmless on 26.1.3+, which no
    // longer restarts an enabled frame's timer; FW26_CHANGES §2). The response is decoded in
    // onCanRx and printed only when it changes.
    if (now - t_enc_cfg >= ENC_CFG_DT_MS) {
        t_enc_cfg = now;
        sendStatusesEnabled(LEFT_ID);
        sendStatusesEnabled(RIGHT_ID);
    }

    // v2d: automatic configuration check ~2 s after boot, then a quiet motor-type
    // re-read every 5 s (interlock). Both go through the non-blocking job queue.
    if (chk_boot_pending && now - t_boot >= BOOT_CHK_DELAY_MS && !burnActive()) {
        chk_boot_pending = false;
        chkStart(true);
    }
    if (now - t_mt_check >= MT_CHECK_MS) {
        t_mt_check = now;
        if (!burnActive() && !chk_active) { queueMotorTypeRead(left, JF_QUIET); queueMotorTypeRead(right, JF_QUIET); }
    }

    serviceJobs();
    if (chk_active && chk_outstanding == 0) chkReport();
    serviceBurn(now);
    if (hb_test_until && (int32_t)(now - hb_test_until) >= 0 && !burnActive()) {
        hb_enabled = true;
        hb_test_until = 0;
        out("# HB test ended: enabled heartbeat restored\n");
    }

    // 50 Hz wheel feedback to host (format byte-identical to v1)
    if (now - t_fb >= FEEDBACK_DT_MS) {
        t_fb = now;
        if (Serial.availableForWrite() >= 64) {   // non-blocking guard
            Serial.printf("E L%.0f %.4f R%.0f %.4f\n",
                          (double)left.meas_rpm, (double)left.meas_pos,
                          (double)right.meas_rpm, (double)right.meas_pos);
        }
    }

    // Optional high-rate telemetry: one line per new STATUS_2 batch.
    if (x_enabled && x_pending) {
        x_pending = false;
        emitTelemetry();
    }

    // IGVC §I.2 safety light — solid (manual) / 2 Hz flash (autonomous)
    serviceLight(now);
}
