// ============================================================================
// Teensy 4.1 — IGVC §I.2 safety-light bench state machine
// ============================================================================
// Serial-driven SOLID/FLASH state machine for the Adafruit NeoPixel Ring 16.
// This is the bench precursor to merging the same logic into
// teensy_diff_drive.ino (driven by actuator_node's autonomous-mode flag).
//
// Rule (IGVC 2026 §I.2 / §I.4 qualification):
//   - Light SOLID whenever the vehicle is powered up.
//   - Light FLASHING whenever the vehicle is in autonomous mode.
//   - Back to SOLID as soon as it leaves autonomous mode.
//   SOLID is the fail-safe default — it must appear on power-up before anything
//   else, and every failure path collapses back to it.
//
// Library: Adafruit_NeoPixel (bit-bang, any GPIO). WS2812Serial was abandoned —
// it did not drive pin 20 on this hardware (silent begin/DMA failure -> floating
// data line -> colorful flicker). show() is called ONLY on a state change or
// flash edge, never every loop, so the IRQ blackout (~480 us / 16 px) stays a
// tiny fraction of time — the pattern the production firmware must preserve so
// it never disturbs the 50 Hz CAN heartbeat to the SparkMAXes.
//
// Wiring (Adafruit NeoPixel Ring 16, product 1463):
//   Teensy pin 20 --> 470 ohm --> ring "Data Input" (DIN; arrow INTO ring)
//   ring "PWR/5V" --> 5V  (production: 74AHCT125 level shift with OE->GND, or
//                          diode-drop VDD to ~4.3V). Bench may run ring + logic
//                          both at 3.3V (dim, green-starved, but logic-valid).
//   ring "GND"    --> Teensy GND  (COMMON GROUND — mandatory)
//   "Data Output" (DOUT) left unconnected.
//
// Host serial protocol (115200, line-oriented), mirrors teensy_diff_drive style:
//   A1\n  -> AUTONOMOUS  : flash at 2 Hz
//   A0\n  -> MANUAL/IDLE : solid
//   Teensy acks "OK A1" / "OK A0" ONLY on an actual state change (no per-line
//   flood; the periodic A-lines act as a silent heartbeat).
// Fail-safe: no valid A-line for SAFETY_TIMEOUT_MS -> revert to SOLID.
//   (In production the A-line piggybacks the 50 Hz command stream; a dead host
//    means the motor watchdog has already stopped the robot -> not autonomous
//    -> SOLID is the correct and safe indication.)
//
// Bench test from the Jetson:
//   while true; do echo A1 > /dev/ttyACM0; sleep 0.2; done   # hold autonomous
//   echo A0 > /dev/ttyACM0                                    # back to manual
//   (stop sending A1 -> reverts to SOLID after the timeout)
// ============================================================================

#include <Adafruit_NeoPixel.h>

static constexpr uint8_t  LED_PIN            = 20;
static constexpr uint8_t  LED_COUNT          = 16;
static constexpr uint8_t  LED_R              = 255;   // amber
static constexpr uint8_t  LED_G              = 140;
static constexpr uint8_t  LED_B              = 0;
static constexpr uint8_t  LED_BRIGHT         = 190;   // ~75% of 255
static constexpr uint32_t FLASH_HALF_MS      = 250;   // 2 Hz, 50% duty
static constexpr uint32_t SAFETY_TIMEOUT_MS  = 750;   // host silence -> SOLID

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

enum Mode { MODE_SOLID, MODE_FLASH };
static Mode     mode     = MODE_SOLID;   // fail-safe default
static bool     flash_on = true;
static uint32_t t_flash  = 0;
static uint32_t t_host   = 0;            // millis() of last valid A-line
static char     line[16];
static uint8_t  line_len = 0;

static void show_amber(bool on) {
    uint32_t c = on ? strip.Color(LED_R, LED_G, LED_B) : 0;
    for (uint8_t i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, c);
    strip.show();                        // only called on transitions / flash edge
}

static void set_mode(Mode m) {
    if (m == mode) return;               // no-op if unchanged (avoids ack flood)
    mode     = m;
    flash_on = true;
    t_flash  = millis();
    show_amber(true);
    Serial.println(mode == MODE_FLASH ? "OK A1" : "OK A0");
}

static void handle_line() {
    if (line_len == 2 && line[0] == 'A' && (line[1] == '0' || line[1] == '1')) {
        t_host = millis();               // valid host heartbeat
        set_mode(line[1] == '1' ? MODE_FLASH : MODE_SOLID);
    }
    line_len = 0;
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}
    strip.begin();
    strip.setBrightness(LED_BRIGHT);
    show_amber(true);                    // SOLID immediately on power (rule)
    Serial.println("# safety light up: SOLID on boot; A1=flash A0=solid");
    t_host = millis();
}

void loop() {
    // --- ingest serial lines ---
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (line_len) handle_line();
        } else if (line_len < sizeof(line) - 1) {
            line[line_len++] = ch;
        } else {
            line_len = 0;                // overflow -> drop
        }
    }

    uint32_t now = millis();

    // --- fail-safe: host silence reverts FLASH -> SOLID ---
    if (mode == MODE_FLASH && (now - t_host) > SAFETY_TIMEOUT_MS) {
        Serial.println("# host-timeout");
        set_mode(MODE_SOLID);
    }

    // --- flash toggle (show() only on the 2 Hz edge) ---
    if (mode == MODE_FLASH && (now - t_flash) >= FLASH_HALF_MS) {
        t_flash  = now;
        flash_on = !flash_on;
        show_amber(flash_on);
    }
}
