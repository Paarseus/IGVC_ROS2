// ============================================================================
// Adafruit_NeoPixel diagnostic — IGVC safety light bring-up
// ============================================================================
// Dead-simple reference-library test for the Adafruit NeoPixel Ring 16 on a
// Teensy 4.1. Uses Adafruit's own library (the gold-standard WS2812 driver),
// which bit-bangs any GPIO, so it removes WS2812Serial / pin-DMA from the
// equation. Purpose: split "code/library problem" from "wiring problem".
//
//   If this lights up but the WS2812Serial sketch didn't -> library/pin issue.
//   If this ALSO shows nothing -> wiring (DIN pad? common ground? dead first
//   pixel from an earlier 3.3V-into-5V hookup?), NOT the firmware.
//
// Wiring (Adafruit Ring 16, product 1463):
//   Teensy pin 20 --> 470 ohm --> ring "Data Input" (DIN, arrow INTO ring)
//   ring "PWR/5V"  --> +5V   (or ~4.3V via series diode; 4-7V per Adafruit)
//   ring "GND"     --> Teensy GND   (COMMON GROUND - mandatory)
//   "Data Output" (DOUT) left unconnected.
//
// NOTE: changing the library does NOT fix the 3.3V-data-into-5V-ring logic
// level (V_IH = 0.7*VDD = 3.5V at 5V). If you power the ring at 5V you still
// need a level shifter or the diode-drop trick. Powering ring + data both at
// 3.3V matches them (V_IH = 2.31V) and is fine for this bench test.
//
// Cycle: ALL pixels RED -> GREEN -> BLUE -> AMBER, 1 s each, repeat.
// Serial monitor @ 115200 prints each state.
// ============================================================================

#include <Adafruit_NeoPixel.h>

#define LED_PIN    20
#define LED_COUNT  16

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

static void fillAll(uint8_t r, uint8_t g, uint8_t b, const char *name) {
    for (uint16_t i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
    Serial.print("# ALL = ");
    Serial.println(name);
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println("# Adafruit_NeoPixel test: pin 20, 16 px, GRB 800kHz");

    strip.begin();
    strip.setBrightness(60);   // ~24% — modest current, still clearly visible
    strip.show();              // start all-off
    Serial.println("# strip.begin() done, cycling colors...");
}

void loop() {
    fillAll(150,   0,   0, "RED");    delay(1000);
    fillAll(  0, 150,   0, "GREEN");  delay(1000);
    fillAll(  0,   0, 150, "BLUE");   delay(1000);
    fillAll(150,  80,   0, "AMBER");  delay(1000);
}
