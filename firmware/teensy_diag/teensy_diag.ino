// ============================================================================
// Teensy 4.1 — SparkMAX CAN sniffer / diagnostic bridge
// ============================================================================
// Purpose: passively observe what CAN frames SparkMAX (FW 26.1.4) emits.
// Sends only heartbeats (so SparkMAX stays enabled — solid magenta LED).
// DOES NOT send velocity setpoints. Safe to run with NEO motors connected.
//
// Output per RX frame (115200 baud):
//   RX id=0x02051841 cls=06 idx=01 dev=01 dlc=8 b=AA BB CC DD EE FF 00 00
//          f0=1.234567 f1=0.000000  (float32 LE decode of [0:3] and [4:7])
//
// Periodic summary every 2 s:
//   -- seen: cls=06 idx=01 dev=01 n=100 last_f0=... last_f1=...
//            cls=06 idx=02 dev=01 n=100 ...
//            (grouped by cls/idx/dev)
//
// Serial commands:
//   D              print summary immediately
//   R              reset summary counters
//   Q              quiet mode — suppress per-frame prints, keep summaries
//   V              verbose mode — print every frame (default)
//   TX <id_hex> <b0> ... <b7>   raw transmit (for param GET experiments)
// ============================================================================

#include <FlexCAN_T4.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

static constexpr uint32_t CAN_BAUD      = 1000000;
static constexpr uint32_t HB_DT_MS      = 20;    // 50 Hz heartbeats
static constexpr uint32_t SUMMARY_DT_MS = 2000;  // 0.5 Hz summary
static constexpr uint32_t UNIVERSAL_HB  = 0x01011840;

// SparkMAX CAN ID decode (extended 29-bit):
// bits 24-28: devType (5b) -- SparkMAX = 2
// bits 16-23: mfg     (8b) -- REV = 5
// bits 10-15: cls     (6b)
// bits  6- 9: idx     (4b)
// bits  0- 5: dev     (6b)

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;

struct Seen {
    uint32_t id;      // full CAN ID
    uint8_t  cls, idx, dev;
    uint32_t count;
    float    last_f0, last_f1;
    uint8_t  last_bytes[8];
};
static constexpr int MAX_SEEN = 32;
static Seen seen[MAX_SEEN];
static int  n_seen = 0;

static uint32_t t_hb = 0, t_sum = 0, tx_count = 0, rx_count = 0;
static bool     verbose = true;

static inline uint32_t sparkId(uint8_t cls, uint8_t idx, uint8_t dev) {
    return ((uint32_t)2 << 24) | ((uint32_t)5 << 16)
         | ((uint32_t)(cls & 0x3F) << 10)
         | ((uint32_t)(idx & 0x0F) <<  6)
         | ((uint32_t)(dev & 0x3F));
}

static void canSend(uint32_t id, const uint8_t *data, uint8_t len) {
    CAN_message_t m;
    m.flags.extended = 1;
    m.id = id;
    m.len = len;
    memcpy(m.buf, data, len);
    if (can.write(m) > 0) tx_count++;
}

static inline float toFloat(const uint8_t *b) { float v; memcpy(&v, b, sizeof(float)); return v; }

static void sendHeartbeats() {
    static const uint8_t uni[8] = {0x78, 0x01, 0x00, 0x12, 0x59, 0x04, 0x00, 0x60};
    canSend(UNIVERSAL_HB, uni, 8);
    static const uint8_t sec[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    canSend(sparkId(11, 2, 0), sec, 8);
}

static Seen *findOrAdd(uint32_t id, uint8_t cls, uint8_t idx, uint8_t dev) {
    for (int i = 0; i < n_seen; i++) {
        if (seen[i].id == id) return &seen[i];
    }
    if (n_seen >= MAX_SEEN) return nullptr;
    Seen &s = seen[n_seen++];
    s.id = id; s.cls = cls; s.idx = idx; s.dev = dev;
    s.count = 0;
    return &s;
}

static void onCanRx(const CAN_message_t &msg) {
    if (!msg.flags.extended) return;
    rx_count++;

    uint8_t dev = msg.id & 0x3F;
    uint8_t idx = (msg.id >> 6) & 0x0F;
    uint8_t cls = (msg.id >> 10) & 0x3F;

    Seen *s = findOrAdd(msg.id, cls, idx, dev);
    if (s) {
        s->count++;
        if (msg.len >= 4) s->last_f0 = toFloat(msg.buf);
        if (msg.len >= 8) s->last_f1 = toFloat(msg.buf + 4);
        memcpy(s->last_bytes, msg.buf, msg.len < 8 ? msg.len : 8);
    }

    if (verbose && Serial.availableForWrite() >= 96) {
        Serial.printf("RX id=0x%08lX cls=%02u idx=%02u dev=%02u dlc=%u b=",
                      (unsigned long)msg.id, cls, idx, dev, msg.len);
        for (int i = 0; i < msg.len; i++) Serial.printf("%02X ", msg.buf[i]);
        if (msg.len >= 4) {
            Serial.printf("f0=%.6f ", toFloat(msg.buf));
        }
        if (msg.len >= 8) {
            Serial.printf("f1=%.6f", toFloat(msg.buf + 4));
        }
        Serial.println();
    }
}

static void printSummary() {
    Serial.printf("-- SUMMARY tx=%lu rx=%lu unique=%d --\n",
                  (unsigned long)tx_count, (unsigned long)rx_count, n_seen);
    for (int i = 0; i < n_seen; i++) {
        const Seen &s = seen[i];
        Serial.printf("   id=0x%08lX cls=%02u idx=%02u dev=%02u n=%lu f0=%.4f f1=%.4f\n",
                      (unsigned long)s.id, s.cls, s.idx, s.dev,
                      (unsigned long)s.count, s.last_f0, s.last_f1);
    }
    Serial.println("-- end --");
}

static void handleLine(char *line) {
    if (!line[0]) return;
    char cmd = toupper((unsigned char)line[0]);
    switch (cmd) {
        case 'D': printSummary(); return;
        case 'R':
            n_seen = 0; rx_count = 0; tx_count = 0;
            Serial.println("OK reset");
            return;
        case 'Q': verbose = false; Serial.println("OK quiet"); return;
        case 'V': verbose = true;  Serial.println("OK verbose"); return;
        case 'T': {
            // TX <id_hex> <b0> ... <bn>
            uint32_t id = 0;
            uint8_t  buf[8] = {0};
            int      len = 0;
            char    *p = line + 1;
            while (*p == ' ') p++;
            if (*p == 'X' || *p == 'x') p++;
            while (*p == ' ') p++;
            id = strtoul(p, &p, 0);
            while (*p == ' ' && len < 8) {
                while (*p == ' ') p++;
                if (!*p) break;
                buf[len++] = (uint8_t)strtoul(p, &p, 0);
            }
            canSend(id, buf, len);
            Serial.printf("OK TX id=0x%08lX len=%d\n", (unsigned long)id, len);
            return;
        }
        default: Serial.println("ERR unknown"); return;
    }
}

static void processSerial() {
    static char buf[128];
    static uint8_t n = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (n > 0) { buf[n] = '\0'; handleLine(buf); n = 0; }
        } else if (n < sizeof(buf) - 1) {
            buf[n++] = c;
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println("# teensy_diag sniffer ready");
    Serial.println("# cmds: D=summary R=reset Q=quiet V=verbose TX <id_hex> <b0>..<b7>");
    Serial.println("# heartbeats @ 50 Hz; NO velocity setpoints sent");

    can.begin();
    can.setBaudRate(CAN_BAUD);
    can.setMaxMB(16);
    can.enableFIFO();
    can.enableFIFOInterrupt();
    can.setFIFOFilter(ACCEPT_ALL);
    can.onReceive(onCanRx);

    delay(100);
    sendHeartbeats();
    t_hb = t_sum = millis();
}

void loop() {
    can.events();
    processSerial();

    uint32_t now = millis();
    if (now - t_hb >= HB_DT_MS) {
        t_hb = now;
        sendHeartbeats();
    }
    if (now - t_sum >= SUMMARY_DT_MS) {
        t_sum = now;
        printSummary();
    }
}
