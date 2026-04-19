/*
 * MCP2515 SPI probe — raw readback diagnostic.
 *
 * Talks to the MCP2515 directly (no mcp_can dependency) so we can see
 * exactly what the chip puts on MISO. Use this when `init_can_controller`
 * keeps printing "Entering Configuration Mode Failure" — the readback
 * pattern below pinpoints whether MISO is floating, grounded, swapped
 * with MOSI, or the chip is dead.
 *
 * Pinout (matches suction_gripper.ino):
 *   D10 -> CS, D11 -> SI/MOSI, D12 -> SO/MISO, D13 -> SCK
 *
 * Expected output on a healthy 8 MHz module:
 *   CANSTAT after SPI reset      = 0x80
 *   CANSTAT after CANCTRL = 0x80 = 0x80
 *   loopback CNF1                = 0xA5
 *
 * Failure interpretation (see chat for full table):
 *   0xFF  MISO floating  -> wiring/swap
 *   0x00  MISO grounded  -> short or dead chip
 *   varies between runs  -> VCC brownout / loose wire
 */

#include <SPI.h>

static const int PIN_CS = 10;

static const uint8_t CMD_RESET = 0xC0;
static const uint8_t CMD_READ  = 0x03;
static const uint8_t CMD_WRITE = 0x02;

static const uint8_t REG_CANSTAT = 0x0E;
static const uint8_t REG_CANCTRL = 0x0F;
static const uint8_t REG_CNF1    = 0x2A;

static inline void csLow()  { digitalWrite(PIN_CS, LOW); }
static inline void csHigh() { digitalWrite(PIN_CS, HIGH); }

void spiBegin() {
    pinMode(PIN_CS, OUTPUT);
    csHigh();
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000UL, MSBFIRST, SPI_MODE0));
}

void mcpReset() {
    csLow();
    SPI.transfer(CMD_RESET);
    csHigh();
    delay(10);
}

uint8_t mcpRead(uint8_t reg) {
    csLow();
    SPI.transfer(CMD_READ);
    SPI.transfer(reg);
    uint8_t v = SPI.transfer(0x00);
    csHigh();
    return v;
}

void mcpWrite(uint8_t reg, uint8_t val) {
    csLow();
    SPI.transfer(CMD_WRITE);
    SPI.transfer(reg);
    SPI.transfer(val);
    csHigh();
}

void printHex(const __FlashStringHelper* label, uint8_t v) {
    Serial.print(label);
    Serial.print(F(" = 0x"));
    if (v < 0x10) Serial.print('0');
    Serial.println(v, HEX);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    Serial.println(F("\n--- MCP2515 SPI probe ---"));

    spiBegin();
    delay(50);
    mcpReset();

    uint8_t s1 = mcpRead(REG_CANSTAT);
    printHex(F("CANSTAT after SPI reset     "), s1);

    mcpWrite(REG_CANCTRL, 0x80);
    delay(2);
    uint8_t s2 = mcpRead(REG_CANSTAT);
    printHex(F("CANSTAT after CANCTRL=0x80  "), s2);

    // Round-trip a known pattern through CNF1 (only writable in config mode).
    mcpWrite(REG_CNF1, 0xA5);
    uint8_t r1 = mcpRead(REG_CNF1);
    printHex(F("CNF1 readback (wrote 0xA5)  "), r1);

    mcpWrite(REG_CNF1, 0x5A);
    uint8_t r2 = mcpRead(REG_CNF1);
    printHex(F("CNF1 readback (wrote 0x5A)  "), r2);

    Serial.println();
    if (s1 == 0x80 && s2 == 0x80 && r1 == 0xA5 && r2 == 0x5A) {
        Serial.println(F("RESULT: chip looks healthy, SPI fine."));
        Serial.println(F("        If suction firmware still fails init,"));
        Serial.println(F("        the issue is in the mcp_can library/clock path."));
    } else if (s1 == 0xFF || s2 == 0xFF) {
        Serial.println(F("RESULT: MISO reads 0xFF -> line is floating."));
        Serial.println(F("        Try swapping D11 <-> D12 (silkscreen swap)."));
    } else if (s1 == 0x00 && s2 == 0x00) {
        Serial.println(F("RESULT: MISO reads 0x00 -> grounded or chip dead."));
        Serial.println(F("        Check for solder bridge near SO; else swap module."));
    } else {
        Serial.println(F("RESULT: unexpected readback."));
        Serial.println(F("        Suspect VCC brownout or RST held low."));
    }
}

void loop() {}
