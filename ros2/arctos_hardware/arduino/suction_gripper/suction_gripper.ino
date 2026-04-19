/*
 * Arctos Suction Gripper — CAN Node (ID 7)
 *
 * Hardware:
 *   Arduino Nano + MCP2515 CAN module
 *   Pump ESC on D3   (servo signal, 500µs=off / 2500µs=on)
 *   Valve ESC on D5  (servo signal, 500µs=closed / 2500µs=open)
 *
 * CAN protocol: MKS-compatible framing
 *   [opcode, data..., CRC]  where CRC = (CAN_ID + sum(all bytes before CRC)) & 0xFF
 */

#include <SPI.h>
#include <mcp_can.h>
#include <Servo.h>

// --- Pin assignments ---
static const int PIN_CAN_CS   = 10;
static const int PIN_CAN_INT  = 2;
static const int PIN_PUMP     = 3;
static const int PIN_VALVE    = 5;

// --- Servo signal bounds (microseconds) ---
static const int SERVO_OFF = 500;
static const int SERVO_ON  = 2500;

// --- CAN config ---
static const unsigned long CAN_ID       = 0x07;
static const unsigned long CAN_BAUDRATE = CAN_500KBPS;

// --- Command opcodes (PC -> Arduino) ---
static const uint8_t CMD_SET_SUCTION   = 0x01;
static const uint8_t CMD_SET_PUMP_PWM  = 0x02;
static const uint8_t CMD_SET_VALVE     = 0x03;
static const uint8_t CMD_QUERY_STATUS  = 0xE0;
static const uint8_t CMD_SET_WATCHDOG  = 0xE1;

// --- Fault bits ---
static const uint8_t FAULT_WATCHDOG = 0x01;

// --- State ---
MCP_CAN can(PIN_CAN_CS);
Servo pumpServo;
Servo valveServo;

uint8_t  pump_pwm       = 0;   // 0 = off, >0 = on
uint8_t  valve_state     = 0;
uint8_t  fault_bits      = 0;
uint16_t watchdog_ms     = 1000;
unsigned long last_cmd_time  = 0;
unsigned long boot_time      = 0;
bool     watchdog_tripped    = false;

// --- Helpers ---

uint8_t calc_crc(unsigned long id, const uint8_t* data, uint8_t len) {
    uint16_t sum = (uint8_t)(id & 0xFF);
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

void send_response(const uint8_t* payload, uint8_t len) {
    uint8_t buf[8];
    if (len > 7) len = 7;
    memcpy(buf, payload, len);
    buf[len] = calc_crc(CAN_ID, payload, len);
    can.sendMsgBuf(CAN_ID, 0, len + 1, buf);
}

void apply_outputs() {
    pumpServo.writeMicroseconds(pump_pwm > 0 ? SERVO_ON : SERVO_OFF);
    valveServo.writeMicroseconds(valve_state ? SERVO_ON : SERVO_OFF);
}

void force_off() {
    pump_pwm = 0;
    valve_state = 0;
    apply_outputs();
}

void send_status() {
    unsigned long uptime_s = (millis() - boot_time) / 1000;
    uint8_t payload[] = {
        CMD_QUERY_STATUS,
        pump_pwm,
        valve_state,
        fault_bits,
        (uint8_t)((uptime_s >> 8) & 0xFF),
        (uint8_t)(uptime_s & 0xFF),
    };
    send_response(payload, sizeof(payload));
}

bool init_can_controller() {
    const uint8_t clocks[] = {MCP_8MHZ, MCP_16MHZ};
    const char* labels[] = {"8MHz", "16MHz"};

    for (uint8_t i = 0; i < 2; ++i) {
        if (can.begin(MCP_ANY, CAN_BAUDRATE, clocks[i]) == CAN_OK) {
            Serial.print(F("MCP2515 ready, CAN ID=7, 500kbps, clock="));
            Serial.println(labels[i]);
            return true;
        }
    }

    return false;
}

// --- CAN message handler ---

void handle_can_message(unsigned long rx_id, uint8_t len, uint8_t* buf) {
    if (rx_id != CAN_ID || len < 2) return;

    // Validate CRC (last byte)
    uint8_t rx_crc = buf[len - 1];
    uint8_t expected_crc = calc_crc(rx_id, buf, len - 1);
    if (rx_crc != expected_crc) return;

    uint8_t opcode = buf[0];
    last_cmd_time = millis();

    // Clear watchdog fault on any valid command
    if (watchdog_tripped) {
        watchdog_tripped = false;
        fault_bits &= ~FAULT_WATCHDOG;
    }

    switch (opcode) {
        case CMD_SET_SUCTION:
            if (len >= 4) {  // opcode + pump + valve + crc
                pump_pwm = buf[1];
                valve_state = buf[2] ? 1 : 0;
                apply_outputs();
                uint8_t ack[] = {CMD_SET_SUCTION, 0x00};
                send_response(ack, 2);
            }
            break;

        case CMD_SET_PUMP_PWM:
            if (len >= 3) {
                pump_pwm = buf[1];
                apply_outputs();
                uint8_t ack[] = {CMD_SET_PUMP_PWM, 0x00};
                send_response(ack, 2);
            }
            break;

        case CMD_SET_VALVE:
            if (len >= 3) {
                valve_state = buf[1] ? 1 : 0;
                apply_outputs();
                uint8_t ack[] = {CMD_SET_VALVE, 0x00};
                send_response(ack, 2);
            }
            break;

        case CMD_QUERY_STATUS:
            send_status();
            break;

        case CMD_SET_WATCHDOG:
            if (len >= 3) {
                watchdog_ms = (uint16_t)buf[1] * 100;
                uint8_t ack[] = {CMD_SET_WATCHDOG, buf[1]};
                send_response(ack, 2);
            }
            break;

        default:
            break;
    }
}

// --- Setup ---

void setup() {
    pumpServo.attach(PIN_PUMP);
    valveServo.attach(PIN_VALVE);
    force_off();

    pinMode(PIN_CAN_INT, INPUT);

    Serial.begin(115200);
    Serial.println(F("Arctos Suction Gripper CAN Node"));

    while (!init_can_controller()) {
        force_off();
        Serial.println(F("MCP2515 init failed on 8MHz and 16MHz, retrying..."));
        delay(500);
    }
    can.setMode(MCP_NORMAL);

    boot_time = millis();
    last_cmd_time = millis();
}

// --- Main loop ---

void loop() {
    // Check for incoming CAN messages
    if (!digitalRead(PIN_CAN_INT)) {
        unsigned long rx_id;
        uint8_t len;
        uint8_t buf[8];
        if (can.readMsgBuf(&rx_id, &len, buf) == CAN_OK) {
            // Mask off extended/RTR bits
            rx_id &= 0x7FF;
            handle_can_message(rx_id, len, buf);
        }
    }

    // Watchdog: if no command for watchdog_ms, shut off pump/valve
    if (watchdog_ms > 0 && !watchdog_tripped) {
        if (millis() - last_cmd_time > watchdog_ms) {
            if (pump_pwm > 0 || valve_state) {
                force_off();
                fault_bits |= FAULT_WATCHDOG;
                watchdog_tripped = true;

                // Send unsolicited fault event
                uint8_t fault_msg[] = {0xFF, fault_bits};
                send_response(fault_msg, 2);
                Serial.println(F("WATCHDOG: suction forced off"));
            }
        }
    }
}
