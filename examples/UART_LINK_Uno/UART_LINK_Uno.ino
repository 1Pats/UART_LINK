// UART_LINK on an Arduino UNO (AVR): polled version, no RTOS.
// Pairs with UART_LINK_Demo on an ESP32 using Link.bBegin(Serial2, 38400, 16, 17, 100).
//
// Wiring: the UNO is 5 V, the ESP32 is 3.3 V and NOT 5 V tolerant -> levels must be shifted.
//
// Option A (recommended): 4-channel bidirectional logic level shifter module (BSS138 type)
//
//            UNO                 Level shifter                ESP32
//                            HV side      LV side
//         5V ───────────────── HV          LV ─────────────── 3V3
//        GND ───────────────── GND        GND ─────────────── GND
//   pin 11 (TX) ────────────── HV1        LV1 ─────────────── GPIO16 (RX)
//   pin 10 (RX) ────────────── HV2        LV2 ─────────────── GPIO17 (TX)
//
//   HV = high-voltage side (5 V), LV = low-voltage side (3.3 V). Both GND pins and
//   both boards' GND must be connected together.
//
// Option B: resistor divider (only the UNO -> ESP32 direction needs shifting)
//
//   UNO pin 11 (TX) ──[1k]──┬────── ESP32 GPIO16 (RX)     5 V * 2k / (1k + 2k) = 3.3 V
//                           │
//                         [2k]
//                           │
//                          GND
//   UNO pin 10 (RX) ─────────────── ESP32 GPIO17 (TX)     3.3 V is above the UNO's 3.0 V "high" level
//   UNO GND ─────────────────────── ESP32 GND

#include <SoftwareSerial.h>
#include <UART_LINK.h>

// 1. Same data structure as on the ESP32 (fixed-width types, packed)
struct __attribute__((packed)) MyData {
    float fTelemetryX;
    float fTelemetryY;
    uint32_t u32StatusFlags;
    int16_t i16MotorSpeed;
};
static_assert(sizeof(MyData) == 14, "MyData must be 14 bytes on both boards");

// 2. SoftwareSerial keeps the USB Serial free for the monitor (reliable up to 38400 baud)
SoftwareSerial LinkSerial(10, 11);     // RX, TX
CUartLink<MyData> Link;

void setup() {
    Serial.begin(115200);              // Serial monitor for debug output
    LinkSerial.begin(38400);           // Open the link port yourself on AVR
    Link.bBegin(LinkSerial, 100);      // Send a frame every 100 ms
    Serial.println(F("UART Link (UNO) started"));
}

void loop() {
    Link.vUpdate();                    // Receive and send: call on every pass, avoid long delay()

    // --- Sending Section ---
    static uint32_t u32LastTxTime = 0;
    if (millis() - u32LastTxTime >= 100) {
        u32LastTxTime = millis();

        MyData sTxPackage;
        sTxPackage.fTelemetryX = 1.5f;
        sTxPackage.fTelemetryY = -2.25f;
        sTxPackage.u32StatusFlags = 0x12345678UL;
        sTxPackage.i16MotorSpeed = 300;

        Link.vSetTxData(sTxPackage);
    }

    // --- Receiving Section ---
    MyData sRxPackage;
    if (Link.bGetRxData(sRxPackage)) {
        Serial.print(F("X: "));      Serial.print(sRxPackage.fTelemetryX);
        Serial.print(F(", Y: "));    Serial.print(sRxPackage.fTelemetryY);
        Serial.print(F(", Flags: 0x")); Serial.print(sRxPackage.u32StatusFlags, HEX);
        Serial.print(F(", Speed: ")); Serial.println(sRxPackage.i16MotorSpeed);
    }

    // --- Connection Monitoring Diagnostic ---
    static uint32_t u32DiagTime = 0;
    if (millis() - u32DiagTime >= 1000) {
        u32DiagTime = millis();
        if (!Link.bIsLinkAlive(500)) {
            Serial.print(F("Link is down! CRC Error count: "));
            Serial.println(Link.u32GetCrcErrors());
        }
    }
}
