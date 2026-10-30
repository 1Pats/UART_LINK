#include <UART_LINK.h>

// 1. Define your custom data structure (must be trivially copyable)
//    Fixed-width types + packed = identical 14-byte layout on ESP32 and AVR (UNO)
struct __attribute__((packed)) MyData {
    float fTelemetryX;
    float fTelemetryY;
    uint32_t u32StatusFlags;
    int16_t i16MotorSpeed;
};
static_assert(sizeof(MyData) == 14, "MyData must be 14 bytes on both boards");

// 2. Instantiate the link with your specific data structure type
CUartLink<MyData> Link;

void setup() {
    Serial.begin(115200); // Serial monitor for debug output
    
    // Initialize the link: Serial2, 115200 baud, RX=16, TX=17, 20ms period
    // (talking to an Arduino UNO? use Link.bBegin(Serial2, 38400, 16, 17, 100) to match UART_LINK_Uno)
    if (Link.bBegin(Serial2, 115200, 16, 17, 20)) {
        Serial.println("UART Link background task spawned successfully!");
    } else {
        Serial.println("Failed to allocate synchronization structures!");
        while (1) delay(1000);
    }
}

void loop() {
    // --- Sending Section ---
    static uint32_t u32LastTxTime = 0;
    if (millis() - u32LastTxTime >= 100) { // Update outgoing buffer values every 100ms
        u32LastTxTime = millis();
        
        MyData sTxPackage;
        sTxPackage.fTelemetryX = 12.34f;
        sTxPackage.fTelemetryY = 56.78f;
        sTxPackage.u32StatusFlags = 0xABCDEF01;
        sTxPackage.i16MotorSpeed = -450;
        
        Link.vSetTxData(sTxPackage); // Safe push to background task loop
    }

    // --- Receiving Section ---
    MyData sRxPackage;
    if (Link.bGetRxData(sRxPackage)) { // True if fresh data arrived since last call
        Serial.println("--- New Frame Arrived! ---");
        Serial.printf("X: %.2f, Y: %.2f\n", sRxPackage.fTelemetryX, sRxPackage.fTelemetryY);
        Serial.printf("Flags: 0x%lX\n", (unsigned long)sRxPackage.u32StatusFlags);
        Serial.printf("Speed: %d\n", sRxPackage.i16MotorSpeed);
    }

    // --- Connection Monitoring Diagnostic ---
    static uint32_t u32DiagTime = 0;
    if (millis() - u32DiagTime >= 1000) {
        u32DiagTime = millis();
        if (!Link.bIsLinkAlive(500)) {
            Serial.printf("Link is down! CRC Error count: %lu\n", (unsigned long)Link.u32GetCrcErrors());
        }
    }
}