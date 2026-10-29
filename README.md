# UART_LINK v1.0.0

Lightweight, non-blocking data synchronization between two dual-core ESP32 boards over a hardware UART.

## ✨ Features

- ✅ Sends any user-defined struct (template parameter) in both directions
- ✅ Fully non-blocking: a background FreeRTOS task on core 0 does all TX/RX work
- ✅ Thread-safe access to TX/RX data (mutex protected)
- ✅ Framed protocol with sync bytes, sequence counter, payload size and CRC16
- ✅ Link-alive detection and error counter
- ✅ Header-only, no external dependencies

## 📦 Requirements

- Dual-core ESP32 (ESP32, ESP32-S3)
- Arduino IDE with the ESP32 core (arduino-esp32 2.x or newer)

## 🔌 Wiring

| Board A | Board B |
|---------|---------|
| TX pin  | RX pin  |
| RX pin  | TX pin  |
| GND     | GND     |

Both boards must use 3.3 V logic (ESP32 to ESP32 is fine). Always connect GND.

## 🚀 Basic Usage

```cpp
#include <UART_LINK.h>

struct MyData {                // Same definition on both boards!
    float    fValue;
    uint32_t u32Flags;
};

CUartLink<MyData> Link;

void setup() {
    Link.bBegin(Serial2, 115200, 16, 17, 20);   // UART, baud, RX pin, TX pin, TX period ms
}

void loop() {
    MyData sTx = { 1.23f, 0x01 };
    Link.vSetTxData(sTx);                      // Sent automatically every TX period

    MyData sRx;
    if (Link.bGetRxData(sRx)) {                // true only for new data
        // use sRx
    }
}
```

See `examples/UART_LINK_Demo` for a complete sketch.

## 📚 API

| Method | Description |
|--------|-------------|
| `bool bBegin(HardwareSerial&, uint32_t baud, int8_t rxPin, int8_t txPin, uint32_t txPeriodMs = 20)` | Opens the UART and starts the background task. Returns `false` if the mutex or task could not be created. |
| `void vEnd()` | Stops the task, closes the UART and frees resources. |
| `void vSetTxData(const TData&)` | Stores the data that the task sends every TX period. |
| `bool bGetRxData(TData&)` | Copies the latest received data. Returns `true` only if it is new since the last call. |
| `bool bIsLinkAlive(uint32_t timeoutMs = 500)` | `true` if a valid frame arrived within the timeout. |
| `uint32_t u32GetCrcErrors()` | Number of rejected frames (bad CRC or payload size mismatch). |

## 🏗 Architecture

```
 loop() (core 1)                    UartLink task (core 0)
 ───────────────                    ──────────────────────
 vSetTxData() ──► [mutex] m_sTxData ──► every TX period: build frame ──► UART TX
 bGetRxData() ◄── [mutex] m_sRxData ◄── parse bytes, check CRC  ◄──────── UART RX
```

### Frame format (little-endian, packed)

| Field   | Size           | Content                         |
|---------|----------------|---------------------------------|
| Sync1   | 1              | `0xAA`                          |
| Sync2   | 1              | `0xBB`                          |
| Seq     | 2              | Sequence counter                |
| Size    | 2              | `sizeof(TData)`                 |
| Payload | `sizeof(TData)`| User struct, raw bytes          |
| CRC     | 2              | CRC-16/CCITT-FALSE over all preceding bytes |

Frame overhead is 8 bytes.

## ⚠️ Notes

- `TData` must be trivially copyable: plain numbers, arrays and nested plain structs only. No pointers, `String` or `std::` containers. This is checked at compile time.
- Both boards must use the identical `TData` definition. A size mismatch is detected and counted as an error.
- Check the bandwidth: at 115200 baud about 11 bytes/ms fit on the wire. A frame of `sizeof(TData) + 8` bytes must be sent well within the TX period.
- The maximum frame size is 1024 bytes (UART buffer size).

## 🎯 Design Principles

- KISS (Keep It Simple and Small)
- Minimal memory footprint
- Payload type fixed at compile time (template)

## 📄 License

MIT License, see `LICENSE`.
