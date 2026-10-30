# UART_LINK v1.1.0

Lightweight, non-blocking data synchronization between two boards over a UART: ESP32 ↔ ESP32, ESP32 ↔ Arduino UNO/Nano/Mega, or UNO ↔ UNO.

## ✨ Features

- ✅ Sends any user-defined struct (template parameter) in both directions
- ✅ ESP32: fully non-blocking, a background FreeRTOS task on core 0 does all TX/RX work
- ✅ AVR (UNO, Nano, Mega): non-blocking `vUpdate()` called from `loop()`, works with `Serial`, `SoftwareSerial`, `AltSoftSerial`
- ✅ Same wire format on every board
- ✅ Thread-safe access to TX/RX data on ESP32 (mutex protected)
- ✅ Framed protocol with sync bytes, sequence counter, payload size and CRC16
- ✅ Link-alive detection and error counter
- ✅ Header-only, no external dependencies

## 📦 Requirements

- ESP32 (ESP32, ESP32-S3) with arduino-esp32 2.x or newer, and/or
- AVR board (UNO, Nano, Mega) with Arduino AVR Boards 1.8.x
- Arduino IDE

## 🔌 Wiring

| Board A | Board B |
|---------|---------|
| TX pin  | RX pin  |
| RX pin  | TX pin  |
| GND     | GND     |

Always connect GND. ESP32 ↔ ESP32 and UNO ↔ UNO connect directly.

### ESP32 ↔ UNO: level shifting

The UNO uses **5 V** logic and the ESP32 **3.3 V**; ESP32 pins are **not 5 V tolerant**.

**Option A (recommended): bidirectional logic level shifter module** (4-channel, BSS138 type)

```
        UNO                 Level shifter                ESP32
                        HV side      LV side
     5V ───────────────── HV          LV ─────────────── 3V3
    GND ───────────────── GND        GND ─────────────── GND
pin 11 (TX) ───────────── HV1        LV1 ─────────────── GPIO16 (RX)
pin 10 (RX) ───────────── HV2        LV2 ─────────────── GPIO17 (TX)
```

**Option B: resistor divider** (only the UNO → ESP32 direction needs shifting)

| From | To | How |
|------|----|-----|
| UNO TX (5 V) | ESP32 RX | 1 kΩ in series, 2 kΩ to GND (5 V → 3.3 V) |
| ESP32 TX (3.3 V) | UNO RX | Direct (3.3 V is above the UNO's 3.0 V threshold) |
| GND | GND | Always |

### UNO serial ports and baud rates

The UNO has one hardware UART (pins 0/1), shared with USB.

| Port | Notes | Recommended baud |
|------|-------|------------------|
| `Serial` (pins 0/1) | Most reliable; no serial monitor, unplug while uploading | 57600 |
| `SoftwareSerial` (any pins) | Keeps the monitor free | 38400 or lower |
| `AltSoftSerial` (pins 8/9) | More reliable than SoftwareSerial | 38400–57600 |

Both boards must use the same baud rate. 115200 is +2.1 % off on a 16 MHz UNO.

## 🚀 Basic Usage (ESP32)

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

## 🚀 Basic Usage (UNO / AVR)

```cpp
#include <SoftwareSerial.h>
#include <UART_LINK.h>

struct __attribute__((packed)) MyData {   // Same definition on both boards!
    float    fValue;
    uint32_t u32Flags;
};

SoftwareSerial LinkSerial(10, 11);        // RX, TX
CUartLink<MyData> Link;

void setup() {
    LinkSerial.begin(38400);              // Open the port yourself on AVR
    Link.bBegin(LinkSerial, 100);         // Stream, TX period ms
}

void loop() {
    Link.vUpdate();                       // Call on every pass, avoid long delay()
    // vSetTxData() / bGetRxData() exactly as on ESP32
}
```

Examples:
- `UART_LINK_Demo` — ESP32 (pairs with another ESP32, or with a UNO at 38400 baud / 100 ms)
- `UART_LINK_Uno` — Arduino UNO with SoftwareSerial, pairs with `UART_LINK_Demo`

## 📚 API

| Method | Description |
|--------|-------------|
| `bool bBegin(HardwareSerial&, uint32_t baud, int8_t rxPin, int8_t txPin, uint32_t txPeriodMs = 20)` | **ESP32:** opens the UART and starts the background task. Returns `false` if the mutex or task could not be created. |
| `bool bBegin(Stream&, uint32_t txPeriodMs = 50)` | **AVR:** attaches to an already opened port. |
| `void vUpdate()` | **AVR:** receives and sends; call on every pass of `loop()`. |
| `void vEnd()` | **ESP32:** stops the task, closes the UART and frees resources. **AVR:** detaches from the port. |
| `void vSetTxData(const TData&)` | Stores the data that the task sends every TX period. |
| `bool bGetRxData(TData&)` | Copies the latest received data. Returns `true` only if it is new since the last call. |
| `bool bIsLinkAlive(uint32_t timeoutMs = 500)` | `true` if a valid frame arrived within the timeout. |
| `uint32_t u32GetCrcErrors()` | Number of rejected frames (bad CRC or payload size mismatch). |

## 🏗 Architecture (ESP32)

On AVR there is no task: `vUpdate()` does the same parse/send work inside `loop()`.

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
- **ESP32 ↔ AVR:** `int` is 2 bytes on AVR and 4 on ESP32, `double` is 4 vs 8 bytes, and the ESP32 pads structs. Use only fixed-width types (`int16_t`, `uint32_t`, `float`) and declare the struct `__attribute__((packed))` on both boards. Add a `static_assert(sizeof(MyData) == N)` to catch mistakes.
- Check the bandwidth: at 115200 baud about 11 bytes/ms fit on the wire (38400: about 4 bytes/ms). A frame of `sizeof(TData) + 8` bytes must be sent well within the TX period.
- The maximum frame size is 1024 bytes on ESP32 (UART buffer size) and 256 bytes on AVR (RAM).
- On AVR the RX buffer holds only 64 bytes: call `vUpdate()` often and avoid long `delay()` calls.

## 🎯 Design Principles

- KISS (Keep It Simple and Small)
- Minimal memory footprint
- Payload type fixed at compile time (template)

## 📄 License

MIT License, see `LICENSE`.
