# Release Notes — UART_LINK

## v1.1.0 (2026-10-30)

- Arduino UNO / Nano / Mega (AVR) support: polled `CUartLink` with `bBegin(Stream&, periodMs)` and `vUpdate()`; works with `Serial`, `SoftwareSerial`, `AltSoftSerial`.
- Same wire format on all boards, so an ESP32 and a UNO can talk to each other.
- Frame building, parsing and CRC moved into a shared `CUartLinkCore`; ESP32 API unchanged.
- Example `UART_LINK_Uno`; `UART_LINK_Demo` struct is now packed (14 bytes on both boards).

## v1.0.0 (2026-10-29)

- First release.
- Template class `CUartLink<TData>` for periodic, bidirectional struct exchange between two ESP32 boards.
- Background FreeRTOS task on core 0 with mutex-protected TX/RX data.
- Frame format: 2 sync bytes, sequence counter, payload size, payload, CRC16 (CCITT-FALSE).
- Link-alive detection (`bIsLinkAlive`) and rejected-frame counter (`u32GetCrcErrors`).
- Example: `UART_LINK_Demo`.
