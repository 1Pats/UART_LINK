# Release Notes — UART_LINK

## v1.0.0 (2026-10-29)

- First release.
- Template class `CUartLink<TData>` for periodic, bidirectional struct exchange between two ESP32 boards.
- Background FreeRTOS task on core 0 with mutex-protected TX/RX data.
- Frame format: 2 sync bytes, sequence counter, payload size, payload, CRC16 (CCITT-FALSE).
- Link-alive detection (`bIsLinkAlive`) and rejected-frame counter (`u32GetCrcErrors`).
- Example: `UART_LINK_Demo`.
