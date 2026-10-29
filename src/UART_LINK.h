#ifndef UART_LINK_H                                                     // Include guard start
#define UART_LINK_H                                                     // Include guard define

#include <Arduino.h>                                                    // Arduino core (millis, HardwareSerial, types)
#include <type_traits>                                                  // std::is_trivially_copyable
#include <freertos/FreeRTOS.h>                                          // FreeRTOS base
#include <freertos/task.h>                                              // FreeRTOS tasks
#include <freertos/semphr.h>                                            // FreeRTOS mutex

// Generic two-board UART link: TData is any trivially copyable struct (no pointers, no String)
// Both boards must use the identical TData definition (same layout, same byte order)
template <typename TData>                                               // Template parameter = user data structure
class CUartLink  {                                                      // Class name (C prefix = class)
    static_assert(std::is_trivially_copyable<TData>::value,             // Payload is sent as raw bytes ...
                  "CUartLink: TData must be trivially copyable");       // ... so it must be a plain struct

public:                                                                 // Public interface
    static constexpr uint8_t  SYNC1          = 0xAA;                    // Magic sync byte 1
    static constexpr uint8_t  SYNC2          = 0xBB;                    // Magic sync byte 2
    static constexpr size_t   UART_BUF_SIZE  = 1024;                    // UART RX/TX ring buffer size
    static constexpr uint32_t STOP_TIMEOUT_MS = 1000;                   // Max wait for the task to exit in vEnd()

    CUartLink()                                                         // Constructor
        : m_pSerial(nullptr),                                           // No serial port assigned yet
          m_xMutex(nullptr),                                            // No mutex yet
          m_xTaskHandle(nullptr),                                       // No task yet
          m_sTxData{},                                                  // Zero-init TX data
          m_sRxData{},                                                  // Zero-init RX data
          m_bNewRx(false),                                              // No new RX data yet
          m_bEverRx(false),                                             // Nothing received yet
          m_u32LastRxMs(0),                                             // Time of last valid frame
          m_u32TxPeriodMs(20),                                          // Default TX period 20 ms
          m_u32CrcErrors(0),                                            // No rejected frames yet
          m_u16TxSeq(0),                                                // Start TX sequence at 0
          m_uRxIdx(0),                                                  // Parser waits for first sync byte
          m_bRunning(false),                                            // Link is not running
          m_bTaskActive(false)                                          // Task is not alive
    {                                                                   // Constructor body start
        memset(m_au8RxBuf, 0, sizeof(m_au8RxBuf));                      // Clear the frame assembly buffer
    }                                                                   // Constructor body end

    CUartLink(const CUartLink&) = delete;                               // Not copyable (task holds 'this')
    CUartLink& operator=(const CUartLink&) = delete;                    // Not assignable

    ~CUartLink()                                                        // Destructor
    {                                                                   // Destructor body start
        vEnd();                                                         // Stop task and UART
    }                                                                   // Destructor body end

    // Start the link on a given UART (Serial1 / Serial2), baud rate, RX pin, TX pin and TX period
    bool bBegin(HardwareSerial& rSerial, uint32_t u32Baud, int8_t i8RxPin, int8_t i8TxPin, uint32_t u32TxPeriodMs = 20) {
        if (m_bRunning) return true;                                    // Already running -> nothing to do
        m_pSerial = &rSerial;                                           // Remember the serial port
        m_u32TxPeriodMs = (u32TxPeriodMs == 0) ? 1 : u32TxPeriodMs;     // Store TX period (at least 1 ms)
        m_uRxIdx = 0;                                                   // Reset frame parser

        m_xMutex = xSemaphoreCreateMutex();                             // Create mutex protecting the shared data
        if (m_xMutex == nullptr) return false;                          // Abort if no memory

        m_pSerial->setRxBufferSize(UART_BUF_SIZE);                      // Enlarge RX ring buffer (before begin)
        m_pSerial->setTxBufferSize(UART_BUF_SIZE);                      // Enlarge TX ring buffer (before begin)
        m_pSerial->begin(u32Baud, SERIAL_8N1, i8RxPin, i8TxPin);        // Open UART: 8 data bits, no parity, 1 stop bit

        m_bRunning = true;                                              // Mark link as running before task start
        m_bTaskActive = true;                                           // Task counts as alive from now on
        BaseType_t xRes = xTaskCreatePinnedToCore(                      // Create the background sync task ...
            &CUartLink::vTaskEntry,                                     // ... entry function
            "UartLink",                                                 // ... task name
            4096,                                                       // ... stack size in bytes
            this,                                                       // ... argument = this object
            1,                                                          // ... priority
            &m_xTaskHandle,                                             // ... handle output
            0);                                                         // ... pinned to core 0
        if (xRes != pdPASS) {                                           // Task creation failed ...
            m_bRunning = false;                                         // ... undo running state
            m_bTaskActive = false;                                      // ... no task exists
            m_xTaskHandle = nullptr;                                    // ... no handle
            m_pSerial->end();                                           // ... close the UART
            vSemaphoreDelete(m_xMutex);                                 // ... free mutex
            m_xMutex = nullptr;                                         // ... reset mutex handle
            return false;                                               // ... report failure
        }
        return true;                                                    // Success
    }                                                                   // bBegin body end

    void vEnd() {                                                       // Stop the link
        if (!m_bRunning) return;                                        // Nothing to stop
        m_bRunning = false;                                             // Ask the task to leave its loop
        uint32_t u32Start = millis();                                   // Start of the wait
        while (m_bTaskActive && (millis() - u32Start) < STOP_TIMEOUT_MS) {
            vTaskDelay(pdMS_TO_TICKS(1));                               // Wait until the task confirms its exit
        }
        if (m_bTaskActive && m_xTaskHandle != nullptr) {                // Task did not exit in time ...
            vTaskDelete(m_xTaskHandle);                                 // ... remove it forcibly
            m_bTaskActive = false;                                      // ... task is gone now
        }
        m_xTaskHandle = nullptr;                                        // Reset task handle
        m_pSerial->end();                                               // Close the UART
        if (m_xMutex != nullptr) vSemaphoreDelete(m_xMutex);            // Free mutex
        m_xMutex = nullptr;                                             // Reset mutex handle
    }                                                                   // vEnd body end

    // Store the newest data to send; the background task transmits it periodically
    void vSetTxData(const TData& rsData) {
        if (m_xMutex == nullptr) return;                                // Not started -> ignore
        xSemaphoreTake(m_xMutex, portMAX_DELAY);                        // Lock shared data
        m_sTxData = rsData;                                             // Copy user data into TX buffer
        xSemaphoreGive(m_xMutex);                                       // Unlock shared data
    }                                                                   // vSetTxData body end

    // Copy the latest received data; returns true only if it is new since the last call
    bool bGetRxData(TData& rsData) {
        if (m_xMutex == nullptr) return false;                          // Not started -> nothing received
        xSemaphoreTake(m_xMutex, portMAX_DELAY);                        // Lock shared data
        rsData = m_sRxData;                                             // Copy RX buffer to the caller
        bool bIsNew = m_bNewRx;                                         // Remember the "new" flag
        m_bNewRx = false;                                               // Clear the flag
        xSemaphoreGive(m_xMutex);                                       // Unlock shared data
        return bIsNew;                                                  // Tell the caller if data was fresh
    }                                                                   // bGetRxData body end

    // True if a valid frame arrived within the given timeout
    bool bIsLinkAlive(uint32_t u32TimeoutMs = 500) const {
        if (!m_bEverRx) return false;                                   // Never received anything
        return (millis() - m_u32LastRxMs) <= u32TimeoutMs;              // Compare age of last frame
    }                                                                   // bIsLinkAlive body end

    uint32_t u32GetCrcErrors() const {                                  // Number of frames rejected (bad CRC or size)
        return m_u32CrcErrors;                                          // Return the error counter
    }                                                                   // u32GetCrcErrors body end

private:                                                                // Internal data and helper methods
    // Structure defining the physical byte alignment over the UART wire
    #pragma pack(push, 1)                                               // Force compiler to packing with 1-byte alignment
    struct SFrame {                                                     // Wire protocol structure
        uint8_t  u8Sync1;                                               // Magic sync byte 1 (SYNC1)
        uint8_t  u8Sync2;                                               // Magic sync byte 2 (SYNC2)
        uint16_t u16Seq;                                                // Sequence counter to track packet loss
        uint16_t u16Size;                                               // Size of the payload (sizeof(TData))
        TData    sPayload;                                              // The actual user payload data
        uint16_t u16Crc;                                                // CRC16 checksum over all preceding bytes
    };                                                                  // Structure definition end
    #pragma pack(pop)                                                   // Restore default compiler packing rules

    static constexpr size_t FRAME_SIZE  = sizeof(SFrame);               // Bytes per frame on the wire
    static constexpr size_t HEADER_SIZE = offsetof(SFrame, sPayload);   // Sync1 + Sync2 + Seq + Size = 6 bytes
    static_assert(FRAME_SIZE <= UART_BUF_SIZE,                          // A frame must fit the UART buffers
                  "CUartLink: TData is too large for the UART buffers");

    HardwareSerial*   m_pSerial;                                        // Pointer to selected hardware serial port
    SemaphoreHandle_t m_xMutex;                                         // Thread safety lock handle
    TaskHandle_t      m_xTaskHandle;                                    // Worker thread task reference

    TData             m_sTxData;                                        // Internal copy of user data to send
    TData             m_sRxData;                                        // Internal copy of latest valid received data
    bool              m_bNewRx;                                         // Flag signaling unread fresh incoming data
    volatile bool     m_bEverRx;                                        // At least one valid frame was received
    volatile uint32_t m_u32LastRxMs;                                    // millis() of the last valid frame
    uint32_t          m_u32TxPeriodMs;                                  // Interval between transmitted frames
    volatile uint32_t m_u32CrcErrors;                                   // Counter of rejected frames
    uint16_t          m_u16TxSeq;                                       // Sequence number of the next TX frame
    size_t            m_uRxIdx;                                         // Write position in the assembly buffer
    volatile bool     m_bRunning;                                       // Task keeps looping while true
    volatile bool     m_bTaskActive;                                    // Task is alive (cleared by the task on exit)
    uint8_t           m_au8RxBuf[FRAME_SIZE];                           // Frame assembly buffer

    static void vTaskEntry(void* pvArg) {                               // FreeRTOS task entry (static trampoline)
        static_cast<CUartLink*>(pvArg)->vTaskLoop();                    // Run the member loop
        vTaskDelete(nullptr);                                           // Delete this task (never returns)
    }                                                                   // vTaskEntry body end

    void vTaskLoop() {                                                  // Background TX/RX loop
        uint32_t u32LastTxMs = millis();                                // Time of last transmitted frame
        while (m_bRunning) {                                            // Run until vEnd() is called
            while (m_pSerial->available() > 0) {                        // Drain all received bytes ...
                vParseByte((uint8_t)m_pSerial->read());                 // ... and feed them to the parser
            }
            if ((millis() - u32LastTxMs) >= m_u32TxPeriodMs) {          // Time to send the next frame?
                u32LastTxMs = millis();                                 // Remember send time
                vSendFrame();                                           // Transmit current TX data
            }
            vTaskDelay(pdMS_TO_TICKS(1));                               // Yield the CPU (feeds the watchdog)
        }
        m_bTaskActive = false;                                          // Tell vEnd() the loop has finished
    }                                                                   // vTaskLoop body end

    void vSendFrame() {                                                 // Build and transmit one frame
        SFrame sFrame;                                                  // Frame on the stack
        sFrame.u8Sync1 = SYNC1;                                         // Sync byte 1
        sFrame.u8Sync2 = SYNC2;                                         // Sync byte 2
        sFrame.u16Seq  = m_u16TxSeq++;                                  // Sequence number, then increment
        sFrame.u16Size = (uint16_t)sizeof(TData);                       // Payload size
        xSemaphoreTake(m_xMutex, portMAX_DELAY);                        // Lock shared data
        memcpy((uint8_t*)&sFrame + HEADER_SIZE, &m_sTxData, sizeof(TData)); // Copy the newest TX data (packed field -> memcpy)
        xSemaphoreGive(m_xMutex);                                       // Unlock shared data
        sFrame.u16Crc = u16Crc16((const uint8_t*)&sFrame, FRAME_SIZE - sizeof(uint16_t)); // CRC over everything but the CRC
        m_pSerial->write((const uint8_t*)&sFrame, FRAME_SIZE);          // Push frame into the UART TX buffer
    }                                                                   // vSendFrame body end

    void vParseByte(uint8_t u8Byte) {                                   // Byte-wise frame assembly state machine
        if (m_uRxIdx == 0) {                                            // Waiting for sync byte 1
            if (u8Byte == SYNC1) m_au8RxBuf[m_uRxIdx++] = u8Byte;       // Found -> store it
            return;                                                     // Otherwise drop the byte
        }
        if (m_uRxIdx == 1) {                                            // Waiting for sync byte 2
            if (u8Byte == SYNC2) m_au8RxBuf[m_uRxIdx++] = u8Byte;       // Found -> store it
            else if (u8Byte != SYNC1) m_uRxIdx = 0;                     // Not a sync pair -> start over
            return;                                                     // (repeated SYNC1 keeps index 1)
        }
        m_au8RxBuf[m_uRxIdx++] = u8Byte;                                // Store header / payload / CRC byte
        if (m_uRxIdx == HEADER_SIZE) {                                  // Header complete -> check size early
            uint16_t u16Size;                                           // Payload size from the header
            memcpy(&u16Size, &m_au8RxBuf[4], sizeof(u16Size));          // Read it (unaligned-safe)
            if (u16Size != sizeof(TData)) {                             // Other board uses a different TData ...
                m_u32CrcErrors = m_u32CrcErrors + 1;                    // ... count as rejected frame
                m_uRxIdx = 0;                                           // ... resynchronize
            }
            return;                                                     // Continue with the payload
        }
        if (m_uRxIdx < FRAME_SIZE) return;                              // Frame not complete yet
        m_uRxIdx = 0;                                                   // Prepare for the next frame
        vHandleFrame();                                                 // Validate and publish the frame
    }                                                                   // vParseByte body end

    void vHandleFrame() {                                               // Validate a complete frame in m_au8RxBuf
        uint16_t u16RxCrc;                                              // CRC received with the frame
        memcpy(&u16RxCrc, &m_au8RxBuf[FRAME_SIZE - sizeof(uint16_t)], sizeof(u16RxCrc)); // Read it (unaligned-safe)
        uint16_t u16Crc = u16Crc16(m_au8RxBuf, FRAME_SIZE - sizeof(uint16_t)); // Recompute CRC
        if (u16Crc != u16RxCrc) {                                       // Corrupted frame ...
            m_u32CrcErrors = m_u32CrcErrors + 1;                        // ... count it
            return;                                                     // ... and drop it
        }
        xSemaphoreTake(m_xMutex, portMAX_DELAY);                        // Lock shared data
        memcpy(&m_sRxData, &m_au8RxBuf[HEADER_SIZE], sizeof(TData));    // Publish received payload
        m_bNewRx = true;                                                // Mark as unread
        xSemaphoreGive(m_xMutex);                                       // Unlock shared data
        m_u32LastRxMs = millis();                                       // Remember arrival time
        m_bEverRx = true;                                               // Link has been seen alive
    }                                                                   // vHandleFrame body end

    static uint16_t u16Crc16(const uint8_t* pu8Data, size_t uLen) {     // CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)
        uint16_t u16Crc = 0xFFFF;                                       // Initial value
        for (size_t i = 0; i < uLen; i++) {                             // For every byte ...
            u16Crc ^= (uint16_t)pu8Data[i] << 8;                        // ... mix it into the high byte
            for (uint8_t b = 0; b < 8; b++) {                           // ... process 8 bits
                u16Crc = (u16Crc & 0x8000) ? (uint16_t)((u16Crc << 1) ^ 0x1021) : (uint16_t)(u16Crc << 1);
            }
        }
        return u16Crc;                                                  // Final CRC
    }                                                                   // u16Crc16 body end
};                                                                      // Class definition end

#endif                                                                  // UART_LINK_H
