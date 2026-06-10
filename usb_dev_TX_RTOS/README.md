# usb_dev_TX_RTOS — Camera Capture + UART Transmit

ESP32-S3 firmware that captures MJPEG frames from an OV2640 camera and streams
them over UART1 at 2 Mbaud to the RX board.

## Pin assignment

| Signal | GPIO |
|---|---|
| UART TX | 43 |
| Camera XCLK | 10 |
| Camera SIOD (SDA) | 40 |
| Camera SIOC (SCL) | 39 |
| Camera D0–D7 | 15, 17, 18, 16, 14, 12, 11, 48 |
| Camera VSYNC | 38 |
| Camera HREF | 47 |
| Camera PCLK | 13 |

Matches the ESP32-S3 DevKitC-1 + OV2640 module wiring. Adjust in `main.c` if
using a different board.

## Task architecture

```
app_main
├── uart_init()        UART1 TX-only, 2 Mbaud
├── camera_init()      OV2640, QVGA MJPEG, 2 FB in PSRAM
├── capture_task  ─────────────────────────────────────┐
│     esp_camera_fb_get()                              │ xTaskNotifyGive
│     xSemaphoreGive(bufferMutex)                      ▼
└── uart_task                                   uart_task
      ulTaskNotifyTake()                        uart_write_bytes(fb->buf, fb->len)
      uart_write_bytes(0xD9)  ← frame delimiter
      esp_camera_fb_return()
```

## Frame framing

Each JPEG frame is sent raw over UART, followed by a single `0xD9` byte. The
RX board uses UART pattern detection on `0xFF 0xD9` (JPEG EOI) to delimit
frames. This works because OV2640 MJPEG output always ends with `0xFF 0xD9`.

## menuconfig requirements

- **PSRAM enabled** — `Component config → ESP PSRAM → Support for external SPI-connected RAM`
- **Flash frequency: 80 MHz** — `Serial flasher config → Flash SPI speed`
- **PSRAM frequency: 80 MHz** — `Component config → ESP PSRAM → Set RAM clock speed`
