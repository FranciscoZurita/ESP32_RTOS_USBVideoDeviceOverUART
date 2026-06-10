# ESP32 USB Webcam over UART

Two ESP32-S3 boards that together act as a USB webcam — no native camera USB
support required on the capturing side.

```
┌─────────────────────┐   UART @ 2 Mbaud   ┌──────────────────────┐
│   usb_dev_TX_RTOS   │ ─────────────────► │   usb_dev_RX_RTOS    │
│                     │   MJPEG frames      │                      │
│  ESP32-S3 + OV2640  │   GPIO43 → GPIO18  │  ESP32-S3 (no cam)   │
│  Captures + encodes │                     │  UVC device over USB │
└─────────────────────┘                     └──────────┬───────────┘
                                                        │ USB
                                                        ▼
                                                   Host PC
                                              (sees a webcam)
```

The TX board captures MJPEG frames from an OV2640 camera using `esp32-camera`
and streams them over UART. The RX board receives the frames, detects JPEG
end-of-image markers (`0xFF 0xD9`), and feeds them to the USB host via the
`usb_device_uvc` component — appearing as a standard UVC webcam.

---

## Why this exists

The ESP32-S3's native USB peripheral can implement UVC, but it cannot also run
a camera interface at the same time — the camera parallel interface and
high-speed USB share silicon resources. This project works around that by
splitting the roles across two boards over UART.

---

## Hardware

| Role | Board | Camera |
|---|---|---|
| TX (capture) | ESP32-S3 DevKitC-1 | OV2640 (parallel DVP) |
| RX (USB device) | ESP32-S3 DevKitC-1 | None |

**Wiring:**

| TX pin | RX pin | Signal |
|---|---|---|
| GPIO43 | GPIO18 | UART TX → RX |
| GND | GND | Common ground |

UART1 · 2 Mbaud · 8N1 · no flow control

---

## Firmware

### `usb_dev_TX_RTOS` — Capture + Transmit

Two FreeRTOS tasks communicate via semaphore and task notification:

- **`capture_task`** — calls `esp_camera_fb_get()`, signals `uart_task` via
  `xTaskNotifyGive`
- **`uart_task`** — waits for notification, writes the JPEG buffer to UART1,
  appends a single `0xD9` end byte as the frame delimiter

Camera config: QVGA (320×240), MJPEG, quality 30, 2 frame buffers in PSRAM.

### `usb_dev_RX_RTOS` — Receive + USB

Two FreeRTOS tasks sharing a mutex-protected frame buffer:

- **`uart_event_task`** — driven by the ESP-IDF UART event queue; uses pattern
  detection on `0xFF 0xD9` (JPEG EOI) to know when a complete frame has
  arrived, then reads it into `uart_buffer`
- **`usb_task`** — initializes the UVC device via `uvc_device_init()`;
  `camera_fb_get_cb` serves the latest frame to the USB host on demand

The `bootloader_components/boot_hooks` component disables the USB-Serial-JTAG
D+ pull-up before the OS starts, so the host sees the UVC device instead of a
debug interface.

---

## Build & Flash

Requires ESP-IDF v5.3 or later.

```bash
# Flash the TX board (connects to the camera)
cd usb_dev_TX_RTOS
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor

# Flash the RX board (connects to the host PC via USB)
cd usb_dev_RX_RTOS
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB1 flash monitor
```

On first build, IDF Component Manager will fetch `espressif/esp32-camera` and
`usb_device_uvc` automatically via `idf_component.yml`.

### menuconfig notes

TX: enable PSRAM (`Component config → ESP PSRAM`) and set both Flash and PSRAM
frequencies to 80 MHz. The camera frame buffers live in PSRAM
(`CAMERA_FB_IN_PSRAM`).

RX: select your board pinout under `USB WebCam config → Camera Pin
Configuration`. The default is ESP-S3-EYE; use `CAMERA_MODULE_CUSTOM` to
enter pins manually.

---

## Tested configuration

| Parameter | Value |
|---|---|
| ESP-IDF | v5.3.1 |
| Target | ESP32-S3 |
| Camera sensor | OV2640 |
| Resolution | QVGA (320×240) |
| Format | MJPEG |
| UART baud rate | 2,000,000 |
| Host OS | Linux (V4L2 / `ffplay`) |

```bash
# Verify the UVC device is enumerated
lsusb | grep -i espressif

# Preview stream
ffplay /dev/video0
```

---

## Known limitations

- **Single frame buffer on RX:** if the USB host polls faster than UART
  delivers frames, the previous frame is repeated. There is no explicit
  buffering between UART reception and USB delivery.
- **No flow control:** at 2 Mbaud, UART is fast enough for QVGA MJPEG at
  ~10 fps. Higher resolutions or frame rates will require hardware flow control
  or a higher baud rate.
- **Frame delimiter fragility:** the receiver detects `0xFF 0xD9` as JPEG EOI.
  This works for OV2640 MJPEG output but is not a robust framing protocol —
  any in-band `0xFF 0xD9` sequence in the payload would cause a false trigger.
  A proper length-prefixed framing layer would fix this.
