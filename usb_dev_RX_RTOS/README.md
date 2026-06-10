# usb_dev_RX_RTOS — UART Receive + USB UVC Device

ESP32-S3 firmware that receives MJPEG frames over UART and presents them to a
host PC as a standard USB Video Class (UVC) webcam.

## Pin assignment

| Signal | GPIO |
|---|---|
| UART RX | 18 |
| UART TX (unused) | 17 |
| USB D+ / D- | Native USB pins (no config needed) |

## Task architecture

```
app_main
├── uart_init()              UART1 @ 2 Mbaud, pattern detect on 0xFF 0xD9
├── bufferMutex              Mutex protecting last_valid_frame / last_valid_frame_size
├── usb_task
│     uvc_device_config()    Register start/fb_get/fb_return/stop callbacks
│     uvc_device_init()      Start USB stack — device enumerated by host
│     (task deletes itself)
└── uart_event_task
      xQueueReceive(uart_queue)
      UART_PATTERN_DET:
        uart_read_frame() → uart_buffer
        last_valid_frame = uart_buffer
        last_valid_frame_size = len
```

### UVC callbacks

| Callback | Behaviour |
|---|---|
| `camera_start_cb` | Validates that the host requested MJPEG; rejects other formats |
| `camera_fb_get_cb` | Takes mutex, fills `uvc_fb_t` from `last_valid_frame`, returns it |
| `camera_fb_return_cb` | No-op — buffer is static, no camera to return to |
| `camera_stop_cb` | Logs stop event |

## Boot hook

`bootloader_components/boot_hooks` runs before the OS and disables the
USB-Serial-JTAG pull-up (`USB_SERIAL_JTAG_DP_PULLUP`). Without this, the host
enumerates the debug interface instead of the UVC device.

## menuconfig

Select your board under `USB WebCam config → Camera Pin Configuration`.
Default is ESP-S3-EYE. Use `CAMERA_MODULE_CUSTOM` to enter pins manually.

## Verifying enumeration

```bash
# Linux
lsusb           # look for Espressif entry
ffplay /dev/video0

# Windows
# Device Manager → Imaging devices → should show a camera
# Open Camera app or VLC → Media → Open Capture Device
```
