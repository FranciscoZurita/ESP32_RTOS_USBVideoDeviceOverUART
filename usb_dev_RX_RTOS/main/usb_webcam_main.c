// From template
#include <stdio.h>

// To debug
#include "esp_log.h"

/////////////////TIMER//////////////////////////////////////////////////////////
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/esp_timer.html
// https://github.com/espressif/esp-idf/tree/v5.3.1/examples/system/esp_timer
// add the following to your CMakeLists.txt: REQUIRES esp_timer
// Include #include "esp_timer.h" in your code
////////////////////////////////////////////////////////////////////////////////

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

/////////////////UART///////////////////////////////////////////////////////////
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/uart.html
// https://github.com/espressif/esp-idf/tree/v5.3.1/examples/system/esp_timer
// add the following to your CMakeLists.txt: REQUIRES esp_driver_uart
// Include #include "driver/uart.h" in your code
////////////////////////////////////////////////////////////////////////////////

#include "driver/uart.h"
#include "esp_intr_alloc.h"
#include "freertos/queue.h"
#include "hal/uart_ll.h"
#include "soc/interrupts.h"
QueueHandle_t uart_queue;

/////////////////FreeRTOS//////////////////////////////////////////////////////////
// Include #include "esp_heap_caps.h" in your code
// Include #include "freertos/FreeRTOS.h" in your code
// Include #include "freertos/task.h" in your code
////////////////////////////////////////////////////////////////////////////////
#include "esp_heap_caps.h"
// Global variables
SemaphoreHandle_t bufferMutex;

// Task handle for UART sending task
TaskHandle_t usbTaskHandle = NULL;
TaskHandle_t uartEventTaskHandle = NULL;

/////////////////USB///////////////////////////////////////////////////////////
// https://docs.espressif.com/projects/esp-iot-solution/en/latest/usb/usb_device/usb_device_uvc.html
// https://github.com/espressif/esp-iot-solution/tree/09c4d112/examples/usb/device/usb_webcam
// Include #include "usb_device_uvc.h" in your code
////////////////////////////////////////////////////////////////////////////////

#include "usb_device_uvc.h"
#include "uvc_frame_config.h"

// To debug
static const char *TAG = "usb_webcam_uart";

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#define UART_PORT_NUM UART_NUM_1
#define UART_TX_PIN 17 //pin 17
#define UART_RX_PIN 18 //pin 18
#define UART_BAUD_RATE 2000000

// Debug purposes
#if 0
#define BUF_SIZE (1024)   // Size of the buffer for incoming data
uint8_t data[BUF_SIZE];   // Array to hold the incoming data
#endif

#define UVC_MAX_FRAMESIZE_SIZE (8*1024) // Maintain the maximum frame size
#define UART_BUFFER_SIZE (UVC_MAX_FRAMESIZE_SIZE)

// Buffer to hold incoming image data from UART
DMA_ATTR uint8_t uart_buffer[UART_BUFFER_SIZE];

// Structure to hold frame buffer
typedef struct {
    uvc_fb_t uvc_fb;
} fb_t;


static fb_t s_fb;

// Example No Signal Frame (black 320x240 JPEG)
static const uint8_t no_signal_jpeg[] = {
    // Insert your JPEG-encoded black screen image data here
    // This is an example placeholder for a black JPEG (320x240)
};

#define NO_SIGNAL_JPEG_SIZE sizeof(no_signal_jpeg)

static uint8_t *last_valid_frame = NULL;
static size_t last_valid_frame_size = 0;

// For the UART interrupts
static void uart_event_task(void *pvParameters);

// Initialize UART for receiving JPEG frames
static void uart_init()
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };


    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 1024 * 8, 0, 4*1024, &uart_queue, 0)); // Original
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    // ESP_ERROR_CHECK(uart_disable_intr_mask(UART_PORT_NUM, UART_RXFIFO_FULL_INT_ENA_M | UART_FRM_ERR_INT_CLR_M));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(UART_PORT_NUM, 0xD9, 2, 9, 0, 0));
    ESP_ERROR_CHECK(uart_pattern_queue_reset(UART_PORT_NUM, 20));
}

// Read JPEG frame from UART
static int uart_read_frame(uint8_t *buffer, size_t buffer_size) {

    int len = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT_NUM, (size_t*)&len));
    len = uart_read_bytes(UART_PORT_NUM, buffer, len-1, 20 / portTICK_RATE_MS);
    if (len <= 0) {
        ESP_LOGI(TAG, "No new data from UART");
        // No new data from UART, return failure
        return -1;
    }
    else {
        // Output the size of the captured frame to the console
        ESP_LOGI(TAG, "Captured frame size: %d bytes", len);
    }
    return len;
}

// Callback for when the USB device starts
static esp_err_t camera_start_cb(uvc_format_t format, int width, int height, int rate, void *cb_ctx)
{
    ESP_LOGI(TAG, "USB Webcam Start");
    ESP_LOGI(TAG, "Format: %d, width: %d, height: %d, rate: %d", format, width, height, rate);

    if (format != UVC_FORMAT_JPEG) {
        ESP_LOGE(TAG, "Only MJPEG format is supported");
        return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_OK;
}

// Get a frame from UART and pass it to the USB framework
static uvc_fb_t* camera_fb_get_cb(void *cb_ctx) {

    // Fill UVC frame buffer with the last valid frame
    if (xSemaphoreTake(bufferMutex, portMAX_DELAY)) {
        s_fb.uvc_fb.buf = last_valid_frame;
        s_fb.uvc_fb.len = last_valid_frame_size;
        s_fb.uvc_fb.width = 320;    // Example width of the "no signal" frame
        s_fb.uvc_fb.height = 240;   // Example height of the "no signal" frame
        s_fb.uvc_fb.format = UVC_FORMAT_JPEG;
        int64_t now = esp_timer_get_time();
        s_fb.uvc_fb.timestamp.tv_sec = now / 1000000;  // Convert microseconds to seconds
        s_fb.uvc_fb.timestamp.tv_usec = now % 1000000; // Get the remaining microseconds

        // return &s_fb.uvc_fb;
        xSemaphoreGive(bufferMutex); // Release the buffer after processing
    }
    return &s_fb.uvc_fb;
}

// Return the frame after it has been sent to the USB host
static void camera_fb_return_cb(uvc_fb_t *fb, void *cb_ctx)
{
    // Simply return the frame buffer; no camera to release resources from
    (void)fb;
    (void)cb_ctx;
    ESP_LOGI(TAG, "Frame returned");
}

// Callback for when the USB video device stops
static void camera_stop_cb(void *cb_ctx)
{
    ESP_LOGI(TAG, "USB Webcam Stop");
}

/////////////////////
// UART Interrupt  //
/////////////////////

// https://www.esp32.com/viewtopic.php?t=33960
// https://github.com/theElementZero/ESP32-UART-interrupt-handling/blob/master/uart_interrupt.c
// https://github.com/espressif/esp-idf/blob/v5.3.1/examples/peripherals/uart/uart_events/main/uart_events_example_main.c
// https://github.com/vcc-gnd/YD-ESP32-S3
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html

// UART event handler task
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    while (1) {
        // Wait for UART event.
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
                case UART_BREAK:
                    // ESP_LOGI(TAG, "UART break detected");
                    break;
                case UART_PATTERN_DET:
                    // Pattern detected (JPEG EOI)
                    ESP_LOGI(TAG, "JPEG End Detected (0xFF 0xD9)");
                    
                    if (xSemaphoreTake(bufferMutex, portMAX_DELAY)) 
                    {
                        size_t len = 0; 
                        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT_NUM, &len));
                        int uart_ret = uart_read_frame(uart_buffer, len);
                        if (uart_ret > 0) {
                            // New valid frame received from UART
                            last_valid_frame = uart_buffer;
                            last_valid_frame_size = uart_ret;
                        } else if (!last_valid_frame) {
                            // No valid frame received, and no previous valid frame, use no signal frame
                            last_valid_frame = (uint8_t *)no_signal_jpeg;
                            last_valid_frame_size = NO_SIGNAL_JPEG_SIZE;
                        }
                        ESP_LOGI(TAG, "Free stack size: %d", uxTaskGetStackHighWaterMark(NULL));
                        uart_flush_input(UART_PORT_NUM);
                        xSemaphoreGive(bufferMutex);  // Signal that frame is ready
                    }
                    xQueueReset(uart_queue);
                break;
                default:
                    // ESP_LOGI(TAG, "Unhandled UART event: %d", event.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}

// USB TASK //

void usb_task(void *pvParameter) {
    ESP_LOGI(TAG, "Starting USB Webcam over UART");

    // Allocate buffer for UVC frames
    uint8_t *uvc_buffer = (uint8_t *)malloc(UVC_MAX_FRAMESIZE_SIZE);
    if (uvc_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate UVC frame buffer");
        vTaskDelete(NULL);
    }

    // Configure the USB video device
    uvc_device_config_t config = {
        .uvc_buffer = uvc_buffer,
        .uvc_buffer_size = UVC_MAX_FRAMESIZE_SIZE,
        .start_cb = camera_start_cb,
        .fb_get_cb = camera_fb_get_cb,
        .fb_return_cb = camera_fb_return_cb,
        .stop_cb = camera_stop_cb,
    };

    ESP_ERROR_CHECK(uvc_device_config(0, &config));
    ESP_ERROR_CHECK(uvc_device_init());

    vTaskDelete(NULL);  // Delete the task when done
}

/****************************************************************************************** */
void app_main(void)
{
    uart_init();
    // Initialize the mutex
    bufferMutex = xSemaphoreCreateMutex();

    BaseType_t usbResult = xTaskCreatePinnedToCore(usb_task, "usbTask", 8 * 1024, NULL, 2, &usbTaskHandle, tskNO_AFFINITY); // Core 0
    if (usbResult != pdPASS) {
        ESP_LOGE("INIT", "Failed to create usbTask");
    } else {
        ESP_LOGI("INIT", "Successfully created usbTask on Core 0");
    }

    BaseType_t result = xTaskCreatePinnedToCore(uart_event_task, "uartEventTask", 8*1024, NULL, 2, &uartEventTaskHandle, tskNO_AFFINITY );

    if (result != pdPASS) {
        ESP_LOGE("INIT", "Failed to create uartEventTask");
    }
    else {ESP_LOGI("INIT", "Successfully created uartEventTask");}

    // Infinite loop to keep task running
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
