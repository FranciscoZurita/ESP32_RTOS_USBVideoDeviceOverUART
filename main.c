// From template
#include <stdio.h>

/////////////////CAMERA/////////////////////////////////////////////////////////
// https://github.com/espressif/esp32-camera
// idf.py add-dependency "espressif/esp32-camera"
// Enable PSRAM in menuconfig (also set Flash and PSRAM frequiencies to 80MHz)
// Include esp_camera.h in your code
////////////////////////////////////////////////////////////////////////////////

#include "esp_camera.h"

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

/////////////////FreeRTOS//////////////////////////////////////////////////////////
// Include #include "esp_heap_caps.h" in your code
// Include #include "freertos/FreeRTOS.h" in your code
// Include #include "freertos/task.h" in your code
////////////////////////////////////////////////////////////////////////////////
#include "esp_heap_caps.h"
// Global variables
SemaphoreHandle_t bufferMutex;
camera_fb_t *fb = NULL;

// Task handle for UART sending task
TaskHandle_t captureTaskHandle = NULL;
TaskHandle_t uartTaskHandle = NULL;

/////////////////UART///////////////////////////////////////////////////////////
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/uart.html
// add the following to your CMakeLists.txt: REQUIRES esp_driver_uart
// Include #include "driver/uart.h" in your code
////////////////////////////////////////////////////////////////////////////////

#include "driver/uart.h"

// To debug
static const char *TAG = "camera_uart";

// Camera definitions

#define COLOR 1

#if COLOR
#define CAMERA_XCLK_FREQ           20000000
#else
#define CAMERA_XCLK_FREQ           10000000
#endif
#define CAMERA_FB_COUNT            2

#define CAMERA_MODULE_SOC "esp32s3"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1
#define CAMERA_PIN_XCLK     10
#define CAMERA_PIN_SIOD     40
#define CAMERA_PIN_SIOC     39

#define CAMERA_PIN_D7       48
#define CAMERA_PIN_D6       11
#define CAMERA_PIN_D5       12
#define CAMERA_PIN_D4       14
#define CAMERA_PIN_D3       16
#define CAMERA_PIN_D2       18
#define CAMERA_PIN_D1       17
#define CAMERA_PIN_D0       15
#define CAMERA_PIN_VSYNC    38
#define CAMERA_PIN_HREF     47
#define CAMERA_PIN_PCLK     13

#define UART_TX_PIN 43    // GPIO 43 for UART TX
#define UART_PORT_NUM UART_NUM_1
#define UART_BAUD_RATE 2000000

#define CHUNK_SIZE 1024  // Define the size of each chunk
#define DELAY_BETWEEN_CHUNKS_MS 20  // Define the delay between sending each chunk


#if COLOR
    size_t jpg_len = 0;
    uint8_t *jpg_buf = NULL;
#endif

//////////////////////
// Camera functions //
//////////////////////

static esp_err_t camera_init(uint32_t xclk_freq_hz, pixformat_t pixel_format, framesize_t frame_size, int jpeg_quality, uint8_t fb_count)
{
    static bool inited = false;
    static uint32_t cur_xclk_freq_hz = 0;
    static pixformat_t cur_pixel_format = 0;
    static framesize_t cur_frame_size = 0;
    static int cur_jpeg_quality = 0;
    static uint8_t cur_fb_count = 0;

    if ((inited && cur_xclk_freq_hz == xclk_freq_hz && cur_pixel_format == pixel_format
            && cur_frame_size == frame_size && cur_fb_count == fb_count && cur_jpeg_quality == jpeg_quality)) {
        ESP_LOGD(TAG, "camera already inited");
        return ESP_OK;
    } else if (inited) {
        esp_camera_return_all();
        esp_camera_deinit();
        inited = false;
        ESP_LOGI(TAG, "camera RESTART");
    }

    camera_config_t camera_config = {
        .pin_pwdn = CAMERA_PIN_PWDN,
        .pin_reset = CAMERA_PIN_RESET,
        .pin_xclk = CAMERA_PIN_XCLK,
        .pin_sscb_sda = CAMERA_PIN_SIOD,
        .pin_sscb_scl = CAMERA_PIN_SIOC,

        .pin_d7 = CAMERA_PIN_D7,
        .pin_d6 = CAMERA_PIN_D6,
        .pin_d5 = CAMERA_PIN_D5,
        .pin_d4 = CAMERA_PIN_D4,
        .pin_d3 = CAMERA_PIN_D3,
        .pin_d2 = CAMERA_PIN_D2,
        .pin_d1 = CAMERA_PIN_D1,
        .pin_d0 = CAMERA_PIN_D0,
        .pin_vsync = CAMERA_PIN_VSYNC,
        .pin_href = CAMERA_PIN_HREF,
        .pin_pclk = CAMERA_PIN_PCLK,

        .xclk_freq_hz = xclk_freq_hz,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = pixel_format,
        .frame_size = frame_size,
#if COLOR
        .jpeg_quality = jpeg_quality,
        .fb_count = fb_count,
#else
        .jpeg_quality = jpeg_quality,
        .fb_count = fb_count,
#endif
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .fb_location = CAMERA_FB_IN_PSRAM
    };

    // initialize the camera sensor
    esp_err_t ret = esp_camera_init(&camera_config);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Get the sensor object, and then use some of its functions to adjust the parameters when taking a photo.
    // Note: Do not call functions that set resolution, set picture format and PLL clock,
    // If you need to reset the appeal parameters, please reinitialize the sensor.
    
    sensor_t *s = esp_camera_sensor_get();
    /*
    s->set_vflip(s, 1); // flip it back
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
        s->set_brightness(s, 1); // up the blightness just a bit
        s->set_saturation(s, -2); // lower the saturation
    }

    if (s->id.PID == OV3660_PID || s->id.PID == OV2640_PID) {
        s->set_vflip(s, 1); // flip it back
    } else if (s->id.PID == GC0308_PID) {
        s->set_hmirror(s, 0);
    } else if (s->id.PID == GC032A_PID) {
        s->set_vflip(s, 1);
    }
    */

    // Get the basic information of the sensor.
    camera_sensor_info_t *s_info = esp_camera_sensor_get_info(&(s->id));
    
#if COLOR
    if (ESP_OK == ret && PIXFORMAT_JPEG == pixel_format && s_info->support_jpeg == true) {
        cur_xclk_freq_hz = xclk_freq_hz;
        cur_pixel_format = pixel_format;
        cur_frame_size = frame_size;
        cur_jpeg_quality = jpeg_quality;
        cur_fb_count = fb_count;
        inited = true;
    } else {
        ESP_LOGE(TAG, "JPEG format is not supported");
        return ESP_ERR_NOT_SUPPORTED;
    }
#else 
    if (ESP_OK == ret && PIXFORMAT_GRAYSCALE == pixel_format && s_info->support_jpeg == true) {
        cur_xclk_freq_hz = xclk_freq_hz;
        cur_pixel_format = pixel_format;
        cur_frame_size = frame_size;
        cur_fb_count = 1;
        inited = true;
    } else {
        ESP_LOGE(TAG, "JPEG format is not supported");
        return ESP_ERR_NOT_SUPPORTED;
    }
#endif
    return ret;

}

////////////////////
// UART functions //
////////////////////

// Function to initialize the UART
static void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    uart_driver_install(UART_PORT_NUM, 1024 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/////////////////////
//          Tasks  //
/////////////////////

// Task to capture image
void capture_task(void *pvParameter) {
    while (true) {

        ESP_LOGI(TAG, "Starting capture...");

        // Lock the mutex before accessing the frame buffer
        BaseType_t xStatus = xSemaphoreTake(bufferMutex, portMAX_DELAY);
        if (xStatus == pdTRUE) {
            // Semaphore was successfully taken
            ESP_LOGI(TAG,"Semaphore taken successfully");

            // Critical section code here
            
            // Give back the semaphore
            xSemaphoreGive(bufferMutex);
        } else {
            // Semaphore could not be taken
            ESP_LOGI(TAG, "Failed to take semaphore, status: %d", xStatus);
        }
        
        if (fb) {
            // Return the previous frame buffer if it exists
            ESP_LOGI(TAG,"frame buffer if it exists...");
            esp_camera_fb_return(fb);
        }

        // Capture a new frame
        fb = esp_camera_fb_get();
        if (!fb) {
            // Handle error
            xSemaphoreGive(bufferMutex);
            continue;
        }

        ESP_LOGI(TAG, "Captured frame");
        // Unlock the mutex after capturing
        xSemaphoreGive(bufferMutex);

        UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI(TAG,"Stack high water mark: %u words", stackHighWaterMark);

        // In the capturing task
        xTaskNotifyGive(uartTaskHandle);  // Notify UART task that new data is available

        // Optional: Wait before capturing the next frame
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// Task to process image data
void uart_task(void *pvParameter) {
    while (true) {

        // In the UART sending task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for notification

        // Lock the mutex before accessing the frame buffer
        xSemaphoreTake(bufferMutex, portMAX_DELAY);

        if (fb) {
            // Process the image data in fb->buf
            char test[1] = {0xD9};
            // int uart_ret = uart_write_bytes(UART_NUM_1, fb->len, sizeof(size_t)); // send the length before or the null character // TODO: THIS IS NOT WORKING
            // fb->buf[fb->len] = test[0];
            // fb->len = fb->len+1;

            // Process the received data here
            int uart_ret = uart_write_bytes(UART_NUM_1, (const char *)fb->buf, fb->len);
            if (uart_ret != fb->len) {
                ESP_LOGE(TAG, "UART transmission failed");
            }
            else {ESP_LOGI(TAG, "UART transmission succeed");}
            uart_ret = uart_write_bytes(UART_NUM_1, test, 1);
            
            // After processing, return the buffer
            esp_camera_fb_return(fb);
            fb = NULL;
        }

        // Unlock the mutex after processing
        xSemaphoreGive(bufferMutex);

        // Optional: Wait before processing the next frame
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "Starting code...");

    // Initialize UART
    uart_init();

    int compression_rate = 30;

    // Initialize camera
#if COLOR
    esp_err_t ret = camera_init(CAMERA_XCLK_FREQ, PIXFORMAT_JPEG, FRAMESIZE_QVGA, compression_rate, CAMERA_FB_COUNT); // color
#else
    esp_err_t ret = camera_init(CAMERA_XCLK_FREQ, PIXFORMAT_GRAYSCALE, FRAMESIZE_QVGA, 12, 1); // grayscale
#endif
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", ret);
        return;
    }

    // Initialize the mutex
    bufferMutex = xSemaphoreCreateMutex();
    
    // Create tasks
    BaseType_t result;
    result = xTaskCreatePinnedToCore(capture_task, "CaptureTask", 4*1024, NULL, 2, &captureTaskHandle, tskNO_AFFINITY );
    if (result != pdPASS) {
        ESP_LOGE("INIT", "Failed to create CaptureTask");
    }
    else {ESP_LOGI("INIT", "Successfully created CaptureTask");}
    
    result = xTaskCreatePinnedToCore(uart_task, "uartTask", 4*1024, NULL, 2, &uartTaskHandle, tskNO_AFFINITY );
    if (result != pdPASS) {
        ESP_LOGE("INIT", "Failed to create uartTask");
    }
    else {ESP_LOGI("INIT", "Successfully created uartTask");}

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to keep the task running
        // take_picture(NULL);
    }
}
