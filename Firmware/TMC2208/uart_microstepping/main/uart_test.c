#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "TMC2208";

// ---------------- PIN CONFIG ----------------
#define STEP_PIN   GPIO_NUM_26
#define DIR_PIN    GPIO_NUM_33

#define UART_TX_PIN GPIO_NUM_14
#define UART_RX_PIN GPIO_NUM_12

#define UART_PORT  UART_NUM_1
#define BAUDRATE   115200

// ---------------- CRC ----------------
uint8_t crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (b & 1))
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
            b >>= 1;
        }
    }
    return crc;
}

// ---------------- UART INIT ----------------
void uart_init_tmc()
{
    uart_config_t config = {
        .baud_rate = BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_PORT, &config);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_PORT, 1024, 1024, 0, NULL, 0);

    // Improve UART responsiveness
    uart_set_rx_timeout(UART_PORT, 2);
    uart_set_rx_full_threshold(UART_PORT, 1);
}

// ---------------- WRITE REGISTER ----------------
void tmc_write(uint8_t reg, uint32_t data)
{
    uint8_t buf[8];

    buf[0] = 0x05;         // sync
    buf[1] = 0x00;         // slave addr
    buf[2] = reg | 0x80;   // write

    buf[3] = data >> 24;
    buf[4] = data >> 16;
    buf[5] = data >> 8;
    buf[6] = data;

    buf[7] = crc8(buf, 7);

    uart_write_bytes(UART_PORT, (char*)buf, 8);
    printf("Packet Sent: for 1/256 microstepping\n");
    for(int i=0;i<8;i++) {
        printf("0x%02X ", buf[i]);
    }
    printf("\n");
    // uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(10));
}

// ---------------- READ REGISTER ----------------
// ---------------- READ REGISTER (FIXED) ----------------
bool tmc_read(uint8_t reg, uint32_t *val)
{
    uint8_t req[4];

    req[0] = 0x05;
    req[1] = 0x00;
    req[2] = 0x00;   // read: MSB=0 (no 0x80)
    req[3] = crc8(req, 3);

    uart_flush_input(UART_PORT);

    uart_write_bytes(UART_PORT, (char*)req, 4);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(10));

    // Step 1: Read and discard the 4 echo bytes (our own TX)
    uint8_t echo[4];
    int echo_len = uart_read_bytes(UART_PORT, echo, 4, pdMS_TO_TICKS(50));
    if (echo_len != 4) {
        ESP_LOGE(TAG, "Echo read failed, got %d bytes", echo_len);
        return false;
    }

    // Step 2: Read the actual 8-byte response from TMC2208
    uint8_t resp[8];
    int len = uart_read_bytes(UART_PORT, resp, 8, pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Response (%d bytes):", len);
    for (int i = 0; i < len; i++) {
        printf("0x%02X ", resp[i]);
    }
    printf("\n");

    if (len != 8) {
        ESP_LOGE(TAG, "Short response: got %d bytes (expected 8)", len);
        return false;
    }

    // Verify CRC over first 7 bytes
    uint8_t crc = crc8(resp, 7);
    if (crc != resp[7]) {
        ESP_LOGE(TAG, "CRC mismatch: calc=0x%02X, recv=0x%02X", crc, resp[7]);
        return false;
    }

    *val = ((uint32_t)resp[3] << 24) |
           ((uint32_t)resp[4] << 16) |
           ((uint32_t)resp[5] <<  8) |
           (uint32_t)resp[6];
    return true;
}

// bool tmc_read(uint8_t reg, uint32_t *val)
// {
//     uint8_t req[4];

//     req[0] = 0x05;
//     req[1] = 0x00;
//     req[2] = reg;
//     req[3] = crc8(req, 3);

//     uart_flush_input(UART_PORT);

//     uart_write_bytes(UART_PORT, (char*)req, 4);
//     printf("test Packet Sent 2");
//     uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(10));

//     uint8_t resp[8];
//     int len = uart_read_bytes(UART_PORT, resp, 8, 200 / portTICK_PERIOD_MS);
//     printf("Received %d bytes:\n", len); 
//     for(int i=0;i<len;i++) {
//         printf("0x%02X ", resp[i]);
//     }
//     printf("\n");
//     if (len == 0) {
//         ESP_LOGE(TAG, "No response (%d)", len);
//         return false;
//     }

//     uint8_t crc = crc8(resp, 7);
//     if (crc != resp[7]) {
//         ESP_LOGE(TAG, "CRC error");
//         return false;
//     }

//     *val = (resp[3]<<24)|(resp[4]<<16)|(resp[5]<<8)|resp[6];
//     return true;
// }

// ---------------- RETRY READ ----------------
bool tmc_read_retry(uint8_t reg, uint32_t *val)
{
    for (int i = 0; i < 3; i++) {
        if (tmc_read(reg, val)) return true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

// ---------------- DRIVER INIT ----------------
void tmc_init()
{
    ESP_LOGI(TAG, "Initializing TMC2208...");

    // GCONF: pdn_disable=1 (bit6), I_scale_analog=1 (bit0)
    // bit6=1 disables PDN pin so UART works properly
    tmc_write(0x00, 0x00000041);  // I_scale_analog + pdn_disable

    // IHOLD_IRUN: IHOLD=10, IRUN=20, IHOLDDELAY=6
    // bits 4:0 = IHOLD, bits 12:8 = IRUN, bits 19:16 = IHOLDDELAY
    tmc_write(0x10, 0x00061410);

    // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, intpol=1, MRES=0(256 microsteps)
    tmc_write(0x6C, 0x10000053);

    // PWMCONF: stealthChop defaults (pwm_autoscale=1, pwm_autograd=1)
    tmc_write(0x70, 0xC10D0024);

    vTaskDelay(pdMS_TO_TICKS(200)); // Give TMC2208 time to settle
}

// void tmc_init()
// {
//     ESP_LOGI(TAG, "Initializing TMC2208...");

//     tmc_write(0x00, 0x00000040);  // GCONF
//     tmc_write(0x10, 0x00061F10);  // IHOLD_IRUN (current)
//     tmc_write(0x6C, 0x000100C3);  // CHOPCONF
//     tmc_write(0x70, 0xC10D0024);  // PWMCONF

//     vTaskDelay(pdMS_TO_TICKS(100));
// }

// ---------------- STEPPER TASK ----------------
void step_task(void *arg)
{
    gpio_set_level(DIR_PIN, 1);

    while (1) {
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(300);

        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(300);

        // Prevent watchdog reset
        vTaskDelay(1);
    }
}

// ---------------- MAIN ----------------
void app_main(void)
{
    // GPIO setup
    gpio_set_direction(STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);

    // UART init
    uart_init_tmc();
    vTaskDelay(pdMS_TO_TICKS(200));

    // Driver init
    tmc_init();

    // Test UART communication
    uint32_t val;
    if (tmc_read_retry(0x02, &val)) {
        ESP_LOGI(TAG, "UART OK, IFCNT = %lu", val);
    } else {
        ESP_LOGE(TAG, "UART FAILED (check wiring!)");
    }

    // Start motor task
    xTaskCreate(step_task, "step_task", 2048, NULL, 5, NULL);
}
