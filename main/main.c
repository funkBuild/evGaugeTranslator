#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/twai.h"
#include <stdint.h>

static const char *TAG = "example";

#define CAN_TX_PIN GPIO_NUM_27
#define CAN_RX_PIN GPIO_NUM_26
#define CAN_SE_PIN GPIO_NUM_23

static const twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

void send_amps_and_temp(float volts, float amps, float temp) {
  twai_message_t msg = {
      .extd = 0,
      .rtr = 0,
      .ss = 0,
      .self = 0,
      .dlc_non_comp = 0,
      .identifier = 0x356,
      .data_length_code = 8,
      .data = {0},
  };

  // Pack voltage (unsigned, little endian), scale 0.1
  uint16_t volts_scaled = (uint16_t)(volts * 10);
  msg.data[0] = (volts_scaled & 0xFF);
  msg.data[1] = (volts_scaled >> 8) & 0xFF;

  // Pack current (signed, little endian), scale 0.1
  int16_t amps_scaled = (int16_t)(amps * 10);
  msg.data[2] = (amps_scaled & 0xFF);
  msg.data[3] = (amps_scaled >> 8) & 0xFF;

  // Pack temperature (signed, little endian), scale 0.1
  int16_t temp_scaled = (int16_t)(temp * 10);
  msg.data[4] = (temp_scaled & 0xFF);
  msg.data[5] = (temp_scaled >> 8) & 0xFF;

  twai_transmit(&msg, pdMS_TO_TICKS(50));
}

void send_battery_level(float soc) {
  twai_message_t msg = {
      .extd = 0,
      .rtr = 0,
      .ss = 0,
      .self = 0,
      .dlc_non_comp = 0,
      .identifier = 0x355,
      .data_length_code = 8,
      .data = {0},
  };

  // Pack battery level (unsigned, little endian)
  uint16_t level = (uint16_t)(soc);
  msg.data[0] = (level & 0xFF);
  msg.data[1] = (level >> 8) & 0xFF;

  twai_transmit(&msg, pdMS_TO_TICKS(50));
}

static void twai_receive_task(void *arg) {
  float soc = 50.0;     // TODO: Change back to 0.0 after testing
  float current = 50.0; // TODO: Change back to 0.0 after testing
  float voltage = 0;
  float temp = 0;

  int64_t last_tx_time = esp_timer_get_time();
  int64_t last_log_time = esp_timer_get_time();

  while (1) {
    twai_message_t rx_msg;

    // Receive messages
    if (twai_receive(&rx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
      switch (rx_msg.identifier) {
      case 0x292: {
        uint16_t soc_raw =
            ((rx_msg.data[2] & 0x0F) << 6) | ((rx_msg.data[1] & 0xFC) >> 2);
        soc = soc_raw / 10.0;
        break;
      }
      case 0x132: {
        int16_t current_raw = (rx_msg.data[3] << 8) | rx_msg.data[2];
        current = current_raw / -10.0;

        uint16_t voltage_raw = (rx_msg.data[1] << 8) | rx_msg.data[0];
        voltage = voltage_raw * 0.01;
        break;
      }
      case 0x332: {
        uint8_t mux = (rx_msg.data[0] & 0x03);

        if (mux == 0) {
          unsigned int battery_max_temp = (rx_msg.data[2] * 5) - 400;
          temp = battery_max_temp / 10.0;
        }
      }
      }
    }

    // Send data every 100ms
    int64_t now = esp_timer_get_time();
    if (now - last_tx_time > 100000) {
      send_amps_and_temp(voltage, current, temp);
      send_battery_level(soc);
      last_tx_time = now;
    }

    // Log values every 1s
    if (now - last_log_time > 1000000) {
      ESP_LOGI(TAG, "SOC: %.1f%%", soc);
      ESP_LOGI(TAG, "Current: %.1f A", current);
      ESP_LOGI(TAG, "Voltage: %.2f V", voltage);
      ESP_LOGI(TAG, "Temperature: %.1f C", temp);
      last_log_time = now;
    }
  }

  vTaskDelete(NULL);
}

void app_main(void) {
  // Setup CAN_SE pin to ground for high speed mode
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = 1ULL << CAN_SE_PIN,
      .pull_down_en = 0,
      .pull_up_en = 0,
  };
  gpio_config(&io_conf);

  gpio_set_level(CAN_SE_PIN, 0);

  // Install the TWAI driver
  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  ESP_LOGI(TAG, "Driver installed");

  // Start the TWAI driver
  ESP_ERROR_CHECK(twai_start());
  ESP_LOGI(TAG, "Driver started");

  // Create and start the receiving task
  xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, 5, NULL,
                          tskNO_AFFINITY);

  // Application can continue running other tasks
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000)); // Keep main task alive
  }

  // Uninstall the TWAI driver (not reached in this example)
  ESP_ERROR_CHECK(twai_driver_uninstall());
  ESP_LOGI(TAG, "Driver uninstalled");
}
