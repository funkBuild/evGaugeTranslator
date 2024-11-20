#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/twai.h"

static const char *TAG = "example";

#define CAN_TX_PIN GPIO_NUM_27
#define CAN_RX_PIN GPIO_NUM_26
#define CAN_SE_PIN 23

static const twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

static void twai_receive_task(void *arg) {
  while (1) {
    twai_message_t rx_msg;

    // Receive messages
    if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
      // Decode and process SOC and Current messages
      if (rx_msg.identifier == 0x292) {
        uint16_t soc_raw =
            ((rx_msg.data[2] & 0x0F) << 6) | ((rx_msg.data[1] & 0xFC) >> 2);
        float soc = soc_raw / 10.0;
        ESP_LOGW(TAG, "SOC: %.1f%%", soc);
      } else if (rx_msg.identifier == 0x132) {
        int16_t current_raw = (rx_msg.data[3] << 8) | rx_msg.data[2];
        float current = current_raw / 10.0;
        ESP_LOGW(TAG, "Current: %.1f A", current);
      } else {
        // Log other messages
        ESP_LOGW(TAG, "Received message ID: 0x%03X",
                 (unsigned int)rx_msg.identifier);
        ESP_LOG_BUFFER_HEX(TAG, rx_msg.data, rx_msg.data_length_code);
      }
    } else {
      ESP_LOGE(TAG, "Failed to receive message");
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
