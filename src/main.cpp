#include <Arduino.h>
#include "driver/twai.h"

// WROOM-1U CAN Pins
#define CAN_TX_PIN GPIO_NUM_17
#define CAN_RX_PIN GPIO_NUM_18

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[CAN RECEIVER] Starting...");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
        Serial.println("[CAN RECEIVER] Driver started successfully! Listening...");
    } else {
        Serial.println("[CAN RECEIVER] Driver initialization failed!");
    }
}

void loop() {
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
        // Parse raw bytes as signed 16-bit integers and divide by 100.0 to get real numbers
        if (rx_msg.data_length_code >= 2) {
            int16_t rawVal1 = rx_msg.data[0] | (rx_msg.data[1] << 8);
            float realVal1 = (float)rawVal1 / 100.0f;

            Serial.printf("[CAN RECEIVER] ID: 0x%X | Val 1: %.2f", rx_msg.identifier, realVal1);

            // If the message contains a second value in bytes 2 and 3
            if (rx_msg.data_length_code >= 4) {
                int16_t rawVal2 = rx_msg.data[2] | (rx_msg.data[3] << 8);
                float realVal2 = (float)rawVal2 / 100.0f;
                Serial.printf(" | Val 2: %.2f", realVal2);
            }

            Serial.println();
        }
    }
}