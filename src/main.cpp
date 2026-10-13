
#include <Arduino.h>
#include <ESP32Encoder.h>
#include "driver/twai.h"

// ==========================================
// ENCODER CONFIGURATION
// ==========================================
#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 5

const long COUNTS_PER_REVOLUTION = 400;
const float WHEEL_DIAMETER_MM = 100.0f;

const float WHEEL_CIRCUMFERENCE_M =
    (PI * WHEEL_DIAMETER_MM) / 1000.0f;

ESP32Encoder encoder;

// ==========================================
// CAN CONFIGURATION
// ==========================================
#define CAN_TX_PIN GPIO_NUM_17
#define CAN_RX_PIN GPIO_NUM_16

#define CAN_COMMENT_ID  0x200
#define CAN_RESULT_ID   0x201
#define CAN_STATUS_ID   0x202

bool canReady = false;

// ==========================================
// FABRIC COMMENT STORAGE
// ==========================================
const int MAX_COMMENTS = 20;

struct FabricComment {
    uint32_t id;
    uint16_t startCm;
    uint16_t endCm;
    bool valid;
};

FabricComment comments[MAX_COMMENTS];

uint32_t selectedCommentId = 0;
uint8_t matchStatus = 0;

// Status:
// 0 = No comments received
// 1 = Current length is inside selected range
// 2 = Nearest range selected, outside its range

// ==========================================
// ENCODER VARIABLES
// ==========================================
long encoderPosition = 0;
float wheelRevolutions = 0.0f;
float fabricLengthMeters = 0.0f;

unsigned long lastPrintTime = 0;
unsigned long lastSendTime = 0;

const unsigned long PRINT_INTERVAL_MS = 200;
const unsigned long SEND_INTERVAL_MS = 500;

// ==========================================
// READ / WRITE 32-BIT LITTLE-ENDIAN VALUES
// ==========================================
uint32_t readUint32(const uint8_t *data) {
    return (uint32_t)data[0]
         | ((uint32_t)data[1] << 8)
         | ((uint32_t)data[2] << 16)
         | ((uint32_t)data[3] << 24);
}

void writeUint32(uint8_t *data, uint32_t value) {
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
    data[2] = (value >> 16) & 0xFF;
    data[3] = (value >> 24) & 0xFF;
}

// ==========================================
// INITIALIZE CAN
// ==========================================
void setupCAN() {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(
            CAN_TX_PIN,
            CAN_RX_PIN,
            TWAI_MODE_NORMAL
        );

    twai_timing_config_t t_config =
        TWAI_TIMING_CONFIG_500KBITS();

    twai_filter_config_t f_config =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(
            &g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[CAN] Driver installation failed");
        return;
    }

    if (twai_start() != ESP_OK) {
        Serial.println("[CAN] Start failed");
        twai_driver_uninstall();
        return;
    }

    canReady = true;
    Serial.println("[CAN] Ready at 500 kbps");
}

// ==========================================
// RECEIVE FABRIC COMMENTS FROM TOUCH LCD
// CAN ID: 0x200
//
// Byte 0-1: Start length in cm
// Byte 2-3: End length in cm
// Byte 4-7: Comment ID
// ==========================================
void receiveFabricComments() {
    if (!canReady) return;

    twai_message_t rx;

    // Read all currently queued messages
    while (twai_receive(&rx, 0) == ESP_OK) {

        if (rx.extd || rx.rtr) continue;

        if (rx.identifier != CAN_COMMENT_ID ||
            rx.data_length_code != 8) {
            continue;
        }

        uint16_t startCm =
            (uint16_t)rx.data[0] |
            ((uint16_t)rx.data[1] << 8);

        uint16_t endCm =
            (uint16_t)rx.data[2] |
            ((uint16_t)rx.data[3] << 8);

        uint32_t commentId = readUint32(&rx.data[4]);

        if (commentId == 0 ||
            startCm > endCm) {
            Serial.println("[COMMENT] Invalid range");
            continue;
        }

        int slot = -1;

        // Update an existing comment with the same ID
        for (int i = 0; i < MAX_COMMENTS; i++) {
            if (comments[i].valid &&
                comments[i].id == commentId) {
                slot = i;
                break;
            }
        }

        // Otherwise use an empty slot
        if (slot == -1) {
            for (int i = 0; i < MAX_COMMENTS; i++) {
                if (!comments[i].valid) {
                    slot = i;
                    break;
                }
            }
        }

        if (slot == -1) {
            Serial.println("[COMMENT] Storage full");
            continue;
        }

        comments[slot].id = commentId;
        comments[slot].startCm = startCm;
        comments[slot].endCm = endCm;
        comments[slot].valid = true;

        Serial.printf(
            "[COMMENT] ID=%lu Range=%.2f-%.2f m\n",
            (unsigned long)commentId,
            startCm / 100.0f,
            endCm / 100.0f
        );
    }
}

// ==========================================
// FIND THE MOST SUITABLE FABRIC COMMENT
//
// Priority:
// 1. If length is inside one or more ranges,
//    choose the narrowest matching range.
// 2. If no range contains the length,
//    choose the range with the nearest boundary.
// ==========================================
void identifyBestComment() {
    float currentCm = fabricLengthMeters * 100.0f;

    int bestInside = -1;
    int bestNearest = -1;

    uint32_t narrowestWidth = UINT32_MAX;
    float nearestDistance = 1.0e30f;

    for (int i = 0; i < MAX_COMMENTS; i++) {

        if (!comments[i].valid) continue;

        float start = comments[i].startCm;
        float end = comments[i].endCm;

        if (currentCm >= start && currentCm <= end) {

            uint32_t width =
                comments[i].endCm - comments[i].startCm;

            if (bestInside == -1 ||
                width < narrowestWidth) {
                bestInside = i;
                narrowestWidth = width;
            }

        } else {

            float distance;

            if (currentCm < start) {
                distance = start - currentCm;
            } else {
                distance = currentCm - end;
            }

            if (bestNearest == -1 ||
                distance < nearestDistance) {
                bestNearest = i;
                nearestDistance = distance;
            }
        }
    }

    if (bestInside != -1) {
        selectedCommentId = comments[bestInside].id;
        matchStatus = 1;
    }
    else if (bestNearest != -1) {
        selectedCommentId = comments[bestNearest].id;
        matchStatus = 2;
    }
    else {
        selectedCommentId = 0;
        matchStatus = 0;
    }
}

// ==========================================
// SEND RESULT TO TOUCH LCD
// CAN ID: 0x201
//
// Byte 0-3: Selected comment ID
// Byte 4-7: Current length in centimetres
// ==========================================
void sendResultToLCD() {
    if (!canReady) return;

    twai_message_t tx = {};

    tx.identifier = CAN_RESULT_ID;
    tx.extd = 0;
    tx.rtr = 0;
    tx.data_length_code = 8;

    uint32_t lengthCm = 0;

    if (fabricLengthMeters > 0.0f) {
        lengthCm =
            (uint32_t)(fabricLengthMeters * 100.0f + 0.5f);
    }

    writeUint32(&tx.data[0], selectedCommentId);
    writeUint32(&tx.data[4], lengthCm);

    if (twai_transmit(&tx, pdMS_TO_TICKS(20)) != ESP_OK) {
        Serial.println("[CAN] Result transmission failed");
    }

    // Send match status separately
    twai_message_t statusTx = {};

    statusTx.identifier = CAN_STATUS_ID;
    statusTx.extd = 0;
    statusTx.rtr = 0;
    statusTx.data_length_code = 1;
    statusTx.data[0] = matchStatus;

    twai_transmit(&statusTx, pdMS_TO_TICKS(20));
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("==================================");
    Serial.println(" Fabric Comment Matching System");
    Serial.println(" ESP32-S3-WROOM-1U");
    Serial.println("==================================");

    ESP32Encoder::useInternalWeakPullResistors =
        puType::up;

    encoder.attachFullQuad(
        ENCODER_A_PIN,
        ENCODER_B_PIN
    );

    encoder.clearCount();

    for (int i = 0; i < MAX_COMMENTS; i++) {
        comments[i].valid = false;
    }

    setupCAN();

    Serial.println("Encoder initialized.");
    Serial.println("Waiting for fabric comments...");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {

    // 1. Read encoder
    encoderPosition = encoder.getCount();

    wheelRevolutions =
        (float)encoderPosition / COUNTS_PER_REVOLUTION;

    fabricLengthMeters =
        wheelRevolutions * WHEEL_CIRCUMFERENCE_M;

    // 2. Receive comments from the Touch LCD
    receiveFabricComments();

    // 3. Find the best range for current length
    identifyBestComment();

    // 4. Print results every 200 ms
    unsigned long now = millis();

    if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = now;

        Serial.printf(
            "Counts: %ld | Length: %.2f m | ",
            encoderPosition,
            fabricLengthMeters
        );

        if (matchStatus == 1) {
            Serial.printf(
                "MATCH: ID=%lu\n",
                (unsigned long)selectedCommentId
            );
        }
        else if (matchStatus == 2) {
            Serial.printf(
                "NEAREST: ID=%lu (outside range)\n",
                (unsigned long)selectedCommentId
            );
        }
        else {
            Serial.println("No fabric comments available");
        }
    }

    // 5. Send updated result to the Touch LCD
    if (now - lastSendTime >= SEND_INTERVAL_MS) {
        lastSendTime = now;
        sendResultToLCD();
    }

    delay(1);
}
