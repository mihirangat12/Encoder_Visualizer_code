
#include <Arduino.h>
#include <ESP32Encoder.h>
#include "driver/twai.h"

// ==========================================
// ENCODER PINS - ESP32 WROOM-1U
// ==========================================
#define ENCODER_A GPIO_NUM_4
#define ENCODER_B GPIO_NUM_5

// ==========================================
// CAN PINS - ESP32 WROOM-1U
// ==========================================
#define CAN_TX_PIN GPIO_NUM_17
#define CAN_RX_PIN GPIO_NUM_18

#define CAN_COMMENT_ID 0x200
#define CAN_RESULT_ID  0x201

// ==========================================
// MEASUREMENT CONFIGURATION
// ==========================================
const float ROLLER_DIAMETER_MM = 100.0f;
const float COUNTS_PER_REVOLUTION = 400.0f;
const float MM_PER_INCH = 25.4f;

ESP32Encoder encoder;

// ==========================================
// DYNAMIC MARKER STORAGE
// Marker pairs are received from touchscreen
// ==========================================
struct MarkerPair {
    float start;
    float end;
    uint32_t id;
};

const int MAX_MARKERS = 100;

MarkerPair markers[MAX_MARKERS];
int markerCount = 0;

bool canReady = false;

// ==========================================
// STORE OR UPDATE MARKER
// ==========================================
void storeMarker(uint32_t id, float start, float end) {

    // Update an existing marker if its ID matches
    for (int i = 0; i < markerCount; i++) {

        if (markers[i].id == id) {

            markers[i].start = start;
            markers[i].end = end;

            Serial.printf(
                "Marker updated: ID=%lu, (%.2f, %.2f)\n",
                (unsigned long)id,
                start,
                end
            );

            return;
        }
    }

    // Add a new marker
    if (markerCount < MAX_MARKERS) {

        markers[markerCount].id = id;
        markers[markerCount].start = start;
        markers[markerCount].end = end;

        markerCount++;

        Serial.printf(
            "Marker received: ID=%lu, (%.2f, %.2f)\n",
            (unsigned long)id,
            start,
            end
        );

    } else {
        Serial.println("Marker storage full!");
    }
}

// ==========================================
// RECEIVE MARKERS FROM TOUCHSCREEN
//
// CAN ID: 0x200
// Bytes 0-1: start * 100
// Bytes 2-3: end * 100
// Bytes 4-7: marker ID
//
// All values use little-endian encoding.
// ==========================================
void receiveMarkerMessages() {

    twai_message_t msg;

    while (twai_receive(&msg, 0) == ESP_OK) {

        // Accept standard data frames only
        if (msg.extd || msg.rtr) {
            continue;
        }

        if (msg.identifier != CAN_COMMENT_ID) {
            continue;
        }

        if (msg.data_length_code != 8) {
            Serial.println("Invalid marker packet length");
            continue;
        }

        // Decode start position
        uint16_t rawStart =
            (uint16_t)msg.data[0] |
            ((uint16_t)msg.data[1] << 8);

        // Decode end position
        uint16_t rawEnd =
            (uint16_t)msg.data[2] |
            ((uint16_t)msg.data[3] << 8);

        // Decode marker ID
        uint32_t id =
            (uint32_t)msg.data[4] |
            ((uint32_t)msg.data[5] << 8) |
            ((uint32_t)msg.data[6] << 16) |
            ((uint32_t)msg.data[7] << 24);

        // Convert encoded values back to inches
        float start = rawStart / 100.0f;
        float end   = rawEnd / 100.0f;

        storeMarker(id, start, end);
    }
}

// ==========================================
// SELECT SUITABLE MARKER
//
// NEW LOGIC:
//
// Select the marker with the largest END value
// that does not exceed the measured length.
//
// Examples:
// Length 50  -> (42, 48)
// Length 60  -> (42, 48)
// Length 75  -> (42, 48)
// Length 85  -> (70, 82)
// Length 105 -> (70, 82)
// Length 115 -> (104, 110)
// ==========================================
MarkerPair findSuitableMarker(float lengthInches) {

    MarkerPair best = {0.0f, 0.0f, 0};

    float bestEnd = -1.0f;
    bool found = false;

    for (int i = 0; i < markerCount; i++) {

        // The marker's end must be reached
        if (markers[i].end <= lengthInches) {

            // Choose the largest eligible end position
            if (!found || markers[i].end > bestEnd) {

                best = markers[i];
                bestEnd = markers[i].end;
                found = true;
            }
        }
    }

    // No eligible marker was found
    if (!found) {
        return {0.0f, 0.0f, 0};
    }

    return best;
}

// ==========================================
// SEND MEASUREMENT + SELECTED MARKER
//
// CAN ID: 0x201
// Bytes 0-1: measured length in millimetres
// Bytes 2-3: selected start * 100
// Bytes 4-5: selected end * 100
//
// All values are little-endian.
// ==========================================
void sendMeasurement(
    uint16_t lengthMM,
    MarkerPair marker
) {

    if (!canReady) {
        return;
    }

    twai_message_t msg = {};

    msg.identifier = CAN_RESULT_ID;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 6;

    uint16_t values[3] = {

        lengthMM,

        (uint16_t)(
            marker.start * 100.0f + 0.5f
        ),

        (uint16_t)(
            marker.end * 100.0f + 0.5f
        )
    };

    for (int i = 0; i < 3; i++) {

        msg.data[i * 2] =
            values[i] & 0xFF;

        msg.data[i * 2 + 1] =
            (values[i] >> 8) & 0xFF;
    }

    if (twai_transmit(
            &msg,
            pdMS_TO_TICKS(20)
        ) != ESP_OK) {

        Serial.println(
            "CAN result transmission failed"
        );
    }
}

// ==========================================
// SETUP
// ==========================================
void setup() {

    Serial.begin(115200);
    delay(1000);

    // Configure encoder
    ESP32Encoder::useInternalWeakPullResistors =
        puType::up;

    encoder.attachFullQuad(
        ENCODER_A,
        ENCODER_B
    );

    encoder.clearCount();

    // Configure CAN
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

    // Install CAN driver
    esp_err_t err = twai_driver_install(
        &g_config,
        &t_config,
        &f_config
    );

    if (err != ESP_OK) {

        Serial.printf(
            "CAN install failed: %d\n",
            (int)err
        );

        return;
    }

    // Start CAN
    err = twai_start();

    if (err != ESP_OK) {

        Serial.printf(
            "CAN start failed: %d\n",
            (int)err
        );

        twai_driver_uninstall();

        return;
    }

    canReady = true;

    Serial.println(
        "Encoder + dynamic CAN markers ready."
    );

    Serial.println(
        "Waiting for marker pairs from touchscreen..."
    );

    Serial.println(
        "Marker selection uses END position."
    );
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {

    // 1. Receive new marker pairs from touchscreen
    receiveMarkerMessages();

    // 2. Read encoder count
    int64_t counts = encoder.getCount();

    // 3. Convert encoder counts into millimetres
    float measuredMM =
        ((float)counts / COUNTS_PER_REVOLUTION) *
        (PI * ROLLER_DIAMETER_MM);
/*
    // Prevent negative fabric length
    if (measuredMM < 0.0f) {
        measuredMM = 0.0f;
    }
*/
    // 4. Convert millimetres into inches
    float measuredInches =
        measuredMM / MM_PER_INCH;

    // 5. Prepare measurement for CAN transmission
    uint16_t lengthMM =
        (uint16_t)(measuredMM + 0.5f);

    // 6. Select marker using the new END-position logic
    MarkerPair selected =
        findSuitableMarker(measuredInches);

    // 7. Print current measurement and selected marker
    Serial.printf(
        "Length: %.2f in | "
        "Markers: %d | "
        "Selected: (%.2f, %.2f) | "
        "Marker ID: %lu\n",

        measuredInches,
        markerCount,
        selected.start,
        selected.end,
        (unsigned long)selected.id
    );

    // 8. Send measurement and selected marker to touchscreen
    sendMeasurement(
        lengthMM,
        selected
    );

    delay(200);
}
