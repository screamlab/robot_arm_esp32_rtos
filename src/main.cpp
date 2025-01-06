#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "armDriver.h"

#define UPDATE_ARM_DELAY 1.0
#define SERIAL_READ_DELAY 10
#define SERIAL_WRITE_DELAY 250
const uint8_t NUM_OF_SERVOS = 11;
const uint8_t servoMinAngles[] = {0, 80, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t servoMaxAngles[] = {180, 180, 180, 120, 180, 90, 130, 130, 130, 130, 130};
const uint8_t servoInitAngles[] = {10, 170, 80, 60, 80, 10, 80, 80, 80, 80, 80};

// Enum to define the arm actions
// enum class ArmAction {
//     NO_ACTION,
//     TURN_BASE_LEFT,
//     TURN_BASE_RIGHT,
//     MOVE_ARM_UP,
//     MOVE_ARM_DOWN,
//     SHIFT_ARM_RIGHT,
//     SHIFT_ARM_LEFT,
//     ROTATE_FINGER_RIGHT,
//     ROTATE_FINGER_LEFT
// };

// ArmAction armAction = ArmAction::NO_ACTION;
float currentAngles[NUM_OF_SERVOS];
uint8_t targetAngles[NUM_OF_SERVOS];

// Define task handles
TaskHandle_t armControlTask;
TaskHandle_t serialCommunicationTask;
TaskHandle_t serialWriterTask;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.

// Function declarations
void armControlTaskFunction(void *parameter);

void serialCommunicationTaskFunction(void *parameter);

void serialWriterTaskFunction(void *parameter);

void setup() {
    // Start serial communication
    Serial.begin(115200);

    // Create the arm control task
    for (uint8_t i = 0; i < NUM_OF_SERVOS; i++) {
        currentAngles[i] = (float)servoMinAngles[i];
        targetAngles[i] = servoInitAngles[i];
    }
    // Create the arm control task
    Serial.println("xTaskCreatePinnedToCore armControlTaskFunction ");
    delay(100);
    xTaskCreatePinnedToCore(
        armControlTaskFunction,  // Task function
        "Arm Control Task",      // Task name
        2048,                    // Stack size (in bytes)
        NULL,                    // Task parameters
        1,                       // Task priority
        &armControlTask,         // Task handle
        0                        // Core ID (0 or 1)
    );

    // Create the serial communication task
    Serial.println("xTaskCreatePinnedToCore serialCommunicationTaskFunction ");
    delay(100);
    xTaskCreatePinnedToCore(
        serialCommunicationTaskFunction,  // Task function
        "Serial Communication Task",      // Task name
        2048,                             // Stack size (in bytes)
        NULL,                             // Task parameters
        3,                                // Task priority
        &serialCommunicationTask,         // Task handle
        1                                 // Core ID (0 or 1)
    );
    // Create the serial communication task
    Serial.println("xTaskCreatePinnedToCore serialWriterTaskFunction ");
    delay(100);
    xTaskCreatePinnedToCore(
        serialWriterTaskFunction,  // Task function
        "Serial Writer Task",      // Task name
        2048,                      // Stack size (in bytes)
        NULL,                      // Task parameters
        2,                         // Task priority
        &serialWriterTask,         // Task handle
        0                          // Core ID (0 or 1)
    );
    delay(100);
}

void loop() {
    // Empty. Tasks are running in the background.
}

// Task function for arm control
void armControlTaskFunction(void *parameter) {
    // Create an instance of ArmManager

    // uint8_t currentAngles[ArmManager::NUM_SERVOS];
    ArmManager armManager(NUM_OF_SERVOS, servoMinAngles, servoMaxAngles, servoInitAngles);

    Serial.println("armControlTaskFunction start");

    for (;;) {
        // Control the robot arm using ArmManager
        for (uint8_t i = 0; i < NUM_OF_SERVOS; i++) {
            armManager.setServoTargetAngle(i, targetAngles[i]);
        }

        armManager.moveArm();
        // armManager.printStatus();
        armManager.getCurrentAngles(currentAngles);
        // armAction = ArmAction::NO_ACTION;

        // Wait for some time before the next iteration
        vTaskDelay(UPDATE_ARM_DELAY / portTICK_PERIOD_MS);
    }
}

// Task function for serial reader
void serialCommunicationTaskFunction(void *parameter) {
    Serial.println("serialCommunicationTaskFunction start ");
    String jsonString = "";
    for (;;) {
        if (Serial.available()) {
            // Read the incoming serial data
            jsonString = Serial.readStringUntil('\n');
            // String jsonString = Serial.readString();
            Serial.println("receive:");
            Serial.println(jsonString);
            // Parse the JSON string
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, jsonString);

            // Check if the JSON parsing was successful
            if (error) {
                Serial.print("JSON parsing error: ");
                Serial.println(error.c_str());
            } else {
                // Check if the parsed JSON document contains the "servo_target_angles" array
                if (doc.containsKey("servo_target_angles")) {
                    // Get the "servo_target_angles" array from the JSON document
                    JsonArray servoAngles = doc["servo_target_angles"].as<JsonArray>();

                    // Check if the array size matches the number of servos
                    if (servoAngles.size() == NUM_OF_SERVOS) {
                        // Update the target angles in the ArmManager
                        for (uint8_t i = 0; i < NUM_OF_SERVOS; i++) {
                            targetAngles[i] = servoAngles[i];
                        }
                    } else {
                        Serial.println("Invalid number of servo target angles.");
                    }
                } else {
                    Serial.println("Missing servo_target_angles in JSON");
                }
            }
        }

        // Wait for some time before the next iteration
        vTaskDelay(SERIAL_READ_DELAY / portTICK_PERIOD_MS);
    }
}

// Task function for serial writer
void serialWriterTaskFunction(void *parameter) {
    Serial.println("serialWriterTaskFunction start");

    for (;;) {
        StaticJsonDocument<512> doc;
        // Populate the JSON document with the current angles
        JsonArray servoAngles = doc.createNestedArray("servo_current_angles");
        // armManager.getCurrentAngles(currentAngles);

        for (uint8_t i = 0; i < NUM_OF_SERVOS; i++) {
            servoAngles.add(currentAngles[i]);
        }

        // Serialize the JSON document to a string
        String jsonString;
        serializeJson(doc, jsonString);

        // Print the JSON string
        Serial.println(jsonString);

        // Read incoming serial data

        // Wait for some time before the next iteration
        vTaskDelay(SERIAL_WRITE_DELAY / portTICK_PERIOD_MS);
    }
}
