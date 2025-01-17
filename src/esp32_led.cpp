#include <esp32_led.hpp>

void led_task(void *parameter) {
    int *state = (int *)parameter;
    pinMode(ESP32_LED, OUTPUT);
    while (true) {
        switch (*state) {
            case WAITING_AGENT:
                digitalWrite(ESP32_LED, HIGH);
                vTaskDelay(125 / portTICK_PERIOD_MS);  // 0.25 ms period
                digitalWrite(ESP32_LED, LOW);
                vTaskDelay(125 / portTICK_PERIOD_MS);  // 0.25 ms period
                break;
            case AGENT_CONNECTED:
                digitalWrite(ESP32_LED, HIGH);
                break;
            default:
                digitalWrite(ESP32_LED, LOW);
                break;
        }
    }
}
