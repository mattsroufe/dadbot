#include "Task1.h"

void Task1::setup() {
    // Create a task using a lambda function to call loop()
    xTaskCreatePinnedToCore([](void *parameter) {
        Task1 *task = (Task1 *)parameter;
        task->loop();
    }, "Task 1", 10000, this, 1, &taskHandle, 0);
}

void Task1::loop() {
    while (true) {
        Serial.println("Task 1 is running!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
    }
}
