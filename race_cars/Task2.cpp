#include "Task2.h"

void Task2::setup() {
    // Create a task using a lambda function to call loop()
    xTaskCreatePinnedToCore([](void *parameter) {
        Task2 *task = (Task2 *)parameter;
        task->loop();
    }, "Task 2", 10000, this, 1, &taskHandle, 0);
}

void Task2::loop() {
    while (true) {
        Serial.println("Task 2 is running!");
        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Delay for 2 seconds
    }
}
