#ifndef TASK2_H
#define TASK2_H

#include "BaseTask.h"

class Task2 : public BaseTask {
public:
    Task2() : BaseTask("Task2", 1000, 1, 1) {}

protected:
    void loop() override {
        Serial.println("Task2 loop");
        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Delay for 2 seconds
    }
};

#endif
