#ifndef TASK1_H
#define TASK1_H

#include "BaseTask.h"

class Task1 : public BaseTask {
public:
    Task1() : BaseTask("Task1", 1000, 1, 0) {}

protected:
    void loop() override {
        Serial.println("Task1 loop");
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
    }
};

#endif
