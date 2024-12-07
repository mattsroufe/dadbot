#ifndef BASETASK_H
#define BASETASK_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class BaseTask {
public:
    BaseTask(const char* taskName, uint32_t stackSize, UBaseType_t priority, BaseType_t core)
        : taskName(taskName), stackSize(stackSize), priority(priority), core(core), taskHandle(nullptr) {}

    virtual ~BaseTask() {}

    // Function to create and start the task
    void setup() {
        xTaskCreatePinnedToCore(
            taskFunctionWrapper,  // Static task function
            taskName,             // Name of the task
            stackSize,            // Stack size
            this,                 // Pass the current object as a parameter
            priority,             // Task priority
            &taskHandle,          // Task handle to control the task
            core                  // Core where the task should run
        );
    }

protected:
    // Task handle for managing the task
    TaskHandle_t taskHandle;

private:
    const char* taskName;
    uint32_t stackSize;
    UBaseType_t priority;
    BaseType_t core;

    // Static task function wrapper
    static void taskFunctionWrapper(void* parameter) {
        BaseTask* task = static_cast<BaseTask*>(parameter);
        task->loop();  // Call the derived class's loop() implementation
    }

    // Virtual method to be implemented by derived classes
    virtual void loop() = 0;
};

#endif
