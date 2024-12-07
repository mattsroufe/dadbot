#ifndef TASK2_H
#define TASK2_H

#include <Arduino.h>

class Task2 {
public:
    void setup();
    void loop();

private:
    TaskHandle_t taskHandle;
};

#endif
