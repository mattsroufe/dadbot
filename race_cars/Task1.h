#ifndef TASK1_H
#define TASK1_H

#include <Arduino.h>

class Task1 {
public:
    void setup();
    void loop();

private:
    TaskHandle_t taskHandle;
};

#endif
