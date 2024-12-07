#include <Arduino.h>
#include "Task1.h"
#include "Task2.h"

Task1 task1;
Task2 task2;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial to be ready
  }

  // Start the tasks by calling their setup() method
  task1.setup();
  task2.setup();
}

void loop() {
  // The loop function is empty because tasks are running independently
}
