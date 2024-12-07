// Pin definitions
const int enA = 25;    // PWM pin connected to enA
const int in1 = 26;    // Motor direction control
const int in2 = 27;    // Motor direction control

// Define PWM channel and frequency
const int pwmChannel = 0;   // Use channel 0
const int pwmFrequency = 20000; // 20 kHz
const int pwmResolution = 8;    // 8-bit resolution (0-255)

void setup() {
  // Set up motor control pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Set up PWM using ledc functions
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(enA, pwmChannel); // Attach PWM to pin

  // Example: Run motor forward at 50% speed
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  ledcWrite(pwmChannel, 128); // 50% duty cycle (128 out of 255)
}

void loop() {
  // You can adjust the motor speed dynamically
  // Example: Ramp up and down
  for (int duty = 0; duty <= 255; duty += 5) {
    ledcWrite(pwmChannel, duty);
    delay(50);
  }
  for (int duty = 255; duty >= 0; duty -= 5) {
    ledcWrite(pwmChannel, duty);
    delay(50);
  }
}
