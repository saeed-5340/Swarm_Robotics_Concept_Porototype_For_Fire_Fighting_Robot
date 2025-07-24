 //----------------------------Saeed_Ahamed_Mridha----------------------------------------------------------
 
 #include <ESP32Servo.h>  // Include the ESP32Servo library to control servos on ESP32

// Define the pin number that will be used to send the control signal to the servo motor
#define servo_control_pin 15  // GPIO 15 will be used for servo signal

Servo servo;  // Create a Servo object called 'servo' to control the motor

void setup() {
  Serial.begin(115200);  // Initialize serial communication at 115200 baud (for debugging or monitoring)

  servo.attach(servo_control_pin);  // Attach the servo object to the defined control pin (GPIO 15)

  servo.write(0);        // Set the initial position of the servo to 0 degrees
  delay(1000);           // Wait for 1 second to let the servo reach the initial position
}

void loop() {
  // Move the servo from 0 to 180 degrees in steps of 1 degree
  for (int i = 0; i <= 180; i++) {  // Start from 0 degrees and increment to 180 degrees
    servo.write(i);        // Send the current angle to the servo
    delay(20);             // Wait for 20 milliseconds to allow the servo to move smoothly
  }

  // Move the servo from 180 back to 0 degrees
  for (int i = 180; i >= 0; i--) {  // Start from 180 degrees and decrement to 0
    servo.write(i);        // Send the current angle to the servo
    delay(20);             // Wait for 20 milliseconds to allow the servo to move smoothly
  }
}