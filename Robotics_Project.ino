#include "MeMegaPi.h"

MeMegaPiDCMotor motor1(PORT1B); // Right Wheel
MeMegaPiDCMotor motor2(PORT2B); // Left Wheel
MeMegaPiDCMotor motor3(PORT3B); // Joint (Gripper Arm Movement)
MeMegaPiDCMotor motor4(PORT4B); // Gripper

MeUltrasonicSensor ultraSensor(PORT_7); // Ultrasonic sensor port

uint8_t motorSpeed = 80;     // Speed for forward movement
uint8_t armSpeed = 50;       // Slower speed for precise arm movement
uint16_t detectionDistance = 12.5; // Object detection distance

// To move forward: Motor1: negative, Motor2: positive
// To move arm: Pos: move up, Neg: move down
// Gripper: Pos: Open up, Neg: close down

// Function to initialize arm position
void initializeArmPosition() {
    Serial.println("Initializing Arm Position...");
    motor3.run(50);
    motor4.run(80);
    delay(6000);
    motor4.stop();

    // Slowly move the arm down for a fixed duration
    motor3.run(-50); // Controlled descent speed
    delay(3500);    // Move down for 2 seconds (adjust as needed)
    motor3.stop();
    

    Serial.println("Arm Initialized at Home Position");
}


void setup() {
    Serial.begin(9600);

    // Initialize arm to a known starting position
    initializeArmPosition();
}

void loop() {

    // Continuously move forward slowly
    motor1.run(-motorSpeed);
    motor2.run(motorSpeed);

    // Check for an object in front
    float distance = ultraSensor.distanceCm();

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance > 0 && distance <= detectionDistance) {
        // Stop motors to prepare for object pickup
        motor1.stop();
        motor2.stop();

        delay(500); // Pause before gripping process

        // Lower the gripper arm
        motor3.run(-50);  // Positive value to lower arm
        delay(800);      // Adjust time to lower arm sufficiently
        motor3.stop();

        // Close the gripper to grab the object
        motor4.run(-120);  // Positive value to close gripper
        delay(2500);      // Adjust for proper gripping
        motor4.stop();

        // Lift the arm back up
        motor3.run(100); // Negative value to raise arm
        delay(2500);      // Adjust to fully lift the arm
        motor3.stop();

        // Continue moving forward
        delay(500);  // Small pause to ensure the system is stable
        motor1.run(-motorSpeed);
        motor2.run(motorSpeed);
        delay(1000);
        motor1.stop();
        motor2.stop();

        //pause here
        delay(10000);
    }

    delay(100);  // Short delay to stabilize sensor readings
}
