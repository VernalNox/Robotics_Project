#include "MeMegaPi.h"

MeMegaPiDCMotor motor1(PORT1B); // Right Wheel
MeMegaPiDCMotor motor2(PORT2B); // Left Wheel
MeMegaPiDCMotor motor3(PORT3B); // Joint (Gripper Arm Movement)
MeMegaPiDCMotor motor4(PORT4B); // Gripper

MeUltrasonicSensor ultraSensor(PORT_7); // Ultrasonic sensor port

uint8_t motorSpeed = 90;     // Speed for forward movement
uint8_t rotationSpeed = 77; // Speed for rotating in place
uint8_t armSpeed = 50;       // Slower speed for precise arm movement
uint16_t detectionDistance = 32; // Object detection distance
uint16_t grippingDistance = 13; // Object close enough to grip


// BEST FOUND DELAY: 26.53CM -- 450ms

// Function to initialize arm position
void initializeArmPosition() {
    Serial.println("Initializing Arm Position...");
    motor3.run(50);
    motor4.run(80);
    delay(6000);
    motor4.stop();

    // Slowly move the arm down for a fixed duration
    motor3.run(-50); // Controlled descent speed
    delay(3500);    
    motor3.stop();

    Serial.println("Arm Initialized at Home Position");
}

float getAverageDistance(int samples = 5) {
    float total = 0;
    for (int i = 0; i < samples; i++) {
        total += ultraSensor.distanceCm();
        delay(2);  // Delay for stable readings
    }
    return total / samples;
}

// Bell curve delay function
int calculateCenteringDelay(float detectedDistance) {
    const float D_max = 435;     // Maximum delay in milliseconds
    const float d_optimal = 26.53;  // Sweet spot distance in cm
    const float sigma = 7;       // Spread factor for bell curve shape

    // Bell curve calculation
    float delayValue = D_max * exp(-pow(detectedDistance - d_optimal, 2) / (2 * pow(sigma, 2)));

    return (int)delayValue;  // Convert to integer for delay function
}

// Function that executes the object pickup sequence
void objectFound() {

    motor1.stop();
    motor2.stop();

    delay(500); // Pause before gripping process

    // Lower the gripper arm
    motor3.run(-50);  
    delay(1000);       
    motor3.stop();

    // Close the gripper to grab the object
    motor4.run(-140);  
    delay(2800);       
    motor4.stop();

    // Lift the arm back up
    motor3.run(100); 
    delay(2500);     
    motor3.stop();

    // Continue moving forward
    delay(500);
    motor1.run(-motorSpeed);
    motor2.run(motorSpeed);
    delay(1000);
    motor1.stop();
    motor2.stop();

    // Pause here
    delay(10000);
}


// Function to move toward the detected object
void moveTowardObject() {
    Serial.println("Moving Toward Object...");

    motor1.run(-motorSpeed/2);
    motor2.run(motorSpeed/2);

    // Continue moving until object is 14 cm away
    while (true) {
        float distance = getAverageDistance();
      

        if (distance > 0 && distance <= grippingDistance) {
            Serial.println("Object Reached! Engaging Pickup Process...");
            motor1.stop();
            motor2.stop();
            objectFound();
            return;
        }

    }
}

// Function to rotate and scan for objects
bool scanForObjects() {
    float distance;

    // Rotate Left while scanning
    Serial.println("Scanning Left...");
    motor1.run(rotationSpeed);
    motor2.run(rotationSpeed);  // Both motors forward to turn left

    unsigned long startTime = millis();  // Track how long the scan runs
    while (millis() - startTime < 800) { // Scan for 800ms (adjust for desired turn angle)
        distance = getAverageDistance();
        Serial.println(distance);
        if (distance > 0 && distance <= detectionDistance) {
            Serial.println("Object Detected (Left Side)!");
            int alignDelay = calculateCenteringDelay(distance);
            delay(alignDelay);
            //delay(400);
            motor1.stop();
            motor2.stop();
            Serial.println(alignDelay);
            moveTowardObject();
            return true;
        }

    }

    // Return to Center
    motor1.run(-rotationSpeed);
    motor2.run(-rotationSpeed);
    delay(800); // Return to the original position

    // Rotate Right
    Serial.println("Scanning Right...");
    motor1.run(-rotationSpeed);
    motor2.run(-rotationSpeed);

    startTime = millis();
    while (millis() - startTime < 800) {
        distance = getAverageDistance();
        Serial.println(distance);
        if (distance > 0 && distance <= detectionDistance) {
            Serial.println("Object Detected (Right Side)!");
            int alignDelay = calculateCenteringDelay(distance);
            delay(alignDelay);
            motor1.stop();
            motor2.stop();
            Serial.println(alignDelay);
            moveTowardObject();
            return true;
        }
    }

    // Return to Center
    motor1.run(rotationSpeed);
    motor2.run(rotationSpeed);
    delay(800);

    // Stop momentarily
    motor1.stop();
    motor2.stop();
    delay(50);

    return false; // No object detected during scanning
}

void setup() {
    Serial.begin(9600);


// Initialize arm to a known starting position
    initializeArmPosition();
}

void loop() {
    // Continuously move forward
    motor1.run(-motorSpeed);
    motor2.run(motorSpeed);
    delay(750);

    // Check for an object directly in front
    float distance = ultraSensor.distanceCm();

    if (distance > 0 && distance <= detectionDistance) {
        Serial.println("Object Detected (Front)!");
        moveTowardObject();
    } else {
        // Periodically scan the surroundings
        if (scanForObjects()) {
            // If object is found during scanning, stop movement temporarily
            delay(1000);
        }
    }

    delay(10); // Short delay to stabilize sensor readings
}