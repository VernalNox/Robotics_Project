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

// Function to initialize arm position
void initializeArmPosition() {
    Serial.println("Initializing Arm Position...");
    motor3.run(50);
    motor4.run(80);
    delay(6000);
    motor4.stop();

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

// Linear Delay Function
int calculateCenteringDelay(float detectedDistance) {
    const float m = -6.23;      // Slope value — tune this for desired behavior
    const float b = 583;       // Base delay when the object is closest
    const float minDelay = 370; // Minimum delay cap
    const float maxDelay = 435; // Maximum delay cap

    // Linear delay calculation with bounds
    float delayValue = m * detectedDistance + b;
    delayValue = constrain(delayValue, minDelay, maxDelay);  // Ensure delay stays within range

    return (int)delayValue;  // Convert to integer for delay function
}

// Function that executes the object pickup sequence
void objectFound() {
    motor1.stop();
    motor2.stop();

    delay(500); // Pause before gripping process

    motor3.run(-50);  
    delay(1000);       
    motor3.stop();

    motor4.run(-140);  
    delay(2800);       
    motor4.stop();

    motor3.run(100); 
    delay(2500);     
    motor3.stop();

    delay(500);
    motor1.run(-motorSpeed);
    motor2.run(motorSpeed);
    delay(1000);
    motor1.stop();
    motor2.stop();

    delay(10000);
}

// Function to move toward the detected object
void moveTowardObject() {
    Serial.println("Moving Toward Object...");

    motor1.run(-motorSpeed / 2);
    motor2.run(motorSpeed / 2);

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

    Serial.println("Scanning Left...");
    motor1.run(rotationSpeed);
    motor2.run(rotationSpeed);

    unsigned long startTime = millis();
    while (millis() - startTime < 800) {
        distance = getAverageDistance();
        Serial.println(distance);
        if (distance > 0 && distance <= detectionDistance) {
            Serial.println("Object Detected (Left Side)!");
            int alignDelay = calculateCenteringDelay(distance);
            delay(alignDelay);
            motor1.stop();
            motor2.stop();
            Serial.println(alignDelay);
            moveTowardObject();
            return true;
        }
    }

    motor1.run(-rotationSpeed);
    motor2.run(-rotationSpeed);
    delay(800);

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

    motor1.run(rotationSpeed);
    motor2.run(rotationSpeed);
    delay(800);

    motor1.stop();
    motor2.stop();
    delay(50);

    return false;
}

void setup() {
    Serial.begin(9600);
    initializeArmPosition();
}

void loop() {
    motor1.run(-motorSpeed);
    motor2.run(motorSpeed);
    delay(750);

    float distance = ultraSensor.distanceCm();

    if (distance > 0 && distance <= detectionDistance) {
        Serial.println("Object Detected (Front)!");
        moveTowardObject();
    } else {
        if (scanForObjects()) {
            delay(1000);
        }
    }

    delay(10); 
}
