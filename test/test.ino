#include "MeBluetooth.h"

MeBluetooth bt(PORT_4); // Try PORT_4 or PORT_5 first

void setup() {
    Serial.begin(9600);
    bt.begin(9600);  // Standard baud rate for Makeblock Bluetooth
    Serial.println("Bluetooth Ready");
}

void loop() {
    if (bt.available()) {
        Serial.write(bt.read());
    }
}
