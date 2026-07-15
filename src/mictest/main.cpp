#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("[MICTEST] boot");
}

void loop() {
    delay(1000);
}
