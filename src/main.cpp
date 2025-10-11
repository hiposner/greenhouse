#include <Arduino.h>

#include "GreenhouseConfig.h"
#include "GreenhouseLogic.h"

GreenhouseController greenhouse(greenhouseConfig);

void setup() {
    Serial.begin(115200);
    delay(100);
    greenhouse.begin();
}

void loop() {
    greenhouse.loop();
    delay(10);
}
