#include <Arduino.h>

#include "acclgyr.h"

void setup() {
  AcclGyr::begin();
}

void loop() {
  AcclGyr::loop();
}

