#include <Arduino.h>

#include "acclgyr.h"
#include "drone_controller.h"
#include "optical_flow_test.h"

void setup() {
  AcclGyr::begin();
  OpticalFlowTest::begin();
  DroneController::begin();
}

void loop() {
  AcclGyr::loop();
  OpticalFlowTest::loop();
  DroneController::loop();
}

