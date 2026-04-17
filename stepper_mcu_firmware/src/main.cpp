#include <Arduino.h>

#include "stepper_server.h"

robot::StepperServer g_stepper_server;

void setup()
{
    g_stepper_server.begin();
}

void loop()
{
    g_stepper_server.loop();
}
