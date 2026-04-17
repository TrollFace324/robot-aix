#include <Arduino.h>

#include "app/system_controller.h"

robot::app::SystemController g_controller;

void setup()
{
    g_controller.begin();
}

void loop()
{
    g_controller.loop();
}
