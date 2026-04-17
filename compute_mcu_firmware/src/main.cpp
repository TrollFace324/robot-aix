#include <Arduino.h>

#include "app/compute_controller.h"

robot::app::ComputeController g_compute;

void setup()
{
    g_compute.begin();
}

void loop()
{
    g_compute.loop();
}
