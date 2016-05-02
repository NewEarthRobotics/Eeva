// Includes
#include <cmath>
#include "globs.h"
#include "main_control_task.h"
#include "physical_constants.h"
#include "robot_settings.h"
#include "util_assert.h"

//******************************************************************************
void MainControlTask::experiment3Mode(float experiment_input)
{
    if (modes_.state != STATE_NORMAL)
    {
        return;
    }

    // Open loop voltage control (simply put commanded voltage across the motors)
    motors_.left_voltage = experiment_input;
    motors_.right_voltage = experiment_input;

    // Need to 'reverse' the scaling that's done when calculating the PWM values
    // so that an input of 1 volt will actually end up being 1 volt.
    float scale = 1;
    if (CRITICAL_BATTERY_VOLTAGE <= analog_.battery_voltage)
    {
        scale = analog_.battery_voltage / CRITICAL_BATTERY_VOLTAGE;
    }
    motors_.left_voltage *= scale;
    motors_.right_voltage *= scale;

}
