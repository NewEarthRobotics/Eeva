// Includes
#include <cmath>
#include "derivative_filter.h"
#include "globs.h"
#include "main_control_task.h"
#include "physical_constants.h"
#include "robot_settings.h"
#include "util_assert.h"

//******************************************************************************
void MainControlTask::experiment2Mode(float experiment_input)
{
    if (modes_.state != STATE_NORMAL)
    {
        return;
    }

    // Wheel angular position control (in degrees).
    float position_command = experiment_input;

    // Use derivative filters for error terms since simple dx/dt is too noisy with encoders.
    static DerivativeFilter left_error_derivative(delta_t_, 30.0f, 0.707f);
    static DerivativeFilter right_error_derivative(delta_t_, 30.0f, 0.707f);

    float left_position_error = position_command - (odometry_.left_distance / WHEEL_RADIUS * RAD2DEG);
    float left_deriv = left_error_derivative.calculate(left_position_error);
    motors_.left_voltage = left_position_pid.calculate(left_position_error, left_deriv, delta_t_);

    float right_position_error = position_command - (odometry_.right_distance / WHEEL_RADIUS * RAD2DEG);
    float right_deriv = right_error_derivative.calculate(left_position_error);
    motors_.right_voltage = right_position_pid.calculate(right_position_error, right_deriv, delta_t_);

}
