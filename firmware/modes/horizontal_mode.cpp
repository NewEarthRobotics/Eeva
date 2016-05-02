// Includes
#include <cmath>
#include "telemetry_receive_task.h"
#include "globs.h"
#include "main_control_task.h"
#include "physical_constants.h"
#include "robot_settings.h"
#include "telemetry_send_task.h"
#include "util_assert.h"

//******************************************************************************
void MainControlTask::horizontalMode(void)
{
    yaw_command_ += motion_commands_.angular_velocity * delta_t_;

    float left_speed_error = motion_commands_.linear_velocity - odometry_.left_speed;
    float left_voltage_command = left_speed_pid.calculate(left_speed_error, delta_t_);

    float right_speed_error = motion_commands_.linear_velocity - odometry_.right_speed;
    float right_voltage_command = right_speed_pid.calculate(right_speed_error, delta_t_);

    // Run yaw controller to calculate desired difference in voltage between motors.
    float yaw_error = (yaw_command_ - odometry_.yaw);
    float delta_voltage = yaw_pid.calculate(yaw_error, -imu_.gyros[2], delta_t_);

    if (modes_.state != STATE_NORMAL)
    {
        left_voltage_command = 0.0f;
        right_voltage_command = 0.0f;
        delta_voltage = 0.0f;
        yaw_pid.resetIntegral();

        // Keep commands in sync with state to keep from building up large error.
        yaw_command_ = odometry_.yaw;
        motion_commands_.linear_velocity = 0.0f;
        motion_commands_.angular_velocity = 0.0f;
        receive_task.handle(motion_commands_); // request reset to be published.
    }

    motors_.left_voltage = left_voltage_command - delta_voltage;
    motors_.right_voltage = right_voltage_command + delta_voltage;

}
