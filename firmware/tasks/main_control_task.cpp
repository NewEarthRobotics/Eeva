// Includes
#include <cmath>
#include "debug_printf.h"
#include "globs.h"
#include "main_control_task.h"
#include "math_util.h"
#include "physical_constants.h"
#include "robot_settings.h"
#include "telemetry_send_task.h"
#include "util_assert.h"

//******************************************************************************
MainControlTask::MainControlTask(float frequency) :
        PeriodicTask("Main Control", TASK_ID_MAIN_CONTROL, frequency),
        yaw_pid(16, 160, 0.4, -2.5, 2.5, -10, 10),
        left_speed_pid(8, 240, 0.0, -2.5, 2.5, -10, 10),
        right_speed_pid(8, 240, 0.0, -2.5, 2.5, -10, 10),
        left_position_pid(0, 0, 0.0, -2.5, 2.5, -10, 10),
        right_position_pid(0, 0, 0.0, -2.5, 2.5, -10, 10),
        line_track_pid(400, 960, 0.0, -2.5, 2.5, -10, 10),
        left_encoder_(EncoderA),
        right_encoder_(EncoderB),
        left_deriv_(delta_t_, 20.0f , 0.707f),
        right_deriv_(delta_t_, 20.0f , 0.707f),
        pos_cmd_deriv_(delta_t_, 50.0f , 0.707f),
        theta_cmd_deriv_(delta_t_, 100.0f , 0.707f),
        beta_deriv_(delta_t_, 10.0f , 0.707f),
        distance_command_(0.0f),
        yaw_command_(0.0f),
        capturing_data_(false),
        capture_counter_(0),
        capture_run_counts_(0),
        max_samples_(0)
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        // Set full state feedback gains to default values.
        K_[i] = K_DEFAULT[i];
    }
}

//******************************************************************************
void MainControlTask::initialize(void)
{
    // Subtract one since instance 0 isn't used.
    max_samples_ = globs[GLO_ID_CAPTURE_DATA]->get_num_instances() - 1;
}

//******************************************************************************
void MainControlTask::readNewData(void)
{
    glo_modes.read(&modes_);
    glo_imu.read(&imu_);
    glo_roll_pitch_yaw.read(&roll_pitch_yaw_);
    glo_theta_zero.read(&theta_zero_);
    glo_motion_commands.read(&motion_commands_);
    glo_status_data.read(&status_data_);
}

//******************************************************************************
void MainControlTask::publishNewData(void)
{
    glo_odometry.publish(&odometry_);
    glo_analog.publish(&analog_);
    glo_wave.publish(&wave_);
    glo_motor_control.publish(&motors_);
}

//******************************************************************************
void MainControlTask::run(void)
{
    // Read in new data from other tasks.
    readNewData();

    // Copy most recent ADC and estimate battery voltage.
    analog_inputs_.getVoltages(analog_.voltages);
    analog_.battery_voltage = BATTERY_SCALE * analog_.voltages[8] + BATTERY_OFFSET;

    // Convert encoder measurements to odometry data.
    odometry_.left_distance = ENCODER_SCALES[0] * left_encoder_.read();
    odometry_.right_distance = ENCODER_SCALES[1] * right_encoder_.read();
    odometry_.avg_distance = (odometry_.left_distance + odometry_.right_distance) / 2.0f;
    odometry_.yaw = (odometry_.right_distance - odometry_.left_distance) / WHEEL_BASE;
    odometry_.left_speed = left_deriv_.calculate(odometry_.left_distance);
    odometry_.right_speed = right_deriv_.calculate(odometry_.right_distance);
    odometry_.avg_speed = (odometry_.left_speed + odometry_.right_speed) / 2.0f;

    // Run the current mode to calculate the voltage to apply to each motor.
    // Default to zero in case a mode doesn't update one (or both) of the values.
    motors_.left_voltage = 0;
    motors_.right_voltage = 0;
    switch (modes_.main_mode)
    {
        case MAIN_MODE_BALANCE:
            balanceMode();
            break;
        case MAIN_MODE_HORIZONTAL:
            horizontalMode();
            break;
        case MAIN_MODE_LINE_FOLLOWING:
            lineFollowingMode();
            break;
        case MAIN_MODE_EXPERIMENT:
            experimentMode();
            break;
        case MAIN_MODE_CUSTOM:
            customMode();
            break;
        default:
            assert_always_msg(ASSERT_STOP, "Invalid main mode.");
            break;
    }

    // Update how much voltage is being applied to the motors.
    updateMotorVoltage();

    // Capture / send back data if the user has requested it.
    runDataCapture();

    // If the user wants to start and collect data at the same time then "experiment modes"
    // need to do this at a precise time, but other modes just want to start collecting
    // data right away.
    if ((modes_.main_mode != MAIN_MODE_EXPERIMENT) &&
        (capture_command_.is_start && capture_command_.paused))
    {
        capture_command_.paused = false;
        this->handle(capture_command_);
    }

    publishNewData();
}

//******************************************************************************
void MainControlTask::updateMotorVoltage(void)
{
    // Only command motors to spin when in a valid state.
    bool in_valid_state = (modes_.state == STATE_NORMAL);

    // Look for errors that would cause us to need to disable the motors.
    uint8_t error_detected = (status_data_.error_codes & ERROR_CODE_CRICITAL_BATTERY);

    if (!in_valid_state || error_detected)
    {
        motors_.left_voltage = 0;
        motors_.right_voltage = 0;
    }

    // At this point we should always have a valid battery voltage, which we need so we can figure
    // out what PWM to apply to the motors.
    if (analog_.battery_voltage <= 0)
    {
        assert_always_msg(ASSERT_CONTINUE, "Battery voltage reading as 0 volts.");
        return;
    }

    // Convert voltages to duty cycles.
    motors_.left_duty = motors_.left_voltage / analog_.battery_voltage;
    motors_.right_duty = motors_.right_voltage / analog_.battery_voltage;

    // Make sure duty cycle is between -100% and 100%
    motors_.left_duty = limit(motors_.left_duty, -1.0f, 1.0f);
    motors_.right_duty = limit(motors_.right_duty, -1.0f, 1.0f);

    // The effective voltage applied to the motor for a given duty cycle is proportional
    // to the current battery voltage.  To remove this dependency so that performance doesn't change
    // as the battery drains, we can simply scale the duty cycle down.  Since the motors won't
    // spin below the critical battery level it makes sense to use this as the maximum voltage
    // allowed across the motors. So if the battery is at 8 volts and the maximum allowed voltage
    // is 6.3 volts then a 100% duty cycle will get scaled down by (6.3 / 8) to be a 78% duty cycle.
    float scale = 1;
    if (CRITICAL_BATTERY_VOLTAGE <= analog_.battery_voltage)
    {
        scale = CRITICAL_BATTERY_VOLTAGE / analog_.battery_voltage;
    }
    motors_.left_duty *= scale;
    motors_.right_duty *= scale;

    // Recalculate applied voltages so they're consistent with the "corrected" PWM values.
    motors_.left_voltage = motors_.left_duty * analog_.battery_voltage;
    motors_.right_voltage = motors_.right_duty * analog_.battery_voltage;

    // Take into account order of motor wiring.
    float left_duty = motors_.left_duty * MOTOR_SIGNS[0];
    float right_duty = motors_.right_duty * MOTOR_SIGNS[1];

    hbridge_.setDutyA(left_duty);
    hbridge_.setDutyB(right_duty);
}

//******************************************************************************
void MainControlTask::runDataCapture(void)
{
    // If most recent capture command was a start command then need to send data.
    bool currently_capturing_data = capture_command_.is_start && !capture_command_.paused;

    // Reset counter if we just started taking data.
    if (!capturing_data_ && currently_capturing_data)
    {
        capturing_data_ = true;
        capture_counter_ = 0;
        debug_printf("I'm starting to collect data.");
    }

    // Check if we need to stop sending data because our buffer is full or user wants to stop.
    bool buffer_full = (capture_counter_ >= max_samples_);
    bool have_desired_samples = (capture_counter_ >= capture_command_.desired_samples);
    if (currently_capturing_data && (buffer_full || have_desired_samples))
    {
        currently_capturing_data = false;

        // Update glob so next time we don't try to keep capturing data.
        capture_command_.is_start = 0;
        glo_capture_command.publish(&capture_command_);
    }

    if (!currently_capturing_data && capturing_data_ && (capture_counter_ > 0))
    {
        // Just stopped taking data so first send send back all captured data instances.
        // then send back packet telling UI how many samples it should've gotten.
        debug_printf("I collected %d data samples.", capture_counter_);
        send_task.send(glo_capture_data.get_id(), 1, capture_counter_);
        capture_command_.total_samples = capture_counter_;
        glo_capture_command.publish(&capture_command_);
        send_task.send(glo_capture_command.get_id());
    }

    // Save that we're sending back capture data so we know for next loop.
    capturing_data_ = currently_capturing_data;

    if (currently_capturing_data)
    {
        if (throttleHz(capture_command_.frequency))
        {
            if (capture_counter_ == 0)
            {
                // Reset capture run counts here so first timestamp will be 0 seconds.
                capture_run_counts_ = 0;
            }

            // Read in globs that are only used for data capture.
            // -- none right now

            capture_data_.time = delta_t_ * capture_run_counts_;
            capture_data_.d1 = roll_pitch_yaw_.rpy[1]; // robot tilt (i.e. pitch)
            capture_data_.d2 = wave_.value;
            capture_data_.d3 = motors_.left_voltage;
            capture_data_.d4 = motors_.right_voltage;
            capture_data_.d5 = odometry_.left_distance;
            capture_data_.d6 = odometry_.right_distance;
            capture_data_.d7 = odometry_.left_speed;
            capture_data_.d8 = odometry_.right_speed;

            // Use capture counter as instance number to publish to.
            // Need to add one because instance numbers are indexed from 1.
            glo_capture_data.publish(&capture_data_, capture_counter_ + 1);

            ++capture_counter_;
        }

        // Increment after capturing data so first time stamp is 0 seconds.
        ++capture_run_counts_;
    }
}

//******************************************************************************
void MainControlTask::handle(glo_capture_command_t & command)
{
    // Validate request command settings.
    command.desired_samples = limit(command.desired_samples, (uint32_t)1, (uint32_t)max_samples_);

    if ((command.frequency == 0) || (command.frequency > max_samples_))
    {
        command.frequency = this->frequency_; // set to task frequency
    }

    // Scale the desired frequency to the closest frequency that can be achieved based on how fast task is running.
    int32_t scale = int32_t(this->frequency_ / command.frequency);
    command.frequency = (uint16_t)(this->frequency_ / scale);

    capture_command_ = command;
    glo_capture_command.publish(&command);
}

//******************************************************************************
void MainControlTask::reset_zero_tilt_angle(void)
{
    glo_theta_zero_t theta_zero = { 0 };
    glo_theta_zero.publish(&theta_zero);
}

//******************************************************************************
void MainControlTask::handle(glo_wave_t & wave)
{
    wave.state = WAVE_READY_TO_RUN;
    wave.value = 0;
    wave.time = 0;
    wave.total_time = 0;

    // TODO calculate trapezoidal coefficients

    wave_ = wave;
    glo_wave.publish(&wave);
}

//******************************************************************************
void MainControlTask::handle_balance_tilt_gains(glo_pid_params_t & tilt_params)
{
    K_[0] = tilt_params.kp;
    K_[1] = tilt_params.kd;
}

//******************************************************************************
void MainControlTask::handle_balance_position_gains(glo_pid_params_t & position_params)
{
    K_[2] = position_params.kp;
    K_[3] = position_params.kd;
}

//******************************************************************************
void MainControlTask::get_balance_tilt_gains(glo_pid_params_t & tilt_params)
{
    tilt_params.kp = K_[0];
    tilt_params.kd = K_[1];
    tilt_params.ki = 0;
    tilt_params.integral_lolimit = 0;
    tilt_params.integral_hilimit = 0;
    tilt_params.lolimit = -10000;
    tilt_params.hilimit = 10000;
}

//******************************************************************************
void MainControlTask::get_balance_position_gains(glo_pid_params_t & position_params)
{
    position_params.kp = K_[2];
    position_params.kd = K_[3];
    position_params.ki = 0;
    position_params.integral_lolimit = 0;
    position_params.integral_hilimit = 0;
    position_params.lolimit = -10000;
    position_params.hilimit = 10000;
}

//******************************************************************************
void MainControlTask::reset(void)
{
    left_encoder_.set(0);
    right_encoder_.set(0);
    left_deriv_.reset();
    right_deriv_.reset();
    pos_cmd_deriv_.reset();
    theta_cmd_deriv_.reset();
    beta_deriv_.reset();
    distance_command_ = 0.0f;
    yaw_command_ = 0.0f;
}
