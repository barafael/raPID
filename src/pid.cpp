#include "pid_coefficients.h"

static float pid_error_rate;
static float pid_last_error_rate;

static float p_term_rate;
static float i_term_rate;
static float d_term_rate;


static float pid_error;
static float pid_last_error;

static float p_term;
static float i_term;
static float d_term;

settings_t settings;

void init_pid_coefficients() {
    coefficient_t roll_coeffs = coefficient_t(0.3, 0.005, 0.0, 400, 10, false,
                                              0.035, 0.0, 0.03, 400, 10, false);
    flight_mode_t default_mode = flight_mode_t();
    default_mode.set_roll(roll_coeffs);
    settings = settings_t();

    settings.add_mode(default_mode);
}

/* Calculate PID output based on absolute angle in attitude[] */
float calculate_PID_stabilize(float pid_roll_setpoint, float measurement, float roll_rate) {
    float pid_output_roll;

    pid_error = measurement - pid_roll_setpoint;

    p_term = settings.data.modes.get_current_profile()->get_roll()->stbl_p * pid_error;

    i_term += (settings.data.modes.get_current_profile()->get_roll()->stbl_i * pid_error);
    if (i_term > settings.data.modes.get_current_profile()->get_roll()->integral_limit_stbl) i_term = settings.data.modes.get_current_profile()->get_roll()->integral_limit_stbl;
    else if (i_term < (settings.data.modes.get_current_profile()->get_roll()->integral_limit_stbl * -1)) i_term = (settings.data.modes.get_current_profile()->get_roll()->integral_limit_stbl * -1);

    d_term = settings.data.modes.get_current_profile()->get_roll()->stbl_d * roll_rate;
    // d_term = pid_d_gain_roll * (pid_error - pid_last_error);

    pid_output_roll = p_term + i_term + d_term;

    if (pid_output_roll > settings.data.modes.get_current_profile()->get_roll()->max_rate) pid_output_roll =pid_output_roll > settings.data.modes.get_current_profile()->get_roll()->max_rate;
    else if (pid_output_roll < settings.data.modes.get_current_profile()->get_roll()->max_rate * -1) pid_output_roll = settings.data.modes.get_current_profile()->get_roll()->max_rate * -1;

    pid_last_error = pid_error;

    return pid_output_roll;
}

/* Calculate PID output based on angular rate */
float calculate_PID_rate(float pid_roll_rate_setpoint, float measurement) {
    float pid_output_roll_rate;
    pid_error_rate = measurement - pid_roll_rate_setpoint;

    p_term_rate = settings.data.modes.get_current_profile()->get_roll()->rate_p * pid_error_rate;

    i_term_rate += (settings.data.modes.get_current_profile()->get_roll()->rate_i * pid_error_rate);
    if (i_term_rate > settings.data.modes.get_current_profile()->get_roll()->integral_limit_rate) i_term_rate = settings.data.modes.get_current_profile()->get_roll()->integral_limit_rate;
    else if (i_term_rate < (settings.data.modes.get_current_profile()->get_roll()->integral_limit_rate * -1)) i_term_rate = (settings.data.modes.get_current_profile()->get_roll()->integral_limit_rate * -1);

    d_term_rate = settings.data.modes.get_current_profile()->get_roll()->rate_d * (pid_error_rate - pid_last_error_rate);

    pid_output_roll_rate = p_term_rate + i_term_rate + d_term_rate;

    if (pid_output_roll_rate > settings.data.modes.get_current_profile()->get_roll()->max_rate) pid_output_roll_rate = settings.data.modes.get_current_profile()->get_roll()->max_rate;
    else if (pid_output_roll_rate < settings.data.modes.get_current_profile()->get_roll()->max_rate * -1) pid_output_roll_rate = settings.data.modes.get_current_profile()->get_roll()->max_rate * -1;

    pid_last_error_rate = pid_error_rate;

    return pid_last_error_rate;
}
