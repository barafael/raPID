/*
   ————————————————————————————————————————————————————
   ———        PID VARIABLES AND COEFFICIENTS        ———
   ————————————————————————————————————————————————————
*/

float pid_output_roll;

static double pid_p_gain_roll = 0.15;
static double pid_i_gain_roll = 0.002;
static double pid_d_gain_roll = 0.032;
static int pid_max_roll = 400;
static int pid_roll_integral_limit = 10;

static double pid_error;
static double pid_last_error;

static double p_term;
static double i_term;
static double d_term;

/* Calculate PID output based on absolute angle in attitude[] */
void calculate_PID_absolute(float pid_roll_setpoint, float measurement, float roll_rate) {
    pid_error = measurement - pid_roll_setpoint;

    p_term = pid_p_gain_roll * pid_error;

    i_term += (pid_i_gain_roll * pid_error);
    if (i_term > pid_roll_integral_limit) i_term = pid_roll_integral_limit;
    else if (i_term < (pid_roll_integral_limit * -1)) i_term = (pid_roll_integral_limit * -1);

    d_term = pid_d_gain_roll * roll_rate;
    //d_term = pid_d_gain_roll * (pid_error - pid_last_error);

    pid_output_roll = p_term + i_term + d_term;

    if (pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
    else if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

    pid_last_error = pid_error;
}

/* Calculate PID output based on angular rate */
void calculatePID_angular_rate(float pid_roll_setpoint, float measurement) {
    pid_error = measurement - pid_roll_setpoint;

    p_term = pid_p_gain_roll * pid_error;

    i_term += (pid_i_gain_roll * pid_error);
    if (i_term > pid_roll_integral_limit) i_term = pid_roll_integral_limit;
    else if (i_term < (pid_roll_integral_limit * -1)) i_term = (pid_roll_integral_limit * -1);

    d_term = pid_d_gain_roll * (pid_error - pid_last_error);

    pid_output_roll = p_term + i_term + d_term;

    if (pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
    else if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

    pid_last_error = pid_error;
}
