#ifndef PID_H
#define PID_H

double calculate_PID_stabilize(double pid_roll_setpoint, double measurement, double roll_rate);
double calculate_PID_rate(double pid_roll_setpoint, double measurement);

#endif // PID_H
