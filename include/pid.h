#ifndef PID_H
#define PID_H

void calculate_PID_stabilize(double pid_roll_setpoint, double measurement, double roll_rate);
void calculate_PID_rate(double pid_roll_setpoint, double measurement);

#endif // PID_H
