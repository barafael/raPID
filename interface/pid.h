#ifndef PID_H
#define PID_H

float calculate_PID_stabilize(float pid_roll_setpoint, float measurement, float roll_rate);
float calculate_PID_rate(float pid_roll_setpoint, float measurement);

#endif // PID_H
