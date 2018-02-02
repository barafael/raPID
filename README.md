# raPID: Rafaels PID flight controller

[![Build Status](https://travis-ci.org/barafael/raPID.svg?branch=master)](https://travis-ci.org/barafael/raPID)

This flight controller consists mostly of a teensy 3.2 and an MPU-6050. The
teensy can read up to 8 PWM or PPM channels from your standard RC receiver (see
include/pins.h for input/output pin numbers). Each of the outputs can be driven
by a weighted sum of the roll, pitch, yaw PID responses and the throttle
channel. The remaining 4 channels are currently unused, but it would be simple
to use them to drive the outputs like the throttle channel. This is a matrix
multiplication, conceptually. The matrix maps an input (rx, pid) to the output.

Three inputs drive the PID setpoints. By default, a cascaded PID controller is
used - resulting in stabilize mode. The first controller gets the RX input and
the absolute attitude from the IMU sensor. It outputs a desired rate. This rate
is fed as setpoint to another controller, which uses the faster gyro
measurements of angular rate as process value. This setup results in
stabilization mode (stick deviation sets vehicle attitude). By disabling the
first controller (which will then just pass through the setpoint to the next
controller), acro mode can be achieved.

## Features
- [x] Arming/disarming
- [x] Mixers for each output, applying differently weighted inputs and IMU data to the outputs
- [x] Rate/stabilize mode
- [x] PWM/PPM support
- [x] Various output classes (Servo, ESC, anyPWM)
- [ ] 400Hz update rate for ESCs using analogWrite timers
- [ ] Arbitrary waveform generation for PWM to drive even LEDs
      PID improvements:
- [x] D noise filter, derivative-on-{error, feedback, setpoint}
- [ ] Implement cascaded PID in a more explicit way
- [ ] Fixed point PID implementation
- [ ] Calculating error external from algorithm or pass an errorfunc(number, number) -> number function pointer (more general)
- [ ] Telemetry hardware + data logging
## Fixes
- [ ] Fix serial monitor ritual (current: remove tx, reboot, wait for sermon, connect tx)
- [ ] Fix gyro vs. fused and rate vs. stbl issues (-15 factor)
## Ideas
- [ ] Live coefficient tweaking (standard tx or telemetry hardware)
- [ ] IMU solution overhaul: Ultimate SENtral or other
      - constant sampling rate simplifies PID and makes theory on time-discrete systems applicable
- [ ] Arbitrary flight modes (different PID settings, offsets, and I/O matrix)
- [ ] Matrix multiplication for output coefficients (every output is some weighted sum of the inputs + pid response)
- [ ] Flight mode interpolation (otherwise called transitional mixers) to smoothly switch between any two flight modes
- [ ] Use an RTOS?

Adjusting the settings should be done wirelessly, ideally using a desktop
application/website (have not thought about that part yet... far off)

Blog-in-progress @ [https://barafael.github.io/Simple-PID-flight-controller/](https://barafael.github.io/Simple-PID-flight-controller/)

## Inspiration and Acknowledgments

* OpenAeroVTOL from the RCGroups forums, by HappySundays - awesome project.
* Jeff Rowberg's MPU6050 library and example code.
