# raPID: Rafaels PID flight controller

This flight controller consists mostly of a teensy 3.2 and an MPU-6050. The
teensy can read up to 8 PWM or PPM channels from your standard RC receiver (see
include/pins.h for input/output pin numbers). Each of the outputs can be driven
by a weighted combination of the roll, pitch, yaw PID responses and the
throttle channel. The remaining 4 channels are currently unused, but it would
be simple to use them to drive the outputs like the throttle channel. Three
inputs drive the PID setpoints. By default, a cascaded PID controller is used -
resulting in stabilize mode. The first controller gets the RX input and the
absolute attitude from the IMU sensor. It outputs a desired rate. This rate is
fed as setpoint to another controller, which uses the faster gyro measurements
of angular rate as process value. This setup results in stabilization mode
(stick deviation sets vehicle attitude). By disabling the first controller
(which will then just pass through the setpoint to the next controller), acro
mode can be achieved.

- [x] Arming/disarming
- [x] PWM/PPM support
- [x] Rate/stabilize mode
- [x] Mixers for each output, applying differently weighted inputs and IMU data to the outputs
- [ ] 400Hz update rate for ESCs using analogWrite timers (fastpwm branch)
- [ ] Allow arbitrary waveform generation for PWM to drive even LEDs (fastpwm branch)
- [ ] Live coefficient tweaking (standard tx or telemetry hardware)
- [ ] Fix gyro vs. fused and rate vs. stbl issues (-15 factor)
- [ ] Fix serial monitor ritual (current: remove tx, reboot, wait for sermon, connect tx)
- [ ] IMU solution overhaul: Ultimate SENtral or other
- [ ] Migrate to 3.5 teensy
- [ ] Flight mode data structure
- [ ] Arbitrary flight modes (different PID settings, offsets, and I/O matrix)
- [ ] Matrix multiplication for I/O coefficients
- [ ] Flight mode interpolation (otherwise called transitional mixers) to smoothly switch between any two flight modes
- [ ] Telemetry hardware + data logging

Adjusting the settings should be done via bluetooth and a desktop application/website (have not thought about that part yet... far off)

Blog-in-progress @ [https://barafael.github.io/Simple-PID-flight-controller/](https://barafael.github.io/Simple-PID-flight-controller/)

## Inspiration and Acknowledgments

* Jeff Rowberg's MPU6050 library and example code.
* OpenAeroVTOL from the RCGroups forums, by HappySundays - awesome project.

[![Build Status](https://travis-ci.org/barafael/raPID.svg?branch=master)](https://travis-ci.org/barafael/raPID)
