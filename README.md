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
- [ ] 400Hz update rate for ESCs using analogWrite timers
- [ ] Live coefficient tweaking
- [ ] simplify IMU data ranges
- [ ] Flight mode data structure
- [ ] Matrix multiplication for I/O coefficients
- [ ] Arbitrary flight modes (different PID settings and offsets)
- [ ] Flight mode interpolation (otherwise called transitional mixers) to smoothly switch between any two flight modes.

Adjusting the settings should be done via bluetooth and a desktop application/website (have not thought about that part yet... far off)

Blog-in-progress @ [https://barafael.github.io/Simple-PID-flight-controller/](https://barafael.github.io/Simple-PID-flight-controller/)

## Inspiration and Acknowledgments

* Jeff Rowberg's MPU6050 library and example code.
* OpenAeroVTOL from the RCGroups forums, by HappySundays - awesome project.

[![Build Status](https://travis-ci.org/barafael/raPID.svg?branch=master)](https://travis-ci.org/barafael/raPID)
