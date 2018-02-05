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
- [x] Support several output waveforms
  - [x] Various output classes (Servo, ESC, anyPWM)
  - [x] 400Hz update rate for ESCs using analogWrite timers (to do: test it)
  - [x] Arbitrary waveform generation for PWM to drive even LEDs (to do: test it)
- [x] PID Improvements
  - [x] D noise filter, derivative-on-{error, feedback, setpoint}
  - [x] Implement cascaded PID in a more explicit way
  - [x] Fixed point PID implementation (to do: test it)
  - [ ] [unnecessary] Calculating error external from algorithm or pass an ```errorfunc(number, number) -> number``` function pointer (more general)

## Fixes
- [ ] Fix serial monitor ritual (current: remove tx, reboot, wait for sermon, connect tx)
- [ ] Fix gyro vs. fused and rate vs. stbl issues (-15 factor)
- [ ] Receiver
  - [ ] Per-channel offsets to set zero/mid-points and somehow work around special case for throttle, which needs 50% extra offset
  - [ ] Higher resolution, use complete int16_t range for later fast calculation
- [ ] Outputs
  - [ ] Rethink about set_limits, general implementation for generic waveforms? Specialization in servo. Maximum range must be at least standard max signal pulse width
  - [ ] Weights in output matrix are linear. Would it make sense to use matrix of function pointers to support expo? Performance (inlining possible?)?
- [ ] Safety Enhancements
  - [ ] Make sure arming functionality works and is reliable
  - [ ] Add safety mechanisms for receiver signal loss
  - [ ] Fix/Improve watchdog timer functionality. Is this even necessary? crash of software -> likely crash of vehicle

## Ideas
- [ ] Live coefficient tweaking (standard tx or telemetry hardware)
- [ ] IMU solution overhaul: Ultimate SENtral or other; constant sampling rate simplifies PID and makes theory on time-discrete systems applicable
  - [x] General IMU interface class to test different IMU implementations
- [ ] Matrix multiplication for output coefficients (every output is some weighted sum of the inputs + pid response) Possibly use DMP instructions and SIMD - one microsecond for multiplying 8x12 and a 12 column vec is achievable
- [ ] Arbitrary flight modes (different PID settings, offsets, and I/O matrix)
- [ ] Flight mode interpolation (otherwise called transitional mixers) to smoothly switch between any two flight modes
- [ ] Adjust settings wirelessly, ideally using a desktop application/website (have not thought about that part yet... far off)
- [ ] Telemetry hardware + data logging
- [ ] Use an RTOS?

Blog-in-progress @ [https://barafael.github.io/Remote-Control-Vehicle-Balance-controller/](https://barafael.github.io/Remote-Control-Vehicle-Balance-controller/)

## Inspiration and Acknowledgments

* OpenAeroVTOL from the RCGroups forums, by HappySundays - awesome project.
* Jeff Rowberg's MPU6050 library and example code.
