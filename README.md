# raPID: Rafaels PID flight controller

[![Build Status](https://travis-ci.org/barafael/raPID.svg?branch=master)](https://travis-ci.org/barafael/raPID)

This flight controller consists mostly of a teensy 3.2 and an EM-SENtral motion sensor board. The
teensy can read up to 8 PWM or PPM channels from your standard RC receiver (see
include/pins.h for input/output pin numbers). Each of the outputs can be driven
by a weighted sum of the roll, pitch, yaw PID responses and the throttle
channel. The remaining 4 channels are currently unused, but it would be simple
to use them to drive the outputs like the throttle channel does. This is a
matrix multiplication, conceptually. The matrix maps an input (RX, PID) to the
output.

Three inputs drive the PID setpoints. By default, a cascaded PID controller is
used - resulting in stabilize mode. The first controller gets the RX input and
the absolute attitude from the IMU sensor. It outputs the desired rate. This
rate is fed as the setpoint to another controller, which uses the faster gyro
measurements of angular rate as process value. This setup results in
stabilization mode (stick deviation sets vehicle attitude). Acro mode is
achieved by disabling the first controller (which will then just pass through
the setpoint to the next controller) and only using the rate controller.

## Features
- [x] Arming/disarming
- [x] Mixers for each output, applying differently weighted inputs and IMU data to the outputs
- [x] Rate/stabilize mode
- [x] Support several output waveforms
  - [x] Various output classes (Servo, ESC, anyPWM)
  - [x] 400Hz update rate for ESCs using analogWrite timers (to do: test it)
  - [x] Arbitrary PWM waveform generation to drive even LEDs
- [x] PID Improvements
  - [x] D noise filters: lowpass, moving average
  - [x] Derivative-on-{error, feedback, setpoint}
  - [ ] Fixed point PID implementation
  - [ ] [not needed right now] Calculating error external from algorithm or pass an ```errorfunc(number, number) -> number``` function pointer (more general)

## TODO
- [x] Fix serial monitor ritual (current: remove tx, reboot, wait for sermon, connect tx)
- [x] Fix gyro vs. fused and rate vs. stbl issues (-15 factor)
- [ ] Receiver
  - [x] Per-channel offsets to set zero/mid-points and somehow work around special case for throttle, which needs 50% extra offset
  - [ ] Fix PPM receiver read
- Outputs
  - [x] 400Hz PWM output test and fix
  - [ ] Rethink set_limits, general implementation of generic waveforms? Specialization in servo (endpoints, expo?, trimming, inversion). Maximum range must be at least standard max signal pulse width
- [ ] Safety Enhancements
  - [x] Make sure arming functionality works and is reliable
  - [ ] Add safety mechanisms for receiver signal loss (detect RX failsafe output)
  - [ ] Fix/Improve watchdog timer functionality. Is this even necessary? A software crash will likely lead to crash of vehicle, since the controller boots to disarmed mode. Check if wakeup from watchdog, then proceeding armed? Currently, watchdog is disabled
- [x] Increase IMU sensor sampling rates - 1kHz

## Ideas
- [ ] Live coefficient tweaking (standard tx or telemetry hardware)
  - [ ] RFM95 lora board for config data, telemetry
- [x] IMU solution overhaul: Ultimate SENtral or other; constant sampling rate simplifies PID and makes theory on time-discrete systems applicable
  - [x] General IMU interface class to test different IMU implementations
- [ ] Matrix multiplication for output coefficients (every output is some weighted sum of the inputs + PID response) Possibly use DSP instructions and SIMD - one microsecond for multiplying 8x12 and a 12 column vector is achievable
  - [ ] Weights in output matrix are only linear multiplication. Would it make sense to use a matrix of function pointers to support expo? Performance (inlining possible?)?
  - [ ] 8x12 x 12x1 int16_t matrix/vector multiplication with naive for loop implementation: 12us. Unrolled loops: 3us-6us. Sufficient, probably.
  - [ ] 8x12 x 12x1 float matrix-vector multiplication with unrolled loops: 170us. Insufficient.
- [ ] Arbitrary flight modes (different PID settings, offsets, and I/O matrix)
- [ ] Flight mode interpolation (otherwise called transitional mixers) to smoothly switch between any two flight modes
  - [ ] Flight mode matrix to other flight mode matrix: ```(1 - transition) * old_mat + transition * new_mat``` for now. Sine/other functions, duration of transition, etc. later.
  - [ ] How to support more complex transitions where linear is insufficient? Pass pointer to function that takes end and start points, and percent, and gives interpolation value:
        ```squared_sine(startpoint, endpoint, percent) -> interpolated```
- [ ] Adjust settings wirelessly, ideally using a desktop application/website (have not thought about that part yet... far off)
- [ ] Use an RTOS?
  * Periodic tasks that run slower than flight loop:
    * Arming/disarming toggle check
    * Flightmode matrix transition update
    * Telemetry
    * Watchdog
    * RX update (depending on mode(PPM, PWM)) lower frequency than flight loop
    * RX check if has_channel (or use RC RX failsafe mode to detect?) [can run at 5Hz]
    * Serial debug output (?)
  * As fast as possible
    * IMU read
    * PID loops
    * Flight mode matrix vector multiplication
    * ```output.apply(value)```

Blog-in-progress @ [https://barafael.github.io/Remote-Control-Vehicle-Balance-controller/](https://barafael.github.io/Remote-Control-Vehicle-Balance-controller/)

BUG: when the shut_off function is called in the output actuator driver, the PWM signal is set to 0.
The actuator driver should only set the PWM signal to 0 if the output actuator type is a motor (it can also be a servo or something else).

## Inspiration and Acknowledgments

* OpenAeroVTOL from the RCGroups forums, by HappySundays - awesome project.
* Jeff Rowberg's MPU6050 library and example code.
