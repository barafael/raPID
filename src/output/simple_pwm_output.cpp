#include "../../include/output/simple_pwm_output.h"

/*@ requires pin > 0; */
simple_pwm_output_t simple_out_init(const uint8_t pin, float throttle_volume, float roll_volume, float pitch_volume,
                                  float yaw_volume, bool is_throttle) {
    mock_pinMode(pin, OUTPUT);

    mixer_t mixer = mixer_init(throttle_volume, roll_volume, pitch_volume, yaw_volume);

    simple_pwm_output_t result;
    result.pin                         = pin;
    result.mixer                       = mixer;
    result.milli_throttle              = 0;
    result.low_throttle_cutoff_enabled = is_throttle;
    result.throttle_low_cutoff         = 25;
    return result;
}

/* requires \valid(self);
   ensures self->milli_throttle < 256 && self->milli_throttle >= 0;
   assigns self->milli_throttle;
*/
static void simple_out_write(simple_pwm_output_t *self, uint16_t _milli_throttle) {
    self->milli_throttle = clamp(_milli_throttle, 0, 255);
    // reusing _millithrottle could result in error if clamp was necessary
    mock_analogWrite(self->pin, self->milli_throttle);
}

/*@ requires \valid(self);
    behavior cutoff:
      assumes self->low_throttle_cutoff_enabled;
      assumes _milli_throttle < self->throttle_low_cutoff;
      ensures motor_status == MOTOR_OFF;
    behavior on:
      assumes _milli_throttle > self->throttle_low_cutoff;
      ensures motor_status == MOTOR_OFF;

    complete behaviors on, cutoff;
    disjoint behaviors on, cutoff;
 */
void simple_out_apply(simple_pwm_output_t *self, uint16_t _milli_throttle,
        float roll_stbl, float pitch_stbl, float yaw_stbl) {
    /* Throttle cutoff to avoid spinning props due to movement when throttle is low but state is armed
     * Do it here, before the control values are added up */
    if (self->low_throttle_cutoff_enabled && (_milli_throttle < self->throttle_low_cutoff)) {
        simple_out_shutoff(self);
        return;
    }

    /* intermediary int16_t to prevent overflow */
    int16_t throttle_tmp = (int16_t) _milli_throttle * (int16_t) self->mixer.throttle_volume;

    throttle_tmp += (int16_t) (roll_stbl  * self->mixer.roll_volume);
    throttle_tmp += (int16_t) (pitch_stbl * self->mixer.pitch_volume);
    throttle_tmp += (int16_t) (yaw_stbl   * self->mixer.yaw_volume);

    clamp(throttle_tmp, 0, 255);

    simple_out_write(self, throttle_tmp);
}

void simple_out_shutoff(simple_pwm_output_t *self) {
    //@ ghost motor_status = MOTOR_OFF;
    simple_out_write(self, 0);
}

void simple_out_set_throttle_volume(simple_pwm_output_t *self, float volume) {
    self->mixer.throttle_volume = volume;
}

void simple_out_set_roll_volume    (simple_pwm_output_t *self, float volume) {
    self->mixer.roll_volume = volume;
}

void simple_out_set_pitch_volume   (simple_pwm_output_t *self, float volume) {
    self->mixer.pitch_volume = volume;
}

void simple_out_set_yaw_volume     (simple_pwm_output_t *self, float volume) {
    self->mixer.yaw_volume = volume;
}
