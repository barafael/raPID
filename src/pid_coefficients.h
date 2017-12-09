#ifndef PID_COEFFICIENTS_H
#define PID_COEFFICIENTS_H

#include <Arduino.h>
#include "WProgram.h"

#include <stdint.h>
#include <string.h>

/*
   ————————————————————————————————————————————————————
   ———        PID VARIABLES AND COEFFICIENTS        ———
   ————————————————————————————————————————————————————
*/

void init_pid_coefficients();

extern size_t flight_mode_index;

struct __attribute__((packed)) coefficient_t {
    double rate_p = 0.0;
    double rate_i = 0.0;
    double rate_d = 0.0;

    uint16_t max_rate    = 0;
    uint16_t integral_limit_rate = 0;

    bool rate_passthrough = false;

    double stbl_p = 0.0;
    double stbl_i = 0.0;
    double stbl_d = 0.0;

    uint16_t max_stbl    = 0;
    uint16_t integral_limit_stbl = 0;

    bool stbl_passthrough = false;

    public:

    coefficient_t() { }

    coefficient_t(double _rate_p, double _rate_i, double _rate_d, double _max_rate, double _integral_limit_rate, bool _rate_pass,
                  double _stbl_p, double _stbl_i, double _stbl_d, double _max_stbl, double _integral_limit_stbl, bool _stbl_pass) {

        rate_p = _rate_p;
        stbl_p = _stbl_p;

        rate_i = _rate_i;
        stbl_i = _stbl_i;

        rate_d = _rate_d;
        stbl_d = _stbl_d;

        max_rate = _max_rate;
        max_stbl = _max_stbl;

        integral_limit_rate = _integral_limit_rate;
        integral_limit_stbl = _integral_limit_stbl;

        rate_passthrough = _rate_pass;
        stbl_passthrough = _stbl_pass;
    }
};

struct __attribute__((packed)) flight_mode_t {
    char name[20] = { 0 };
    coefficient_t roll;
    coefficient_t pitch;
    coefficient_t yaw;

    public:
    flight_mode_t() {
        strncpy(name, "unnamed", 20);
        roll  = coefficient_t();
        pitch = coefficient_t();
        yaw   = coefficient_t();
    }

    flight_mode_t(char _name[20], coefficient_t roll_coeff, coefficient_t pitch_coeff, coefficient_t yaw_coeff) {
        strncpy(name, _name, 20);
        roll  = roll_coeff;
        pitch = pitch_coeff;
        yaw   = yaw_coeff;
    }

    flight_mode_t* set_roll(coefficient_t _roll) {
        roll = _roll;
        return this;
    }

    flight_mode_t* set_pitch(coefficient_t _pitch) {
        pitch = _pitch;
        return this;
    }

    flight_mode_t* set_yaw(coefficient_t _yaw) {
        yaw = _yaw;
        return this;
    }

    coefficient_t* get_roll() {
        return &roll;
    }

    coefficient_t* get_pitch() {
        return &pitch;
    }

    coefficient_t* get_yaw() {
        return &yaw;
    }
};

struct __attribute__((packed)) modes_arr {
    private:
    static const uint8_t MAX_NUM_MODES = 8;

    flight_mode_t modes[MAX_NUM_MODES] = { };
    public:

    uint8_t num_modes = 0;
    modes_arr() {
        num_modes = 0;
    }

    modes_arr* add_mode(flight_mode_t mode) {
        if (num_modes >= MAX_NUM_MODES) {
            Serial.println("List of modes is already full! Cannot add.");
            Serial.println(num_modes);
            Serial.println(MAX_NUM_MODES);
            //return this;
        }
        Serial.println("Adding flight mode");
        modes[num_modes++] = mode;
        return this;
    }

    flight_mode_t* get_current_profile() {
        return &(modes[flight_mode_index]);
    }
};

union __attribute__((packed)) data_t {
    uint8_t bytes[sizeof(modes_arr)];
    modes_arr modes = modes_arr();

    public:
    data_t() {
    }
};

struct __attribute__((packed)) settings_t {
    data_t data = data_t();
    public:
    settings_t() { }
    void add_mode(flight_mode_t mode) {
        data.modes.add_mode(mode);
    }
};

#endif // PID_COEFFICIENTS_H
