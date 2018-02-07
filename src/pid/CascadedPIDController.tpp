template<typename T>
CascadedPIDController<T>::CascadedPIDController(
        PIDParams<T> *primary_params,
        PIDParams<T> *secondary_params)
    : primary   { primary_params }
    , secondary { secondary_params } {}

template<typename T>
void CascadedPIDController<T>::set_mode(controller_mode mode) {
    this->mode = mode;
}

template<typename T>
T CascadedPIDController<T>::compute(
              const T    measured_angle,
              const T    measured_velocity,
              const T    setpoint) {
    T primary_result   = primary  .compute(measured_angle, setpoint);

    return secondary.compute(measured_velocity, /*-15 **/ primary_result);
}

