def pwm_to_w_velocity(pwm):
    # Function takes pwm value and turns into an angular velocity which is then
    # converted into a translational velocity. Angular velocity is found using
    # experimentation where a non-linear plot is broken down into various linear
    # regions with dead zones and saturation. Experimentation can be found on lab report
    if pwm >= 150:  # Max Speed Forward, Saturation
        w = 6.5
    elif pwm >= 110:  # Fast forward, Linear Near Saturation
        w = 0.0386 * pwm + 0.582
    elif pwm >= 91:  # Slow forward, Linear
        w = 0.254 * pwm - 23.114
    elif pwm >= 86:  # Deadzone
        w = 0
    elif pwm >= 70:  # Slow Backwards, Linear
        w = .297 * pwm - 25.456
    elif pwm >= 60:  # Fast Backwards, Linear Saturation
        w = .0756 * pwm - 10.038
    else:  # Max Speed Reverse, Saturation
        w = -6.33
    return w  # Translational Velocity