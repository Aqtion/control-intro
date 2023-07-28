from pymavlink import mavutil
import sys
import signal
from pid import PID
import numpy as np


def set_rc_channel_pwm(mav, channel_id, pwm=1500):
    """Set RC channel pwm value
    Args:a
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    mav.mav.rc_channels_override_send(
        mav.target_system,  # target_system
        mav.target_component,  # target_component
        *rc_channel_values
    )


def set_rotation_power(mav, power=0):
    """Set rotation power
    Args:
        power (int, optional): Power value -100-100
    """
    if power < -100 or power > 100:
        print("Power value out of range.")
        power = np.clip(power, -100, 100)

    power = int(power)

    set_rc_channel_pwm(mav, 4, 1500 + power * 5)


def main():
    mav = mavutil.mavlink_connection("udpin:0.0.0.0:14550")

    # catch CTRL+C
    def signal_handler(sig, frame):
        print("CTRL+C pressed. Disarming")
        mav.arducopter_disarm()
        mav.motors_disarmed_wait()
        print("Disarmed")
        sys.exit(0)

    # catch CTRL+C
    signal.signal(signal.SIGINT, signal_handler)

    # wait for the heartbeat message to find the system ID
    mav.wait_heartbeat()
    desired_heading_deg = float(input("Enter target heading: "))

    # arm the vehicle
    print("Arming")
    mav.arducopter_arm()
    mav.motors_armed_wait()
    print("Armed")

    # set mode to MANUAL
    print("Setting mode to MANUAL")
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        19,  # Manual mode
    )
    print("Mode set to MANUAL")
    set_rotation_power(mav, 0)

    # ask user for depth

    if desired_heading_deg > 180:
        desired_heading_deg = desired_heading_deg - 360

    desired_heading = np.deg2rad(desired_heading_deg)

    pid = PID(30, 0, -0.5, 100)

    while True:
        # get yaw from the vehicle
        msg = mav.recv_match(type="ATTITUDE", blocking=True)
        yaw = msg.yaw
        yaw_rate = msg.yawspeed

        print("Heading: ", np.rad2deg(yaw))

        # calculate error
        clockwise_error = (desired_heading - yaw) % (2 * np.pi)

        if (desired_heading <= np.pi and desired_heading >= 0) and (
            yaw > -np.pi and yaw < 0
        ):
            first_step = np.pi + yaw
            second_step = np.pi - desired_heading
            counter_clockwise_error = -(first_step + second_step)
        else:
            counter_clockwise_error = desired_heading - yaw

        clockwise_deg_error = np.rad2deg(clockwise_error)
        counter_clockwise_deg_error = np.rad2deg(counter_clockwise_error)

        c_e = abs(clockwise_error)
        cc_e = abs(counter_clockwise_error)

        if c_e < cc_e:
            error = clockwise_error
            output = pid.update(error, error_derivative=yaw_rate)
        elif c_e > cc_e:
            error = counter_clockwise_error
            output = pid.update(error, error_derivative=yaw_rate)

        print("Error: ", np.rad2deg(error))

        output = pid.update(error, error_derivative=yaw_rate)
        print("Output: ", output)

        # Implementation by mapping to sine

        # error = error % (2 * np.pi) - np.pi
        # if error > np.pi / 2:
        #     error = 1
        # elif error < -np.pi / 2:
        #     error = -1
        # else:
        #     error = np.sin(error)
        # output = pid.update(error, error_derivative=yaw_rate)
        set_rotation_power(mav, output)

        # if output < 0:
        #     set_rotation_power(mav, -output)
        # else:
        #     set_rotation_power(mav, output)
        # set_rotation_power(mav, output)


if __name__ == "__main__":
    main()
# init
