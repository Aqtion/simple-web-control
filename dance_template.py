from motor_test import test_motor
import time
from pymavlink import mavutil
import socket
import numpy as np


def arm_rov(mav_connection):
    """
    Arm the ROV, wait for confirmation
    """
    mav_connection.arducopter_arm()
    print("Waiting for the vehicle to arm")
    mav_connection.motors_armed_wait()
    print("Armed!")


def disarm_rov(mav_connection):
    """
    Disarm the ROV, wait for confirmation
    """
    mav_connection.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    mav_connection.motors_disarmed_wait()
    print("Disarmed!")


def run_motors_timed(mav_connection, seconds: int, motor_settings: list) -> None:
    """
    Run the motors for a set time
    :param mav_connection: The mavlink connection
    :param time: The time to run the motors
    :param motor_settings: The motor settings, a list of 6 values -100 to 100
    :return: None
    """
    step = 0
    while step < seconds:
        for i in range(len(motor_settings)):
            test_motor(
                mav_connection=mav_connection, motor_id=i, power=motor_settings[i]
            )
        time.sleep(0.2)
        step += 0.2


if __name__ == "__main__":
    ####
    # Initialize ROV
    ####
    mav_connection = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    mav_connection.wait_heartbeat()
    # Arm the ROV and wait for confirmation

    ####
    # Run choreography
    ####
    """
    Call sequence of calls to run_timed_motors to execute choreography
    Motors power ranges from -100 to 100
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("", 8400))
        s.listen(5)
        conn, addr = s.accept()
        while True:
            command = conn.recv(1024).decode()
            if command.strip() == "arm".lower():
                arm_rov(mav_connection)
            elif command.strip() == "disarm".lower():
                disarm_rov(mav_connection)
            else:
                try:
                    arr = np.array(command.split(" ")).astype(float)
                except:
                    conn.send("Invalid input received\n".encode())
                    continue

                if len(arr) != 7:
                    conn.send("Invalid # of parameters received\n".encode())
                    continue

                powers = arr[0 : len(arr)]
                if arr[-1] <= 0:
                    conn.send("Time must be positive\n".encode())
                    continue

                print("Powers: ", powers)
                print("Time: ", arr[-1])

                run_motors_timed(mav_connection, arr[-1], powers.tolist())
    ####
    # Disarm ROV and exit
    ####
