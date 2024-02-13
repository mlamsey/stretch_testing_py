import zmq
import threading
import numpy as np
from typing import List
from stretch_body.robot import Robot
import json

SOCKET_IP = "tcp://127.0.0.1:5555"

def compute_velocity(error_vector: List[float], deadband: float=2.0, Kp: List[float]=[0.02, 0.1]) -> List[float]:
    """
    Computes base velocity based on effort values.
    Base velocity is based on wrist yaw and arm effort.
    
    Args:
        error_vector (List[float]): Applied effort to the robot [wrist yaw, wrist_pitch, wrist_roll, arm].
        deadband (float): The range of effort values that will result in no velocity command.
        Kp (List[float]): Proportional gains for the controller (translation, rotation).

    Returns:
        List[float]: [translation, rotation] to be sent to the base
    """

    # init
    v = 0
    w = 0

    # unpack state error
    yaw_effort, _, _, arm_effort = error_vector

    # unpack gains
    Kp_v, Kp_w = Kp

    if abs(yaw_effort) > deadband:
        v = Kp_v * yaw_effort

    if abs(arm_effort) > deadband:
        w = Kp_w * arm_effort

    # print(error_vector, [v, w])
    return [v, w]
    
def open_socket_listener() -> tuple:
    """
    Opens a zmq socket to listen for messages.

    Returns:
        tuple: A tuple containing the socket and poller objects.
    """

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.bind(SOCKET_IP)
    socket.setsockopt_string(zmq.SUBSCRIBE, '')

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)
    return socket, poller

def get_wrist_effort(robot: Robot) -> np.ndarray:
    """
    Gets the wrist effort and returns the initial wrist effort and the initial lift velocity.

    Args:
        robot (Robot): The Stretch object.

    Returns:
        np.ndarray: An array containing the wrist joint effort [yaw, pitch, roll]
    """

    wrist_yaw_effort = robot.end_of_arm.status['wrist_yaw']['effort']
    wrist_pitch_effort = robot.end_of_arm.status['wrist_pitch']['effort']
    wrist_roll_effort = robot.end_of_arm.status['wrist_roll']['effort']
    return [wrist_yaw_effort, wrist_pitch_effort, wrist_roll_effort]

def get_arm_effort(robot: Robot) -> float:
    """
    Gets the arm effort and returns the initial arm effort.

    Args:
        robot (Robot): The Stretch object.

    Returns:
        float: The arm effort.
    """

    # print(json.dumps(robot.arm.status, indent=4))
    return robot.arm.status['motor']['effort_ticks'] / 10.  # hack bc force query not online

def get_state_vector(robot: Robot) -> np.ndarray:
    """
    Gets the state vector and returns the initial state vector.

    Args:
        robot (Robot): The Stretch object.

    Returns:
        np.ndarray: An array containing the state vector [wrist_yaw, wrist_pitch, wrist_roll, arm]
    """

    wrist_effort = get_wrist_effort(robot)
    arm_effort = get_arm_effort(robot)
    return np.concatenate([wrist_effort, [arm_effort]])

def follow_me(robot: Robot, control_rate: float=50.):  # Max wait time in milliseconds
    """
    Controller callback.
    Listens for messages from main thread.
    Controls the robot's base based on wrist effort.

    Args:
        robot (Robot): The Stretch object.
        control_rate (float): The control rate in Hz.
    """

    # parse args
    max_wait_time = 1000. / control_rate

    # hold arm position
    arm_pos = robot.arm.status['pos']
    robot.arm.move_to(arm_pos)

    # other init
    initial_state = get_state_vector(robot)

    socket, poller = open_socket_listener()

    while True:
        effort = get_state_vector(robot)
        e = effort - initial_state
        
        # compute and command velocity
        v = compute_velocity(e)
        robot.base.set_velocity(*v)
        robot.push_command()
        
        # Check for messages and sleep
        socks = dict(poller.poll(max_wait_time))
        if socket in socks and socks[socket] == zmq.POLLIN:
            message = socket.recv_string()
            if message:
                print("Received:", message)
                if message == 'q':
                    break

def main(control_rate=50.):
    """
    Entry point for lift control demo.
    Moves lift proportional to wrist_pitch effort with velocity control.
    
    Args:
        control_rate (float): The control rate in Hz.
    """
    
    # set up robot
    robot = Robot()
    robot.startup()
    
    # # Start the callback on a separate thread
    # thread = threading.Thread(target=follow_me, args=(robot, control_rate))
    # thread.start()

    # Send messages using PyZMQ
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.connect(SOCKET_IP)

    print('e')

    # Start the callback on a separate thread
    thread = threading.Thread(target=follow_me, args=(robot, control_rate))
    thread.start()

    # UI loop
    while True:
        message = input("Enter message to send (type 'q' to quit): ")
        socket.send_string(message)
        if message.lower() == 'q':
            robot.base.set_velocity(0, 0)
            robot.stop()
            break

    # Wait for the thread to finish
    thread.join()

if __name__ == "__main__":
    main()
