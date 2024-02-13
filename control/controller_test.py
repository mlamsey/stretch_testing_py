import zmq
import threading
from stretch_body.robot import Robot

SOCKET_IP = "tcp://127.0.0.1:5555"

def compute_velocity(effort_error: float, deadband: float=2.0, Kp: float=0.01) -> float:
    """
    Computes lift velocity based on an effort value.
    For this demo, recommended to use wrist_pitch effort.
    
    Args:
        effort_error (float): The difference between the current effort and the desired effort.
        deadband (float): The range of effort values that will result in no velocity command.
        Kp (float): Proportional gain for the controller.

    Returns:
        float: The velocity command to be sent to the lift joint.
    """

    if abs(effort_error) > deadband:
        return Kp * effort_error
    else:
        return 0.
    
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

def callback(robot: Robot, control_rate: float=50.):  # Max wait time in milliseconds
    """
    Controller callback.
    Listens for messages from main thread.
    Controls the lift joint based on the effort of the wrist_pitch joint.

    Args:
        robot (Robot): The Stretch object.
        control_rate (float): The control rate in Hz.
    """

    # parse args
    max_wait_time = 1000. / control_rate
    
    # other init
    initial_wrist_effort = robot.end_of_arm.status['wrist_pitch']['effort']

    socket, poller = open_socket_listener()

    while True:
        # poll end of arm status
        end_of_arm_status = robot.end_of_arm.status
        pitch_status = end_of_arm_status['wrist_pitch']
        effort = pitch_status['effort']
        
        # compute and command velocity
        e = effort - initial_wrist_effort
        v = compute_velocity(e)
        robot.lift.set_velocity(v)
        robot.push_command()
        
        # Check for messages and sleep
        socks = dict(poller.poll(max_wait_time))
        if socket in socks and socks[socket] == zmq.POLLIN:
            message = socket.recv_string()
            if message:
                print("Received:", message)
                if message == 'q':
                    break

def main():
    # set up robot
    robot = Robot()
    robot.startup()
    
    # Start the callback on a separate thread
    thread = threading.Thread(target=callback, args=(robot, 50.))
    thread.start()

    # Send messages using PyZMQ
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.connect(SOCKET_IP)

    # UI loop
    while True:
        message = input("Enter message to send (type 'q' to quit): ")
        socket.send_string(message)
        if message.lower() == 'q':
            robot.stop()
            break

    # Wait for the thread to finish
    thread.join()

if __name__ == "__main__":
    main()
