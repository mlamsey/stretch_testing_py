import zmq
import threading
from stretch_body.robot import Robot

# demos
from controller_test import lift_control
from follow_me import follow_me

SOCKET_IP = "tcp://127.0.0.1:5555"

def main(demo_handle, control_rate=50.):
    """
    Entry point for control demos.
    Invokes the demo handle and loads it onto a thread.
    Communication (stop demo via keyboard input) handled with sockets.
    
    Args:
        demo_handle (function handle): robot demo to run on a separate thread
        control_rate (float): The control rate in Hz.
    """
    
    # set up robot
    robot = Robot()
    robot.startup()
    
    # Start the callback on a separate thread
    thread = threading.Thread(target=demo_handle, args=(robot, SOCKET_IP, control_rate))
    thread.start()

    # Send messages using PyZMQ
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.connect(SOCKET_IP)

    # UI loop
    while True:
        message = input("Enter message to send to demo (type 'q' to quit): ")
        socket.send_string(message)
        if message.lower() == 'q':
            robot.base.set_velocity(0, 0)
            robot.stop()
            break

    # Wait for the thread to finish
    thread.join()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Run control demos.')
    
    # parse args
    parser.add_argument('--demo', type=str, default='lift_control', help='The demo to run.')
    parser.add_argument('--control-rate', type=float, default=50., help='The control rate in Hz.')
    args = parser.parse_args()
    demo = args.demo
    control_rate = args.control_rate

    if demo == 'lift_control':
        print("Running lift control demo.")
        main(lift_control, control_rate)
    elif demo == 'follow_me':
        print("Running follow me demo.")
        main(follow_me, control_rate)
    else:
        raise ValueError("Invalid demo: {}".format(demo))
