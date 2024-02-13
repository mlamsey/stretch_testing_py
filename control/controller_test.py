import zmq
import threading
from stretch_body.robot import Robot

def callback(robot, control_rate=50.):  # Max wait time in milliseconds
    # parse args
    max_wait_time = 1000. / control_rate
    
    # other init
    initial_wrist_effort = robot.end_of_arm.status['wrist_pitch']['effort']
    deadband = 2.0
    v = 0.03

    # set up message handling
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.bind("tcp://127.0.0.1:5555")
    socket.setsockopt_string(zmq.SUBSCRIBE, '')

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    while True:
        end_of_arm_status = robot.end_of_arm.status
        pitch_status = end_of_arm_status['wrist_pitch']
        effort = pitch_status['effort']
        # print(effort)
        if abs(effort - initial_wrist_effort) > deadband:
            # print('Wrist pitch effort exceeded deadband')
            if effort > initial_wrist_effort:
                robot.lift.set_velocity(v)
            else:
                robot.lift.set_velocity(-v)
        else:
            robot.lift.set_velocity(0.)

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
    socket.connect("tcp://127.0.0.1:5555")

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
