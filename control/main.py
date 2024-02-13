from controller_test import main as lift_control
from follow_me import main as follow_me

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Control the lift joint based on the effort of the wrist_pitch joint.')
    
    # parse args
    parser = argparse.ArgumentParser(description='Control the lift joint based on the effort of the wrist_pitch joint.')
    parser.add_argument('--demo', type=str, default='lift_control', help='The demo to run.')
    parser.add_argument('--control-rate', type=float, default=50., help='The control rate in Hz.')
    args = parser.parse_args()
    demo = args.demo
    control_rate = args.control_rate

    if demo == 'lift_control':
        lift_control(control_rate)
    elif demo == 'follow_me':
        follow_me(control_rate)
    else:
        raise ValueError("Invalid demo: {}".format(demo))