# Stretch Testing
Python code for testing functionality of a Hello Robot Stretch with a dex wrist.

## Control Demos

Two demos so far: lift velocity control proportional to wrist pitch effort, and a hand-holding follow the leader based on wrist yaw and arm effort.

Entry point: `controls/main.py`

Args:
```
--demo [lift_control, follow_me]: demo to run
--control_rate [50]: control rate in Hz
```
