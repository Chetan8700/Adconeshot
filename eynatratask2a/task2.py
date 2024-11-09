import time

# Initialize PID parameters
Kp = 1200   # Proportional gain
Ki = 1.5    # Integral gain
Kd = 350    # Derivative gain

# Control limits
max_signal = 2500  # Maximum control signal value to prevent excessive movement
dead_zone = 0.1  # Dead zone around zero to prevent small oscillations

# Rotation control parameters
rotation_factor = 200  # Stronger factor for left/right rotation

# PID variables
prev_error = 0.0
integral = 0.0
error = 0.0  # To store error globally
up = 0.0
close = 0.0

keyPressed = None  # Initialize keyPressed to None

# Define mock functions for the bot and joints (replace with actual hardware API)
def get_bot_angle():
    alpha, beta, gamma = sim.getObjectOrientation(bot_handle, -1)
    yaw, pitch, roll = sim.alphaBetaGammaToYawPitchRoll(alpha, beta, gamma)
    return roll, yaw  # We use roll as the angle for balancing

def apply_wheel_actuation(control_signal, rotation_signal):
    global keyPressed, up, arm_joint, gripper_joint, gripper_joint2, close

    # Separate rotation and balance control for each wheel
    left_wheel_signal = control_signal - rotation_signal
    right_wheel_signal = control_signal + rotation_signal

    # Debugging output to check the final signals
    print(f"Control Signal: {control_signal}, Rotation Signal: {rotation_signal}")
    print(f"Left Wheel Signal: {left_wheel_signal}, Right Wheel Signal: {right_wheel_signal}")

    # Apply signals to wheels
    sim.setJointTargetVelocity(wheel_left, left_wheel_signal)
    sim.setJointTargetVelocity(wheel_right, right_wheel_signal)
    sim.setJointTargetVelocity(arm_joint, up)
    sim.setJointTargetVelocity(gripper_joint, close)

def sysCall_init():
    global bot_handle, wheel_left, wheel_right, Kp, Ki, Kd, prev_error, integral, error, keyPressed, arm_joint, gripper_joint

    # Initialize the scene objects, bot, wheels, and joints
    sim = require('sim')
    bot_handle = sim.getObjectHandle('/body/SBR_Assembly_7')
    wheel_left = sim.getObjectHandle('/body/left_joint')
    wheel_right = sim.getObjectHandle('/body/right_joint')
    arm_joint = sim.getObjectHandle('/body/SBR_Assembly_7/ForceSensor/arm_base/arm_joint')
    gripper_joint = sim.getObjectHandle('/body/SBR_Assembly_7/ForceSensor/arm_base/arm_joint/grip_base/Prismatic_joint')

    # Initialize PID variables
    prev_error = 0.0
    integral = 0.0
    error = 0.0
    keyPressed = [None, [0]]

def sysCall_sensing():
    global prev_error, integral, error, keyPressed, up, close

    # Get current orientation
    angle, yaw = get_bot_angle()
    target_angle = 0.0
    up = 0.0
    close = 0.0
    keyPressed = sim.getSimulatorMessage()  # Read the current key

    # Determine the target angle based on key press
    if keyPressed and keyPressed[1][0] == 2007:  # Up arrow
        target_angle = -0.4
    elif keyPressed and keyPressed[1][0] == 2008:  # Down arrow
        target_angle = 0.4
    else:
        target_angle = 0.0

    # Control the prismatic joint and arm joint based on key presses
    if keyPressed and keyPressed[1][0] == 119:  # 'w' key
        up = -1.0
    elif keyPressed and keyPressed[1][0] == 115:  # 's' key
        up = 1.0

    if keyPressed and keyPressed[1][0] == 113:  # 'q' key
        close = 0.5
    elif keyPressed and keyPressed[1][0] == 101:  # 'e' key
        close = -0.5

    # Calculate error for PID control
    error = target_angle - angle
    integral += error  # Integral term
    derivative = error - prev_error  # Derivative term

    # Store previous error for next iteration
    prev_error = error

def sysCall_actuation():
    global error, integral, prev_error, keyPressed

    # PID control law with clamping for control signal
    control_signal = Kp * error + Ki * integral + Kd * (error - prev_error)
    control_signal = max(min(control_signal, max_signal), -max_signal)  # Clamp the control signal

    # Apply a dead zone to prevent small oscillations
    if abs(control_signal) < dead_zone:
        control_signal = 0.0

    # Rotation signal based on left/right keys
    rotation_signal = 0.0
    if keyPressed and keyPressed[1][0] == 2009:  # Left arrow
        rotation_signal = -rotation_factor
    elif keyPressed and keyPressed[1][0] == 2010:  # Right arrow
        rotation_signal = rotation_factor

    # Apply control signal to the wheels (actuation)
    apply_wheel_actuation(control_signal, rotation_signal)

def sysCall_cleanup():
    # Perform any necessary cleanup here
    pass

# Main loop that simulates the CoppeliaSim environment
if __name__ == "__main__":
    # Initialize the connection to the simulator
    sim.simxStart(19997, True)
    sysCall_init()

    while True:
        sysCall_sensing()  # Get sensor data and calculate control actions
        sysCall_actuation()  # Apply actuation based on control logic
        time.sleep(0.05)  # Simulate a delay (adjust based on simulation speed)
