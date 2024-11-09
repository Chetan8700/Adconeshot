import time

# Initialize PID parameters
Kp = 1450  # Proportional gain
Ki = 2.00  # Integral gain
Kd = 400   # Derivative gain

# PID variables
prev_error = 0.0
integral = 0.0
error = 0.0  # To store error globally
up = 0.0
close = 0.0

# Bias values for left and right wheels to compensate for weight imbalance
left_bias = 0.2
right_bias = 0.3

keyPressed = None  # Initialize keyPressed to None

# Define mock functions for the bot and joints (replace with actual hardware API)
def get_bot_angle():
    # Custom function to retrieve the bot's angle
    # In the simulation, we would get the bot's orientation here
    alpha, beta, gamma = sim.getObjectOrientation(bot_handle, -1)
    yaw, pitch, roll = sim.alphaBetaGammaToYawPitchRoll(alpha, beta, gamma)
    return roll, yaw  # We use roll as the angle for balancing

def apply_wheel_actuation(control_signal):
    global keyPressed, up, arm_joint, gripper_joint, close

    # Adjust control signal with bias to compensate for weight imbalance
    left_control_signal = control_signal + left_bias
    right_control_signal = control_signal - right_bias

    print(f"keyPressed[1][0]: {keyPressed[1][0]}")  # Debugging line to check the pressed key
    if keyPressed[1][0] == 2009:  # Left arrow or equivalent
        sim.setJointTargetVelocity(wheel_left, left_control_signal)  # Apply to left wheel
        sim.setJointTargetVelocity(wheel_right, -right_control_signal)  # Apply to right wheel
    elif keyPressed[1][0] == 2010:  # Right arrow or equivalent
        sim.setJointTargetVelocity(wheel_left, -left_control_signal)  # Apply to left wheel
        sim.setJointTargetVelocity(wheel_right, right_control_signal)  # Apply to right wheel
    else:
        sim.setJointTargetVelocity(wheel_left, left_control_signal)  # Apply to left wheel
        sim.setJointTargetVelocity(wheel_right, right_control_signal)  # Apply to right wheel
        sim.setJointTargetVelocity(arm_joint, up)
        sim.setJointTargetVelocity(gripper_joint, close)

def apply_joint_velocity(joint_name, velocity):
    # Mock function: Control the velocity of a specific joint (e.g., arm or prismatic joint)
    print(f"{joint_name} Joint Velocity: {velocity}")

def alternative_key_check():
    print(f"Checking for keyPressed: {keyPressed[1][0]}")  # Debugging line to print the current key press value
    if keyPressed[1][0] == 113:  # 'q' key
        return "prismatic_increase"  # Move prismatic joint in one direction
    elif keyPressed[1][0] == 101:  # 'e' key
        return "prismatic_decrease"  # Move prismatic joint in the opposite direction
    elif keyPressed[1][0] == 97:  # 'a' key
        return "arm_increase"  # Move arm joint in one direction
    elif keyPressed[1][0] == 100:  # 'd' key
        return "arm_decrease"  # Move arm joint in the opposite direction
    else:
        return None  # No key press detected

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

    # Get current orientation (e.g., from an IMU sensor or directly from the bot's orientation)
    angle, yaw = get_bot_angle()
    target_angle = 0.0
    up = 0.0
    close = 0.0
    keyPressed = sim.getSimulatorMessage()  # Read the current key
    print(f"keyPressed[1][0]: {keyPressed[1][0]}")  # Debugging line to check the pressed key
    
    # Determine the target angle based on key press
    if keyPressed[1][0] == 2007:  # Up arrow
        target_angle = -0.4
    elif keyPressed[1][0] == 2008:  # Down arrow
        target_angle = 0.4
    else:
        target_angle = 0.0

    # Control the prismatic joint and arm joint based on key presses
    if keyPressed[1][0] == 119:  # 'w' key
        up = -1.0
    elif keyPressed[1][0] == 115:  # 's' key
        up = 1.0

    if keyPressed[1][0] == 113:  # 'q' key
        close = 0.5
    elif keyPressed[1][0] == 101:  # 'e' key
        close = -0.5

    # Calculate error for PID control
    error = target_angle - angle
    integral += error  # Integral term
    derivative = error - prev_error  # Derivative term

    # Store previous error for next iteration
    prev_error = error

def sysCall_actuation():
    global error, integral, prev_error, keyPressed

    # PID control law
    control_signal = Kp * error + Ki * integral + Kd * (error - prev_error)
    # Apply control signal to the wheels (actuation)
    apply_wheel_actuation(control_signal)

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
