"""uav_control controller."""
from controller import Robot, Camera, InertialUnit, GPS, Compass, Gyro, Motor, Keyboard, Emitter, Receiver
import math
import numpy as np
import struct
import time
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(timestep)
receiver = robot.getReceiver("receiver")
receiver.enable(timestep)
imu = robot.getInertialUnit("inertial unit")
imu.enable(timestep)
camera = robot.getCamera("camera")
camera.enable(timestep)
gps = robot.getGPS("gps")
gps.enable(timestep)
compass = robot.getCompass("compass")
compass.enable(timestep)
gyro = robot.getGyro("gyro")
gyro.enable(timestep)
camera_roll_motor = robot.getCamera("camera roll")
camera_pitch_motor = robot.getCamera("camera pitch")
front_left_motor = robot.getMotor("front left propeller")
front_right_motor = robot.getMotor("front right propeller")
rear_left_motor = robot.getMotor("rear left propeller")
rear_right_motor = robot.getMotor("rear right propeller")
motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]

# arming
for i in range(4):
    motors[i].setPosition(float("inf"))
    motors[i].setVelocity(1.0)
print("arming")
k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3.0  
k_roll_p = 50.0
k_pitch_p = 30.0
target_altitude = 1.0 
roll_disturbance = 0
pitch_disturbance = 0
yaw_disturbance = 0
# target
target_z = 1
target_x = 0
target_y = 0
target_yaw = 0
def convert_to_pitch_roll(ex, ey, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array(((c, -s), (s, c)))
    exy_ = np.matmul([ex, ey], R)
    return exy_[0], exy_[1]
def convert_to_target(x, y, yaw):
    dx = 1
    dy = 1
    if yaw > 0:
        yaw = -(yaw - math.pi)
    elif yaw < 0:
        yaw = -(yaw + math.pi)
    c, s = np.cos(-yaw), np.sin(-yaw)
    R = np.array(((c, -s), (s, c)))
    dxy_ = np.matmul([dx, dy], R)
    return x + dxy_[0], y + dxy_[1]
# Main loop:

# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:

    # get data from leader
    message=receiver.getData()
    dataList=struct.unpack("5f",message)
    leader_x, learder_y, leader_z, leader_yaw, leader_yaw_input = dataList[0], dataList[1], dataList[2], dataList[3], dataList[4]
    receiver.nextPacket()
    
    target_x, target_y = convert_to_target(leader_x, learder_y, leader_yaw)
    target_z = leader_z
    # read sensor
    roll = imu.getRollPitchYaw()[0] + math.pi / 2.0
    pitch = imu.getRollPitchYaw()[1]
    yaw = imu.getRollPitchYaw()[2]
    altitude = gps.getValues()[1]
    px = gps.getValues()[0]
    py = gps.getValues()[2]
    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]
        
    # Compute the roll, pitch, yaw and vertical inputs.
    pitch_err, roll_err = convert_to_pitch_roll(px - target_x, target_y - py, yaw)
    roll_input = k_roll_p * np.clip(roll, -1.0, 1.0) + roll_acceleration - roll_err
    pitch_input = k_pitch_p * np.clip(pitch, -1.0, 1.0) - pitch_acceleration - pitch_err
    yaw_input = leader_yaw_input
    clamped_difference_altitude = np.clip(target_z - altitude + k_vertical_offset, -1.0, 1.0)
    vertical_input = k_vertical_p * math.pow(clamped_difference_altitude, 3.0)
    # Actuate the motors taking into consideration all the computed inputs.
    front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
    
    front_left_motor.setVelocity(front_left_motor_input)
    front_right_motor.setVelocity(-front_right_motor_input)
    rear_left_motor.setVelocity(-rear_left_motor_input)
    rear_right_motor.setVelocity(rear_right_motor_input)

    pass

# Enter here exit cleanup code.
