"""lidar_python controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import LidarPoint

import sys
print("Python version")
print (sys.path)

import numpy as np

# create the Robot instance.
robot = Robot()
print(robot.getNumberOfDevices())
for i in range(robot.getNumberOfDevices()):
    a = robot.getDeviceByIndex(i)
    print(i, a, a.getName())
lidar = robot.getDevice("Lidar")
gps = robot.getDevice("gps")
motor_l = robot.getDevice("motor.left")
motor_r = robot.getDevice("motor.right")

# print(motor)
print('hi')

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

lidar.enable(timestep)
motor_l.setPosition(float('inf'))
motor_r.setPosition(float('inf'))

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
def move_forward(u):
    motor_l.setVelocity(u)
    motor_r.setVelocity(u)


def turn_right(u):
    motor_l.setVelocity(u)
    motor_r.setVelocity(-u)

# Main loop:
i = 1
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    a = np.array(lidar.getRangeImage())
    # print(a[a.shape[0]//2])
    # motor_bl.setVelocity(1)
    # motor_br.setVelocity(1)
    # motor_fl.setVelocity(1)
    # motor_fr.setVelocity(1)
    turn_right(5)
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
