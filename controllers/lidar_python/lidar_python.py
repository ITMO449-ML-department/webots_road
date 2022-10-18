#!/usr/bin/env python3

"""lidar_python controller."""
from controller import Robot
from controller import Motor
from controller import LidarPoint
import numpy as np


robot = Robot()


lidar = robot.getDevice("Lidar")
gps = robot.getDevice("gps")
motor_l = robot.getDevice("left wheel motor")
motor_r = robot.getDevice("right wheel motor")
sensor_l = robot.getDevice("left wheel sensor")
sensor_r = robot.getDevice("right wheel sensor")


timestep = 1 #int(robot.getBasicTimeStep())


lidar.enable(timestep)
gps.enable(timestep)
sensor_l.enable(timestep)
sensor_r.enable(timestep)
motor_l.setPosition(float('inf'))
motor_r.setPosition(float('inf'))


wheel_R = 0.0201
dist_between = 0.052
odometry_state = np.array([0, 0, 0],dtype=float)
pos_prev = [0, 0]

def odometry_diff():
    global pos_prev
    pos_l = sensor_l.getValue()
    pos_r = sensor_r.getValue()
    
    diff_l = (pos_l - pos_prev[0])*wheel_R
    diff_r = (pos_r - pos_prev[1])*wheel_R
    
    dtheta = (diff_r - diff_l)/dist_between
    dx = (diff_l + diff_r)/2.0 * np.cos(odometry_state[2]+dtheta)
    dy = (diff_l + diff_r)/2.0 * np.sin(odometry_state[2]+dtheta)
    pos_prev = [pos_l, pos_r]
    print(dx,dy,dtheta)
    return np.array([dx,dy,dtheta])


while robot.step(timestep) != -1:
    gps_data = np.array(gps.getValues())
    # print(odometry_state[-1]/np.pi*180)
    # print(np.sqrt(np.sum((gps_data[:-1]-odometry_state[:-1])**2)))
    # print(gps_data[:-1],odometry_state)
    motor_l.setVelocity(-5)
    motor_r.setVelocity(-4)
    
    odometry_state += odometry_diff()

    lidar_data = np.array(lidar.getRangeImage())
    
