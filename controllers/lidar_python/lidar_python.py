#!/usr/bin/env python3

"""lidar_python controller."""
from controller import Robot
from controller import Motor
from controller import LidarPoint
import numpy as np


robot = Robot()

map_ = np.zeros((5000,5000))

def add_obstacle(x, y, a, b):
    global map_
    x, y, a, b = map(int, [2500 + x*1000, 2500 - y*1000, a*500, b*500])
    # print(x, y, a, b)
    map_[y-b:y+b,x-a:x+a] = 1
    
add_obstacle(0,-1,0.01,0.5)
add_obstacle(-0.5,-0.75,1,0.01)
add_obstacle(0.5,-0.75,1,0.01)
add_obstacle(-1.5,-0.75,1,0.01)
add_obstacle(-2,0.5,0.01,2.5)
add_obstacle(0,-2,0.01,1)
add_obstacle(0,1.5,0.01,2)
add_obstacle(1,-1,0.01,0.5)
add_obstacle(0,0.5,3,0.01)
add_obstacle(1.5,1.5,0.5,0.5)

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


wheel_R = 0.02
dist_between = 0.052
odometry_state = np.array([0, 0, 0],dtype=float)
pos_prev = [0, 0]

def odometry_diff():
    global pos_prev
    pos_l = sensor_l.getValue()
    pos_r = sensor_r.getValue()
    
    diff_l = (pos_l - pos_prev[0])*wheel_R
    diff_r = (pos_r - pos_prev[1])*wheel_R
    
    dtheta = np.arctan((diff_r - diff_l)/dist_between)
    dx = (diff_l + diff_r)/2.0 * np.cos(odometry_state[2]+dtheta)
    dy = (diff_l + diff_r)/2.0 * np.sin(odometry_state[2]+dtheta)
    pos_prev = [pos_l, pos_r]
    # print(dx,dy,dtheta)
    return np.array([dx,dy,dtheta])

gps_data = np.zeros(2)
gps_data_prev = np.zeros(2)
angle = 0
angle_prev = 0

while robot.step(timestep) != -1:
    gps_data = np.array(gps.getValues())[:-1]
    
    diff = gps_data - gps_data_prev
    angle = np.arctan2(np.array(diff[1]),np.array(diff[0]))
    
    dangle = angle - angle_prev
    rho = np.sqrt(np.sum(diff**2))
    
    motor_l.setVelocity(5)
    motor_r.setVelocity(5)
    
    print(angle*180/np.pi)
    
    # print(gps_data[0]/((sensor_l.getValue()+sensor_r.getValue())/2))
    # print(odometry_state[-1]/np.pi*180)
    # print(np.sqrt(np.sum((gps_data[:-1]-odometry_state[:-1])**2)))
    # print(gps_data[:-1],odometry_state)
    # odometry_state += odometry_diff()

    lidar_data = np.array(lidar.getRangeImage())
    gps_data_prev[:] = gps_data[:]
    agle_prev = angle
    
