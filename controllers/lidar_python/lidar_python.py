#!/usr/bin/env python3

"""lidar_python controller."""
from controller import Robot
from controller import Motor
from controller import LidarPoint
import numpy as np
import scipy
from matplotlib import pyplot as plt
import time


robot = Robot()

map_ = np.ones((5000, 5000))


def add_obstacle(x, y, a, b):
    global map_
    x, y, a, b = map(int, [2500 + x*1000, 2500 - y*1000, a*500, b*500])
    map_[y-b:y+b, x-a:x+a] = 0


# Map creation
add_obstacle(0, -1, 0.01, 0.5)
add_obstacle(-0.5, -0.75, 1, 0.01)
add_obstacle(0.5, -0.75, 1, 0.01)
add_obstacle(-1.5, -0.75, 1, 0.01)
add_obstacle(-2, 0.5, 0.01, 2.5)
add_obstacle(0, -2, 0.01, 1)
add_obstacle(0, 1.5, 0.01, 2)
add_obstacle(1, -1, 0.01, 0.5)
add_obstacle(0, 0.5, 3, 0.01)
add_obstacle(1.5, 1.5, 0.5, 0.5)

# Robot sensors initialization
lidar = robot.getDevice("Lidar")
gps = robot.getDevice("gps")
motor_l = robot.getDevice("left wheel motor")
motor_r = robot.getDevice("right wheel motor")
sensor_l = robot.getDevice("left wheel sensor")
sensor_r = robot.getDevice("right wheel sensor")

timestep = 1

lidar.enable(timestep)
gps.enable(timestep)
sensor_l.enable(timestep)
sensor_r.enable(timestep)
motor_l.setPosition(float('inf'))
motor_r.setPosition(float('inf'))


# Odometry and lidar modeling
lidar_points_num = 3

wheel_R = 0.02
dist_between = 0.052
odometry_state = np.array([0, 0, 0], dtype=float)
pos_prev = [0, 0]


def procceed_lidar():  # Retuns lidar_points_num distances
    lidar_data = np.array(lidar.getRangeImage())
    indexes = np.linspace(
        0, lidar_data.shape[0]-1, lidar_points_num, dtype=int)
    return lidar_data[indexes]


def is_obst(x, y):  # checks if there an obstacle
    x, y = map(int, [2500 - y*1000, 2500 + x*1000])
    return x < 0 or x >= map_.shape[0] or y < 0 or y >= map_.shape[1] or map_[x, y] == 0

# takes robot state vector and lidar range (4m)
# returns lidar model data


def modelate_lidar(x, y, angle, ran):
    ang = np.linspace(angle + 120/180*np.pi, angle-120/180 *
                      np.pi, lidar_points_num)  # vision field
    output = np.zeros_like(ang)
    ang[ang < -np.pi] += 2*np.pi
    ang[ang > -np.pi] -= 2*np.pi
    for i, a in enumerate(ang):
        x2, y2 = x + ran*np.cos(a), y + ran*np.sin(a)
        steps = 10**3
        for step in range(steps):
            x_s = x + step*(x2-x)/steps
            y_s = y + step*(y2-y)/steps
            if is_obst(x_s, y_s):
                output[i] = np.sqrt((x_s-x)**2 + (y_s-y)**2)
                break
        else:
            output[i] = np.inf
    return output


def odometry_diff():  # calculates state difference with odometry
    global pos_prev
    pos_l = sensor_l.getValue()
    pos_r = sensor_r.getValue()

    diff_l = (pos_l - pos_prev[0])*wheel_R
    diff_r = (pos_r - pos_prev[1])*wheel_R

    dtheta = np.arctan((diff_r - diff_l)/dist_between)
    dx = (diff_l + diff_r)/2.0 * np.cos(odometry_state[2]+dtheta)
    dy = (diff_l + diff_r)/2.0 * np.sin(odometry_state[2]+dtheta)
    pos_prev = [pos_l, pos_r]
    return np.array([dx, dy, dtheta])


# Create sample
particle_num = 1000
sample = np.zeros((particle_num, 3))
sample[:, 0] = np.random.uniform(-2.5, 2.5, (particle_num))[:]
sample[:, 1] = np.random.uniform(-2.5, 2.5, (particle_num))[:]
sample[:, 2] = np.random.uniform(-np.pi, np.pi, (particle_num))[:]


def resample_from_index(particles, weights, indexes):
    particles[:] = particles[indexes]
    weights.resize(len(particles))
    weights.fill(1.0 / len(weights))


def simple_resample(particles, weights):
    global particle_num
    N = len(particles)
    arr_num = particle_num * weights/weights.sum() 
    # print(arr_num.sum())
    new_points = np.random.normal(particles[0],0.5,(int(arr_num[0]),3))
    for i in range(1,particle_num):
        new_points = np.concatenate([new_points,np.random.normal(particles[i],0.5,(int(arr_num[i]),3))])
    print(new_points.shape)

    particle_num = new_points.shape[0]
    weights.fill(1.0 / N)
    return new_points


def estimate(particles, weights):
    """returns mean and variance of the weighted particles"""

    pos = particles[:, 0:3]
    mean = np.average(pos, weights=weights, axis=0)
    var = np.average((pos - mean)**2, weights=weights, axis=0)
    return mean, var


est_pos = []
t = time.time()
cicle_counter = 0
while robot.step(timestep) != -1:
    cicle_counter += 1
    motor_l.setVelocity(5)
    motor_r.setVelocity(5)
    # Calculate odometry difference
    dvec = odometry_diff()
    # Change states
    sample += dvec
    sample[:,2][sample[:,2] < -np.pi] += 2*np.pi
    sample[:,2][sample[:,2] > np.pi] -= 2*np.pi
    # Lidar
    sample_lidar = np.zeros((particle_num, lidar_points_num))
    for i in range(particle_num):
        sample_lidar[i, :] = modelate_lidar(*sample[i], 4)
    real_lidar = procceed_lidar()
    if cicle_counter % 5 == 0:
        # Calculate probs
        probs = scipy.stats.norm(np.zeros(
            (1, lidar_points_num)), 0.35).pdf(sample_lidar-real_lidar.reshape((1, -1)))
        probs = np.sum(probs, axis=1)
        probs = probs / np.sum(probs)
        # Resample
        mu, var = estimate(sample, probs)
        sample =  simple_resample(sample, probs)
        est_pos.append(mu)
        # add_obstacle(mu[0], mu[1], 0.1, 0.1)
        # plt.imshow(map_)
        # plt.plot(np.array(est_pos)[:, 0], np.array(est_pos)[:, 1])
        # plt.savefig(f"../plots/plt({cicle_counter}).png")
        plt.clf()
        plt.title(sample.shape)
        plt.xlim([-2.5,2.5])
        plt.ylim([-2.5,2.5])
        plt.scatter(sample[:, 0], sample[:, 1],  s=0.5, label="probs")
        plt.legend()
        x, y, z = gps.getValues()
        plt.scatter(x,y, c='r')
        plt.savefig(f"../plots/probs({cicle_counter}).png")
        t = time.time()
        print("Saved!")
    # print(mu, var)
