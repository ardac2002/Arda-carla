#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example of automatic vehicle control from client side."""

from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import random
import re
import sys
import weakref

import subprocess



try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import time
import carla
from carla import ColorConverter as cc
from open3d import *
import pandas

from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.roaming_agent import RoamingAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

j = 0
k = 0
cloud = open3d.geometry.PointCloud()
print(cloud)

# Setting fps of the server
__fps = 1800
subprocess.check_output(f"python3 ../util/config.py --fps {__fps}", shell=True)



# Called every time a measurement is taken
def lidar_callback(data):
    global j
    global cloud
    global k
    j = j + 1

    q = tesla.get_velocity()
    print(q)
    data.save_to_disk('output/lidar/tests{x}.ply'.format(x = 1))
    new_cloud = open3d.io.read_point_cloud('output/lidar/tests{x}.ply'.format(x = 1))
    cloud = cloud + new_cloud
    
    if j == 180:
        k = k + 1
        open3d.io.write_point_cloud("output/lidar/fresh{x}.ply".format(x=k), cloud)
        cloud = open3d.geometry.PointCloud()
        j = 0
        print(1)


    #lidar_data = np.frombuffer(data.raw_data, dtype=np.float32).reshape([-1, 3])


client = carla.Client('localhost', 2000)
print(client)
client.set_timeout(10)
world = client.get_world()

vf = 1

weather = carla.WeatherParameters(cloudiness = 0, precipitation = 0, precipitation_deposits = 0, sun_altitude_angle = 80, wind_intensity = 1)
world.set_weather(weather)
print(world.get_weather())

settings = world.get_settings()
#settings.fixed_delta_seconds = 0.05
settings.synchronous_mode = True
#print(settings)
world.apply_settings(settings)




# Spawning vehicle
blueprint_library = world.get_blueprint_library()
tesla_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
tesla_bp = blueprint_library.find('vehicle.tesla.model3')
spawn_points = world.get_map().get_spawn_points()
transform = carla.Transform(carla.Location(x=230, y=195, z=40), carla.Rotation(yaw=180))
tesla = world.spawn_actor(tesla_bp, spawn_points[3])
vector = carla.Vector3D(18*vf, 0, 0)
#print(vector)
tesla.enable_constant_velocity(vector)
cam_bp = blueprint_library.find('sensor.camera.rgb')
cam_bp.set_attribute('sensor_tick', '0.5')
cam_bp.set_attribute('fov', '120' )
relative_tra = carla.Transform(carla.Location(z=1.5)+carla.Location(x=1))
camera = world.spawn_actor(cam_bp, relative_tra, attach_to=tesla)
spectator = world.get_spectator()
transform = tesla.get_transform()
spectator.set_transform(carla.Transform(transform.location + carla.Location(z=5)+carla.Location(x=0)))
camera.listen(lambda image: image.save_to_disk('output/%06d.png' % image.frame))



lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('sensor_tick', '{x}'.format(x=1/__fps))
lidar_bp.set_attribute('dropoff_general_rate', '0.0')
lidar_bp.set_attribute('points_per_second', '{x}'.format(x=vf*1440000))
lidar_bp.set_attribute('range', '60')

lidar_bp.set_attribute('rotation_frequency', '{x}'.format(x=vf*10))
#lidar_bp.set_attribute('channels', '32')
#lidar_bp.set_attribute('rotation_frequency', '10.0')
relative_tra = carla.Transform(carla.Location(z=2.5)+carla.Location(x=0))
lidar = world.spawn_actor(lidar_bp, relative_tra, attach_to=tesla)
lidar.listen(lambda data: lidar_callback(data))



try:
    while True:
        #asdasd
        x = 1
        spectator_transform =  tesla.get_transform()
        spectator_transform.location += carla.Location(x = -6, y=0, z = 2.0)
        spectator.set_transform(spectator_transform)
        world.tick()

finally:
    print("Ending program and destructing things")
    print(tesla.destroy())
    print(camera.destroy())
    print(lidar.destroy())