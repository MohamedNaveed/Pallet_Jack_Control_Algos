#!/usr/bin/env python

import numpy as np
import rospy

pallet_length = 2.3#2.33 #meters

max_velocity = 0.325
max_acceleration = 0.33 #m/s-2

max_ang_velocity = 0.7 #rad/s

class pallet_jack_control:

    velocity = 0
    steer_angle = 0
    fork_state = 0

class pallet_jack_state:

    x = 0
    y = 0
    theta = 0
