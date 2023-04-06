#!/usr/bin/env python3

from frame2d import Frame2D
import math

import numpy as np


#TODO find sensible wheel distance parameter (determine experimentally)
wheelDistance=50

#TODO find sensible noise amplitudes for motor model
cozmoOdomNoiseX = 0.01
cozmoOdomNoiseY = 0.01
cozmoOdomNoiseTheta = 0.01


# Forward kinematics: compute coordinate frame update as Frame2D from left/right track speed and time of movement
def track_speed_to_pose_change(left, right, time):
    # for forward facing x-axis, left facing y-axis
    l = left*time
    r = right*time
    a = (r-l)/wheelDistance
    if abs(a) > 0.0001:
        rad = (l+r)/(2*a)
        return Frame2D.fromXYA(rad*math.sin(a),-rad*(math.cos(a)-1),a)
    else:
        return Frame2D.fromXYA((l+r)/2,0,a)
    return Frame2D()

# Differential inverse kinematics: compute left/right track speed from desired angular and forward velocity
def velocity_to_track_speed(forward, angular):
    # TODO
    return [0,0]

# Trajectory planning: given target (ralative to robot frame), determine next forward/angular motion 
# Implement in a linear way
# If far away and facing wrong direction: rotate to face target
# If far away and facing target: move forward
# If on target: turn to desired orientation
def target_pose_to_velocity_linear(relativeTarget: Frame2D):
    # TODO
    velocity=0
    angular=0
    return [velocity, angular]

# Trajectory planning: given target (ralative to robot frame), determine next forward/angular motion 
# Implement by means of cubic spline interpolation 
def target_pose_to_velocity_spline(relativeTarget: Frame2D):
    # TODO
    velocity=0
    angular=0
    return [velocity, angular]

# Take a true cube position (relative to robot frame). 
# Compute /probability/ of cube being (i) visible AND being detected at a specific measure position (relative to robot frame)
def cube_sensor_model(trueCubePosition, visible, measuredPosition):
    return 1.0

