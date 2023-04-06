from frame2d import Frame2D
from Cube47cm_20deg  import cubeFrames


import matplotlib.pyplot as plt
import scipy.stats as stats
import math
import numpy as np
import csv

x = []
y = []
angle = []


for cubeframes in cubeFrames:
    for t, frames in cubeFrames:
        x.append(frames.x())
        y.append(frames.y())
        angle.append(frames.angle())

def std_deviation(True_value, Measured_value):
    deviation = []
    
    for true_value in True_value:
        stddeviation = Measured_value -true_value
        stddeviation = stddeviation ** 2
        stddeviation = math.sqrt(stddeviation)
        deviation.append(stddeviation)
    average_deviation = np.mean(deviation)

    return average_deviation

measured_x = 470
measured_y = 0
measured_angle = 20 

x_Deviation = std_deviation(x,measured_x)
y_Deviation = std_deviation(y, measured_y)
angle_Deviation = std_deviation(angle, measured_angle)

print(x_Deviation, y_Deviation, angle_Deviation)


x_variance = x_Deviation ** 2
y_variance = y_Deviation ** 2
angle_variance = angle_Deviation ** 2

print(x_variance, y_variance, angle_variance )

