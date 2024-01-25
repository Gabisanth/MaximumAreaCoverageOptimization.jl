#Based on Code from https://github.com/AtsushiSakai/PythonRobotics/blob/master/AerialNavigation/drone_3d_trajectory_following/drone_3d_trajectory_following.py

from math import cos, sin
import time
from Quadrotor import Quadrotor
import pandas as pd
import math
import matplotlib.pyplot as plt
import numpy as np

def quaternion_to_euler(quaternion):
    # Extract components
    w, x, y, z = quaternion

    # Calculate roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x**2 + y**2)
    roll_x = math.atan2(t0, t1)

    # Calculate pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    # Calculate yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y**2 + z**2)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

show_animation = True

x_pos = -5
y_pos = -5
z_pos = 5
roll = 0
pitch = 0
yaw = 0

# Specify the file path
file_path = 'src/Quadrotor_States.xlsx'

# Read the Excel file into a DataFrame
df = pd.read_excel(file_path)

pitch_array = []

for row in range(0,df.shape[0]):
    x_pos = df.iat[row,0]
    y_pos = df.iat[row,1]
    z_pos = df.iat[row,2]

    w = df.iat[row,3]
    x = df.iat[row,4]
    y = df.iat[row,5]
    z = df.iat[row,6]
    quaternion = [w,x,y,z]

    roll, pitch, yaw = quaternion_to_euler(quaternion)
    print(pitch*180/3.14)
    pitch_array = np.append(pitch_array, pitch*180/3.14)

    if row == 0:
        q = Quadrotor(x=x_pos, y=y_pos, z=z_pos, roll=roll,
                pitch=pitch, yaw=yaw, size=1, show_animation=show_animation)
    else:
        q.update_pose(x_pos, y_pos, z_pos, roll, pitch, yaw)
    
    time.sleep(0.05)


x_travelled = -df.iat[0,0] + df.iat[-1,0]
avg_speed = x_travelled / 20

print("Avg Speed: ")
print(avg_speed)

t = np.arange(df.shape[0])

plt.plot(t, pitch_array)
plt.show()


