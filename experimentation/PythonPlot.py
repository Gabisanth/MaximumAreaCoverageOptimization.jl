import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d

VA1 = [1.38074959940266 ,-0.0003146999375496985, 0.48673504994122085]
VA2 = [1.3807496088930926 ,-0.000314709918189932 ,0.48673504994122085]

VB1 = [-1.1405185990757603 ,-0.34021109890855483, -0.47935235204214266]
VB2 = [-1.1405186054686887, -0.3402110891469277, -0.4793523625379082]

VC1 = [0.2507718486958495, -0.46124310126052553, 0.0001405999930730791]
VC2 = [0.250771842685054, -0.46124308559692306 ,0.00014057844514040894]

VD1 = [-0.35321584096871306, 0.7382185280079052, 0.2879334518767432]
VD2 = [-0.3532158380228703 ,0.7382185125481274 ,0.2879334810627664]

PA = [0, 0, 5]
PB = [10, 0, 5]
PC = [5, 5, 5]
PD = [5, -5, 5]


def plottingVectors(ax, x_start, y_start, z_start, x_end, y_end, z_end, colour):


    # Plot the vector
    ax.quiver(x_start, y_start, z_start,
            x_end - x_start, y_end - y_start, z_end - z_start,
            color=colour)

    # Add an arrowhead at the end of the vector
    arrow_end = np.array([[x_end, y_end, z_end]])
    proj_arrow_end = proj3d.proj_transform(arrow_end[:, 0], arrow_end[:, 1], arrow_end[:, 2], ax.get_proj())
    arrow_size = 20
    #ax.scatter(proj_arrow_end[0], proj_arrow_end[1], proj_arrow_end[2], color='blue', s=arrow_size)

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([0, 10])
    ax.set_ylim([-5, 5])
    ax.set_zlim([0, 5])
    plt.title('3D Implementation of ORCA')

    return 


# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plottingVectors(ax, PA[0], PA[1], PA[2], PA[0] + VA1[0], PA[1] + VA1[1], PA[2] + VA1[2], 'blue')
plottingVectors(ax, PA[0], PA[1], PA[2], PA[0] + VA2[0], PA[1] + VA2[1], PA[2] + VA2[2], 'red')

plottingVectors(ax, PB[0], PB[1], PB[2], PB[0] + VB1[0], PB[1] + VB1[1], PB[2] + VB1[2], 'blue')
plottingVectors(ax, PB[0], PB[1], PB[2], PB[0] + VB2[0], PB[1] + VB2[1], PB[2] + VB2[2], 'red')

plottingVectors(ax, PC[0], PC[1], PC[2], PC[0] + VC1[0], PC[1] + VC1[1], PC[2] + VC1[2], 'blue')
plottingVectors(ax, PC[0], PC[1], PC[2], PC[0] + VC2[0], PC[1] + VC2[1], PC[2] + VC2[2], 'red')

plottingVectors(ax, PD[0], PD[1], PD[2], PD[0] + VD1[0], PD[1] + VD1[1], PD[2] + VD1[2], 'blue')
plottingVectors(ax, PD[0], PD[1], PD[2], PD[0] + VD2[0], PD[1] + VD2[1], PD[2] + VD2[2], 'red')

plt.legend(['Before', 'After'], loc='upper left')
# Show the plot
plt.show()