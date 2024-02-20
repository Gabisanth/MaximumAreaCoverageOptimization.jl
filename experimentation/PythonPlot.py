import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d

VA1 = [1, 1, 1]
VA2 = [0.14666667296224753, 2.153588015193439, -1.860254669268945]

VB1 = [-1, -1, -1]
VB2 = [-0.14666667296224753, -2.153588015193439, 1.8602546692689454]

PA = [0, 0, 0]
PB = [5, 5, 5]


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
    ax.set_xlim([0, 5])
    ax.set_ylim([0, 5])
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

plt.legend(['Before', 'After'], loc='upper left')
# Show the plot
plt.show()