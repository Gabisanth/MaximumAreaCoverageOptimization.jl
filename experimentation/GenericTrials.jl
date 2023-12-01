#Visualising UAV attitude.

using Plots


# Example data of 13 states (replace this with your drone's states)
states = rand(13, 13)  # 13 states each containing 13 values

# Extracting position data (x, y, z)
positions = states[:, 1:3]  # Assuming the first 3 states represent x, y, z coordinates

# Extracting orientation data (roll, pitch, yaw)
orientations = states[:, 4:6]  # Assuming states 4:6 represent roll, pitch, yaw

# Extract x, y, z coordinates for positions
x_coords = positions[:, 1]
y_coords = positions[:, 2]
z_coords = positions[:, 3]

# Create a quiver plot to represent orientation
quiver_data = [(x_coords[i], y_coords[i], z_coords[i], cos(orientations[i, 3]), sin(orientations[i, 3]), 0) for i in 1:size(states, 1)]

# Plot the trajectory in 3D along with orientation arrows
plot3d(x_coords, y_coords, z_coords, line = (:blue, 3), marker = (:circle, 5))
quiver!([(x, y, z, u, v, w) for (x, y, z, u, v, w) in quiver_data],
        quiver = (1, "black", "arrow"))
xlabel!("X-axis")
ylabel!("Y-axis")
zlabel!("Z-axis")
title!("Drone Trajectory with Orientation")