#Visualising UAV attitude. Code adapted from ChatGPT results.

using Plots

plotlyjs() #offers better interactivity than GR.

# Sample UAV states (replace this with your actual UAV states)
states = rand(13)  # Replace with your 13 states

# Extract position and orientation states
x, y, z = states[1:3]
roll, pitch, yaw = states[4:6]

# Set lengths for representing the UAV body
body_length = 1.0
body_width = 0.5
body_height = 0.0

# Compute body vertices based on orientation and position
body_vertices = [
    [x + body_length * cos(yaw), y + body_length * sin(yaw), z],
    [x + 0.5 * body_width * cos(yaw + pi / 2), y + 0.5 * body_width * sin(yaw + pi / 2), z],
    [x - 0.5 * body_width * cos(yaw + pi / 2), y - 0.5 * body_width * sin(yaw + pi / 2), z],
    [x - body_length * cos(yaw), y - body_length * sin(yaw), z],
    [x + body_length * cos(yaw), y + body_length * sin(yaw), z + body_height],
    [x + 0.5 * body_width * cos(yaw + pi / 2), y + 0.5 * body_width * sin(yaw + pi / 2), z + body_height],
    [x - 0.5 * body_width * cos(yaw + pi / 2), y - 0.5 * body_width * sin(yaw + pi / 2), z + body_height],
    [x - body_length * cos(yaw), y - body_length * sin(yaw), z + body_height]
]

# Create a 3D plot
plot3d(
    xlabel = "X-axis", ylabel = "Y-axis", zlabel = "Z-axis",
    xlims = (-10, 10), ylims = (-10, 10), zlims = (-10, 10), legend = false
)

# Plot the UAV body
plot!([v[1] for v in body_vertices], [v[2] for v in body_vertices], [v[3] for v in body_vertices], color = :blue)
