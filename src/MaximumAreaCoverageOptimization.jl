module MaximumAreaCoverageOptimization
include("TDM_TRAJECTORY_opt.jl")
using StaticArrays, Rotations, LinearAlgebra
using RobotZoo: Quadrotor
using RobotDynamics
using Plots
export greet_my_package
include("functions.jl")

function equally_spaced_coordinates(pos_start, pos_final, N)
    x1 = pos_final[1]
    y1 = pos_final[2]
    z1 = pos_final[3]
    delta_x = pos_final[1] - pos_start[1]
    delta_y = pos_final[2] - pos_start[2]
    delta_z = pos_final[3] - pos_start[3]
    delta_x_spacing = delta_x / N
    delta_y_spacing = delta_y / N
    delta_z_spacing = delta_z / N
    coordinates = zeros(N,3)
    for i in 1:N
        coordinates[i,1:3] = [x1 + i * delta_x_spacing, y1 + i * delta_y_spacing, z1 + i * delta_z_spacing]
    end

    return coordinates
end

# Example usage: 
x1, y1, z1 = 1, 1, 1
x2, y2, z2 = 4, 4, 4
N = 3
initial = RBState([5.0, 0.0, 5.95876796], UnitQuaternion(I), zeros(3), zeros(3))
final = RBState([128.8088698, 3.39493165, 30.0], UnitQuaternion(I), zeros(3), zeros(3))
coordinates = equally_spaced_coordinates(initial[1:3], final[1:3], N)
println(coordinates[2,1:3])

#println(RBState(coordinates[1], UnitQuaternion(I), zeros(3), zeros(3)))

end
