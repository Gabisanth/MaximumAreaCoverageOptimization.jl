# Modified based on Logan's codes: https://github.com/Logan1904/FYP_Optimization

include("TDM_TRAJECTORY_opt.jl")
using StaticArrays, Rotations, LinearAlgebra
using RobotZoo: Quadrotor
using RobotDynamics
using Plots

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

# Drone Parameters
mass = 0.5                                       # mass of quadrotor
J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix                    
gravity = SVector(0,0,-9.81)                     # gravity vector
motor_dist = 0.1750                              # distance between motors
kf = 1.0                                         # motor force constant (motor force = kf*u)
km = 0.0245                                      # motor torque constant (motor torque = km*u)

# MPC optimization
hor = 2.0          # Prediction horizon length
dt = 0.2         # Time-step length per horizon
Nt = Int(hor/dt)+1  # Number of timesteps per horizon
R_D = 10.0          # Danger radius
R_C = 1.0           # Collision radius
Nm = 5              # Number of applied time-steps




Xs= [] #Contains the trajectories for each UAV at each epoch.
N=1 #number of UAVs.

x_val = 20
y_val = 20
z_val = 10

#Give the initial and final(target) conditions.
global start = [RBState([5.0, 0.0, 5.95876796], UnitQuaternion(I), zeros(3), zeros(3))]#TDM_TRAJECTORY_opt.GreensPb_to_ALTRO(single_input_pb[q])
global targets = [RBState([19.0461,3.13291,16.673228637], UnitQuaternion(I), zeros(3), zeros(3)), 
RBState([25.9461, 1.30991, 22.36981], UnitQuaternion(I), zeros(3), zeros(3)),
RBState([32.878799999, 0.901127, 28.4035], UnitQuaternion(I), zeros(3), zeros(3)),
RBState([41.88972858, -0.03456748, 29.9993], UnitQuaternion(I), zeros(3), zeros(3)),
RBState([58.91081, -1.6249, 30.0], UnitQuaternion(I), zeros(3), zeros(3)),
RBState([76.3157, 0.339013,30.0], UnitQuaternion(I), zeros(3), zeros(3)),
RBState([94.18215, -0.0998562, 30.0], UnitQuaternion(I), zeros(3), zeros(3)),
RBState([112.0917355, -1.68504, 30.0], UnitQuaternion(I), zeros(3), zeros(3)),
RBState([128.8088698, 3.39493165, 30.0], UnitQuaternion(I), zeros(3), zeros(3))]




# global x_start = [RBState([0.0, 0.0, 10.0], UnitQuaternion(I), zeros(3), zeros(3))]#TDM_TRAJECTORY_opt.GreensPb_to_ALTRO(single_input_pb[q])
# global x_final = [RBState([20,20,10.0], UnitQuaternion(I), zeros(3), zeros(3)), 
# RBState([30,30.0,10.0], UnitQuaternion(I), zeros(3), zeros(3))]


#Setup vector of trajectory problem objects for each drone.
global MAVs = Vector{TDM_TRAJECTORY_opt.Trajectory_Problem}()
for i in 1:N
    push!(MAVs, TDM_TRAJECTORY_opt.Trajectory_Problem(mass,J,gravity,motor_dist,kf,km,start[i],targets[i]))
end

#Optimize the trajectories.
global total_converge = false # Check if all MAVs have converged
global collision = Vector{Any}(undef,N) # Vector describing collision constraints
global virtual_target_counter = 1
global num_of_virtual_targets = 30
global new_target = true
global real_target_counter = 1

for i in 1:N
    global collision[i] = [false,[]]
end

#Find optimial trajectories.
global countIter = 0
while total_converge == false
    global countIter += 1
    println("-----------------------------Unconverged iteration No. $countIter -----------------------------")

    if new_target == true
        if real_target_counter > length(targets)
            break
        end
        x_final = targets[real_target_counter]
        global virtual_targets = equally_spaced_coordinates(MAVs[1].StateHistory[end][1:3],x_final[1:3], num_of_virtual_targets)
        global new_target = false
        global virtual_target_counter = 1
        global real_target_counter += 1
    end
    
    # Optimize if MAV has not converged to final position
    for i in 1:N
        global MAVs[i].TargetState = RBState(virtual_targets[virtual_target_counter,1:3], UnitQuaternion(I), zeros(3), zeros(3))
        t = TDM_TRAJECTORY_opt.optimize(MAVs[i],hor,Nt,Nm,collision[i])
    end

    virtual_target_counter += 1
    if virtual_target_counter > num_of_virtual_targets
        global new_target = true
    end



    # Check for collision
    # for i in 1:N
    #     global collision[i] = [false,[]]
    # end
    # for i in 1:N
    #     for j in i+1:N
    #         MAV1 = MAVs[i]
    #         MAV2 = MAVs[j]
    #         distance = sqrt(sum((MAV1.StateHistory[end][1:3] - MAV2.StateHistory[end][1:3]).^2))
    #         if distance <= R_D
    #             a,b,c = TDM_TRAJECTORY_opt.collide(MAV1,MAV2,R_C,Nt) # Output: Boolean; uav1; uav2
    #             collision[i] = [a,c] # MAV1 -> i -> b; MAV2 -> j -> c
    #             collision[j] = [a,b]
    #         end
    #     end
    # end

    # if countIter > 100
    #     println("Over 100 MPC trails")
    #     break
    # end

end

# # Normalise state histories
# global longest = maximum([length(MAVs[i].StateHistory) for i in 1:N])
# for i in 1:N
#     MAV = MAVs[i]
#     size = length(MAV.StateHistory)
#     if size < longest
#         println("There is $i be increased in statehistory length")
#         for j in 1:longest-size
#             push!(MAV.StateHistory,MAV.StateHistory[end])
#         end
#     end
# end

# Extract trajectories
global X = []
for i in 1:N  # UAV index i
    local MAV = MAVs[i]
    x = zeros(Float64, (length(MAV.StateHistory),13)) # col1-3 (position); col4-7 (quaternion); col8-10(linear velocity); col11-13(angular velocity)
    for j in 1:length(MAV.StateHistory) # trajectory optimization index j (row)
        x[j,:] = MAV.StateHistory[j]    # StateHistory content(column)
    end
    push!(X,x)
    # "x" is the trajectory for each UAV
    # "X" contains all the individual "x"
end

push!(Xs, X)

#The state output is now the next start state for the next epoch. We will call this temporary state conatainer junction_state.
junction_state = Vector{RBState}()
for i in 1:N
    push!(junction_state, RBState(X[i][end,:]))
end

global x_start = junction_state




# start = MAVs[1].StateHistory[1]
final = MAVs[1].StateHistory[end]
# print(start[1:3]) #Display the initial xyz position.
println(final[1:3]) #Display the final xyz position.



## Plot 3D trajectories

p = plot()
palette = ["blue", "orange", "green", "purple", "cyan", "pink", "gray", "olive"]

M = 5                #total number of map updates. Defines how much the dynamic area expands by.
h_max = 30           # (user-defined) Flying altitude upper bound.


## Plot 1(a): cylinder 
using Plots
plotlyjs()

r = 150 + M*5 #150 is the initial radius.
h = h_max
m, n =200, 200
u = range(0, 2pi, length=m)
v = range(0, h, length=n)
us = ones(m)*u'
vs = v*ones(n)'
#Surface parameterization
X = r*cos.(us)
Y = r*sin.(us)
Z = vs
p = plot(Plots.surface(X, Y, Z, size=(600,600), 
    cbar=:none, 
    legend=false,
    colorscale="Reds",
    linewidth=0.5, 
    linealpha=0.4,
    alpha=0.4, 
    # linecolor=:red,
    camera = (45, 10), 
), label = "Domain")



# plot()
## Plot 1(b): 3D trajectories for the all the UAVs
for j in eachindex(Xs)                         # epoch index
    local this_X = Xs[j]
    for i in 1:N                                # UAV index
        local this_color = palette[mod1(i,length(palette))]
        if j!= M
            plot!(this_X[i][:,1],this_X[i][:,2],this_X[i][:,3],
                linewidth = 3,
                color = this_color, 
                label=:none, 
            )
        else
            plot!(this_X[i][:,1],this_X[i][:,2],this_X[i][:,3], 
                linewidth = 3,
                color = this_color,
                label="UAV $i",
            )
        end
    end
end



#Plotting the targets for visually evaluating performance of the controller.
x = []
y = []
z = []

for i in 1:length(targets)
    append!(x,targets[i][1])
    append!(y,targets[i][2])
    append!(z,targets[i][3])
end

plot!(x,y,z, seriestype=:scatter, color = "red", markersize = 2, label = "Targets")


scal = r
plot!(grid = true, gridwidth = 3, 
    legend=:outertopright,
    legendfontsize=10,
    xlims=(-scal,scal), ylims=(-scal,scal), zlims=(0,35),  
    xlabel="x [m]", xguidefontsize=14, xticks = -scal+30:40:scal-30, xtickfontsize= 10,
    ylabel="y [m]", yguidefontsize=14, yticks = -scal+30:40:scal-30,  ytickfontsize= 10,
    zlabel="z [m]", zguidefontsize=14, zticks = 0:5:35, ztickfontsize= 10, zrotation = -90,
    size=(800, 800),
    camera=(45, 30),
)