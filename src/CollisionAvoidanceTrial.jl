#TRIAL: ORCA with UAV dynamics.
include("ORCA.jl")
include("TDM_TRAJECTORY_opt.jl")

using StaticArrays, Rotations, LinearAlgebra
using RobotZoo: Quadrotor
using RobotDynamics
using Plots
default(show = true)
plotlyjs() #offers better interactivity than GR.


# Simulation Parameters.
tf = 15           #How many seconds to run for.
Xs= []              #Contains the trajectories for each UAV at each timestep.
N = 4                #Number of UAVs.
dt_sim = 0.5          #Timestep of whole simulation.
Nt_sim = convert(Int64, tf/dt_sim)  #Number of timesteps in simulation.
R1 = 150.0           # (user-defined) Initial radius 
ΔR = 5.0             # Expanding rate: 5m/update
FOV = 80/180*π       # FOV in radians
h_min = 5            # (user-defined) Flying altitude lower bound (exclude initialization)
h_max = 20.0           # (user-defined) Flying altitude upper bound
r_min = h_min * tan(FOV/2) # (user-defined, replaced later)
global r_max = h_max * tan(FOV/2) * ones(N)
global d_lim = 9.2 * ones(N)           # (user-defined) limitations on displacement of group UAV induced from optimization. 
N_iter = 100         # (use-defined) set the limit of iterations for coverage maximization

# Drone Parameters
mass = 0.5                                       # mass of quadrotor
J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix                    
gravity = SVector(0,0,-9.81)                     # gravity vector
motor_dist = 0.1750                              # distance between motors
kf = 1.0                                         # motor force constant (motor force = kf*u)
km = 0.0245                                      # motor torque constant (motor torque = km*u)

#Receding Horizon control parameters.
hor = 2.0          # Prediction horizon length
dt_horizon = dt_sim           # Time-step length per horizon
Nt_horizon = Int(hor/dt_horizon)+1  # Number of timesteps per horizon
R_D = 10.0          # Danger radius
R_C = 1.0           # Collision radius
Nm = 5              # Number of applied time-steps

radius = 0.7


P_A = RBState([0.0, 10, 5.0], UnitQuaternion(I), [0.0, 0.0, 0.0]+rand(-0.001:0.0000001:0.001, 3), zeros(3)) #velocity of exactly zero will cause issues in ORCA.
P_B = RBState([20.0, 10, 5.0], UnitQuaternion(I), [0.0, 0.0, 0.0]+rand(-0.001:0.0000001:0.001, 3), zeros(3))
P_C = RBState([10.0, 20.0, 5.0], UnitQuaternion(I), [0.0, 0.0, 0.0]+rand(-0.001:0.0000001:0.001, 3), zeros(3))
P_D = RBState([10.0, 0.0, 5.0], UnitQuaternion(I), [0.0, 0.0, 0.0]+rand(-0.001:0.0000001:0.001, 3), zeros(3))

G_A = P_B
G_B = P_A
G_C = P_D
G_D = P_C


global x_start = [P_A, P_B, P_C, P_D]
global x_final = [G_A, G_B, G_C, G_D]

global Xs = []

for t in 1:Nt_sim
    println("Timestep $t")
    #println(norm(x_start[1][1:3] - x_start[2][1:3]))

    neighbours = []

    for i in 1:N
        local_neighbours = []
        for j in 1:N
            if j != i
                if norm(x_start[i][1:3] - x_start[j][1:3]) < 12
                    push!(local_neighbours, x_start[j])
                end
            end
        end
        push!(neighbours, local_neighbours)
    end

    global MAVs = Vector{TDM_TRAJECTORY_opt.Trajectory_Problem}()

    for i in 1:N
        if isempty(neighbours[i])
            collision_avoidance_mode = false

            push!(MAVs, TDM_TRAJECTORY_opt.Trajectory_Problem(mass,J,gravity,motor_dist,kf,km,x_start[i],x_final[i], r_max[i], d_lim[i], FOV))

            local MAV = MAVs[i]
            output = TDM_TRAJECTORY_opt.optimize(MAV,hor,Nt_horizon,Nm,collision_avoidance_mode) #Will append the next state of each UAV into their StateHistory.
            global x_start[i] = output

        else
            collision_avoidance_mode = true
            #Populate the V_pref.
            V_pref =  normalize(x_final[i][1:3] .- x_start[i][1:3]) * norm(x_start[i][8:10])
            #V_pref = reshape(x_start[i][1:3], 1,3)

            #Create neighbours info arrays
            P_Bs = Vector{Matrix{Float64}}()
            V_Bs = Vector{Matrix{Float64}}()
            for j in 1:length(neighbours[i])
                push!(P_Bs, reshape(neighbours[i][j][1:3],1,3))
                push!(V_Bs, reshape(neighbours[i][j][8:10],1,3))
            end

        
            V_optimal, collision_status = ORCA.ORCA_3D(radius, fill(radius, length(neighbours[i])), reshape(x_start[i][1:3], 1,3), P_Bs, reshape(x_start[i][8:10],1,3), V_Bs, 0.5, V_pref)


            #Carry out Trajectory Optimization.
            x_final[i] =  RBState(x_final[i][1:3], UnitQuaternion(I), [V_optimal[1], V_optimal[2], V_optimal[3]], zeros(3)) 
            push!(MAVs, TDM_TRAJECTORY_opt.Trajectory_Problem(mass,J,gravity,motor_dist,kf,km,x_start[i],x_final[i], r_max[i], d_lim[i], FOV))
            
            local MAV = MAVs[i]
            output = TDM_TRAJECTORY_opt.optimize(MAV,hor,Nt_horizon,Nm,collision_avoidance_mode) #Will append the next state of each UAV into their StateHistory.
            global x_start[i] = output
        end
    end

    #Extract Trajectories.
    global X = []
    for i in 1:N  # UAV index i
        local MAV = MAVs[i]
        x = zeros(Float64, (length(MAV.StateHistory),13))
        # col1-3 (position); col4-7 (quaternion); col8-10(linear velocity); col11-13(angular velocity)
        for j in 1:length(MAV.StateHistory) # trajectory optimization index j (row)
            x[j,:] = MAV.StateHistory[j]    # StateHistory content(column)
        end
        push!(X,x)
        # "x" is the trajectory for each UAV
        # "X" contains all the individual "x"
    end
    #print(X)
    push!(Xs,X)




    # if norm(x_start[1][1:3] - x_start[2][1:3]) < 7

    #     println("x_start: ")
    #     println(x_start[1][8:10])
    #     println(x_start[2][8:10])

    #     #Collision Avoidance.
    #     V_pref = []
    #     V_opt_array = []

    #     #Populate the V_pref.
    #     for i in 1:N
    #         push!(V_pref, normalize(x_final[i][1:3] .- x_start[i][1:3]) * norm(x_start[i][8:10]))
    #         #push!(V_pref, reshape(x_start[i][1:3], 1,3))
    #     end

    #     #Populate the V_opt through ORCA.
    #     for i in 1:N

    #         #Create neighbourhood arrays.
    #         P_Bs = Vector{Matrix{Float64}}()
    #         V_Bs = Vector{Matrix{Float64}}()
    #         for j in 1:N
    #             if j != i
    #                 push!(P_Bs, reshape(x_start[j][1:3],1,3))
    #                 push!(V_Bs, reshape(x_start[j][8:10],1,3))
    #             end
    #         end
        
    #         V_optimal, collision_status = ORCA.ORCA_3D(R_A, [R_B], reshape(x_start[i][1:3], 1,3), P_Bs, reshape(x_start[i][8:10],1,3), V_Bs, 0.5, V_pref[i])
    #         push!(V_opt_array, V_optimal)

    #         #print(collision_status)

    #     end        

    #     collision_avoidance_mode = true
    #     #Trajectory Planning.
    #     global MAVs = Vector{TDM_TRAJECTORY_opt.Trajectory_Problem}()
    #     for i in 1:N
    #         x_final[i] =  RBState(x_final[i][1:3], UnitQuaternion(I), [V_opt_array[i][1], V_opt_array[i][2], V_opt_array[i][3]], zeros(3)) 
    #         push!(MAVs, TDM_TRAJECTORY_opt.Trajectory_Problem(mass,J,gravity,motor_dist,kf,km,x_start[i],x_final[i], r_max[i], d_lim[i], FOV))
    #     end

    #     for i in 1:N
    #         local MAV = MAVs[i]
    #         output = TDM_TRAJECTORY_opt.optimize(MAV,hor,Nt_horizon,Nm,collision_avoidance_mode) #Will append the next state of each UAV into their StateHistory.
    #         global x_start[i] = output
    #     end

    #     # println("V_pref1: ")
    #     # println(x_final[1][1:3])
    #     # println(x_start[1][1:3])
    #     # println(x_final[1][1:3] .- x_start[1][1:3])
    #     # println("V_opt_array: ")
    #     # println(V_opt_array)

        

    # else
    #     collision_avoidance_mode = false
    #     #Trajectory Planning.
    #     global MAVs = Vector{TDM_TRAJECTORY_opt.Trajectory_Problem}()
    #     for i in 1:N
    #         push!(MAVs, TDM_TRAJECTORY_opt.Trajectory_Problem(mass,J,gravity,motor_dist,kf,km,x_start[i],x_final[i], r_max[i], d_lim[i], FOV))
    #     end

    #     for i in 1:N
    #         local MAV = MAVs[i]
    #         output = TDM_TRAJECTORY_opt.optimize(MAV,hor,Nt_horizon,Nm,collision_avoidance_mode) #Will append the next state of each UAV into their StateHistory.
    #         global x_start[i] = output
    #     end
    # end


    # #Extract Trajectories.
    # global X = []
    # for i in 1:N  # UAV index i
    #     local MAV = MAVs[i]
    #     x = zeros(Float64, (length(MAV.StateHistory),13))
    #     # col1-3 (position); col4-7 (quaternion); col8-10(linear velocity); col11-13(angular velocity)
    #     for j in 1:length(MAV.StateHistory) # trajectory optimization index j (row)
    #         x[j,:] = MAV.StateHistory[j]    # StateHistory content(column)
    #     end
    #     push!(X,x)
    #     # "x" is the trajectory for each UAV
    #     # "X" contains all the individual "x"
    # end
    # #print(X)
    # push!(Xs,X)



end

#2. Plotting trajectory.
# Plot 3D trajectories
p2 = plot()

global palettes = ["blue", "green", "orange", "purple", "cyan", "pink", "gray", "olive"]

X_data = []
Y_data = []
Z_data = []
a_data = []
b_data = []
c_data = []
d_data = []

## Plot 1(a): cylinder 
#using Plots
r = 150 + Nt_sim*5
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
plot(p2, Plots.surface(X, Y, Z, size=(600,600), 
    cbar=:none, 
    legend=false,
    #colorscale="Reds",
    linewidth=0.5, 
    linealpha=0.4,
    alpha=0.4, 
    # linecolor=:red,
    camera = (45, 10), 
), label = "Domain")



# plot()
## Plot 1(b): 3D trajectories for the all the UAVs
for j in eachindex(Xs)                         # for each timestep
    local this_X = Xs[j]
    for i in 1:N             # for each UAV.

        push!(X_data, this_X[i][end,1])
        push!(Y_data, this_X[i][end,2])
        push!(Z_data, this_X[i][end,3])

        push!(a_data, this_X[i][end,4])
        push!(b_data, this_X[i][end,5])
        push!(c_data, this_X[i][end,6])
        push!(d_data, this_X[i][end,7])
    end
end


for i in 1:N
    local this_color = palettes[mod1(i,length(palettes))]
    plot!(p2, X_data[i:N:end], Y_data[i:N:end], Z_data[i:N:end],
    linewidth = 5,
    color = this_color, label="UAV $i")
    plot!(p2,[X_data[i:N:end][end]], [Y_data[i:N:end][end]], [Z_data[i:N:end][end]], marker = :circle, markersize = 5, color=this_color, legend = false, label=false)
end


scal = 25
plot!(p2, grid = true, gridwidth = 3, 
    legend=:outertopright,
    legendfontsize=10,
    xlims=(0,scal), ylims=(0,scal), zlims=(0,10),  
    xlabel="x [m]", xguidefontsize=14, xticks = -scal+30:40:scal-30, xtickfontsize= 10,
    ylabel="y [m]", yguidefontsize=14, yticks = -scal+30:40:scal-30,  ytickfontsize= 10,
    zlabel="z [m]", zguidefontsize=14, zticks = 0:5:35, ztickfontsize= 10, zrotation = -90,
    size=(800, 800),
    camera=(45, 30),
)






# V_A = [1.0 0.0 0.0] + reshape(rand(-0.001:0.0000001:0.001, 3), 1, 3)
# V_B = [-1.0 0.0 0.0] + reshape(rand(-0.001:0.0000001:0.001, 3), 1, 3)


# vAnew = (ORCA.ORCA_3D(R_A, [R_B], P_A[1:3], [P_B[1:3]], V_A, [V_B], 0.5, V_A))
# vBnew = (ORCA.ORCA_3D(R_B, [R_A], P_B[1:3], [P_A[1:3]], V_B, [V_A], 0.5, V_B))
