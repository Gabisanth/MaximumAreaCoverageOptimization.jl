include("TDM_TRAJECTORY_opt.jl")
#include("TDM_Functions.jl")
include("TDM_STATIC_opt.jl")
include("Base_Functions.jl")
#include("Greens_Method.jl")
include("TDM_Constraints.jl")
#include("TDM_Functions.jl")
include("Plotter.jl")
include("AreaCoverageCalculation.jl")

using StaticArrays, Rotations, LinearAlgebra
using RobotZoo: Quadrotor
using RobotDynamics
using Plots
using Random

default(show = true)
plotlyjs() #offers better interactivity than GR.

# Simulation Parameters.
tf = 100.0            #How many seconds to run for.
Xs= []              #Contains the trajectories for each UAV at each timestep.
N=2                 #Number of UAVs.
dt_sim = 0.5          #Timestep of whole simulation.
Nt_sim = convert(Int64, tf/dt_sim)  #Number of timesteps in simulation.
R1 = 150.0           # (user-defined) Initial radius 
ΔR = 5.0             # Expanding rate: 5m/update
FOV = 80/180*π       # FOV in radians
h_min = 1            # (user-defined) Flying altitude lower bound (exclude initialization)
h_max = 20.0           # (user-defined) Flying altitude upper bound
r_min = h_min * tan(FOV/2) # (user-defined, replaced later)
r_max = h_max * tan(FOV/2) 
d_lim = 9.2           # (user-defined) limitations on displacement of group UAV induced from optimization. 
N_iter = 100         # (use-defined) set the limit of iterations for coverage maximization

# Drone Parameters
mass = 0.5                                       # mass of quadrotor
J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix                    
gravity = SVector(0,0,-9.81)                     # gravity vector
motor_dist = 0.1750                              # distance between motors
kf = 1.0                                         # motor force constant (motor force = kf*u)
km = 0.0245                                      # motor torque constant (motor torque = km*u)

#Receding Horizon control parameters.
hor = 3.0          # Prediction horizon length
dt_horizon = dt_sim           # Time-step length per horizon
Nt_horizon = Int(hor/dt_horizon)+1  # Number of timesteps per horizon
R_D = 10.0          # Danger radius
R_C = 1.0           # Collision radius
Nm = 5              # Number of applied time-steps

##Simulation Initialisation.
#Initialise the time-varying area of interest.
#cir_domain = TDM_Functions.Domains_Problem([], R1, ΔR)

#Initialise the points of interest.
global points_of_interest = AreaCoverageCalculation.createPOI(1.0,1.0,100.0,100.0) #Create initial set of points of interest.

# Define Area maximization objective function.
function AreaMaxObjective(x) #x is the vector of UAV locations in the form [x;y;r]
    objective_circles = AreaCoverageCalculation.make_circles(x) #output is of the form: Vector of Circle objects
    objective_circles = AreaCoverageCalculation.make_MADS(objective_circles) #output is of the form: Vector of circle coordinates and radii [x;y;r]
    area_covered = AreaCoverageCalculation.calculateArea(objective_circles,points_of_interest) #ouputs the area covered by circles.
    return -area_covered
end


# x = [[-5.0, 0.0, 5.0, 0.0];[0.0, 5.0, 0.0, 5.0];[3.0, 3.0, 3.0, 3.0]]
# @timev AreaMaxObjective(x)


# Define extreme and progressive constraints
cons_ext = [cons1, cons2, cons3]
cons_prog = []

#Allocate the initial circles. (i.e. UAV starting positions).
global STATIC_input_MADS = Base_Functions.allocate_even_circles(5.0, N, 10 * tan(FOV/2), 50.0, 50.0) #returns vector of [x;y;R] values.
ini_circles = AreaCoverageCalculation.make_circles(STATIC_input_MADS) #returns vector of Circle objects.

#Initialise area maximisation placeholders.
global circles_pool = ini_circles          # Initialize circle pool
#global pre_optimized_circles = ini_circles # Initialize pre-optimized circles
global pre_optimized_circles_MADS = ini_circles # Initialize pre-optimized circles
single_input_pb = []                        # document the circles at each epoch
single_output_pb = []


##Main Simulation Loop over time.
for t in 1:Nt_sim
    println("Starting iteration $t")

    ##Perform Area Maximization Optimization.
    # 0. Remove covered area.
    global STATIC_input_MADS = AreaCoverageCalculation.make_MADS(pre_optimized_circles_MADS) #output is of the form: [x;y;R]. Holds previous timestep location of drones for MADS.
    drone_locs = STATIC_input_MADS #[x;y;R] location of UAVs from previous timestep. (i.e. beginning location for this iteration.)

    #Remove the covered area from the list of points to explore.
    global points_of_interest = AreaCoverageCalculation.rmvCoveredPOI(drone_locs, points_of_interest)


    # 1. Set up the input at each timestep. (using previous MADS output to warm-start)
    # 2. Main area coverage optimization function.
    if t == 1
        single_input = drone_locs
        global STATIC_output = TDM_STATIC_opt.optimize(single_input, AreaMaxObjective, cons_ext, cons_prog, N_iter) #output is of the form: [x;y;R].
    else
        single_input = single_output
        global STATIC_output = TDM_STATIC_opt.optimize(single_input, AreaMaxObjective, [cons_ext, cons4], cons_prog, N_iter) #output is of the form: [x;y;R]. And we add the 4th constraint.
    end
    
    if STATIC_output == false
        println("The problem cannot be well solved s.t. all constraints")
        break
    end

    global single_output = STATIC_output # Document current optimized target circles. Stores in the form [x;y;R]

    ##Feed this optimization output to the trajectory optimization.
    global x_start = Base_Functions.MADS_to_ALTRO(STATIC_input_MADS)
    global x_final = Base_Functions.MADS_to_ALTRO(STATIC_output)


    ##Perform Trajectory Optimization.
    #Define vector of Trajectory Problem objects. (Does this need to be done inside this time loop??)
    global MAVs = Vector{TDM_TRAJECTORY_opt.Trajectory_Problem}()
    for i in 1:N
        push!(MAVs, TDM_TRAJECTORY_opt.Trajectory_Problem(mass,J,gravity,motor_dist,kf,km,x_start[i],x_final[i]))
    end

    #Collision Avoidance Placeholders.
    global collision = Vector{Any}(undef,N) # Vector describing collision constraints
    for i in 1:N
        collision[i] = [false,[]]
    end

    #Run Main Trajectory Optimization step.
    global velocities = [] #To hold the velocities for next timestamp.

    for i in 1:N
        local MAV = MAVs[i]
        output = TDM_TRAJECTORY_opt.optimize(MAV,hor,Nt_horizon,Nm,collision[i]) #Will append the next state of each UAV into their StateHistory.

        push!(velocities, output[8:10])
        #println("X speed: ")
        #print(t[8])
        #println(output_log)
    end

    #Check for collision. (Can add this after).

    ##Prepare for next timestep. (Use the next state to document the initial value for next time step.)
    #Give new 'pre_optimized_circles' (i.e. vector of Circle objects.)
    x_positions = Vector{Float64}([])
    y_positions = Vector{Float64}([])
    R_values = Vector{Float64}([])   
    R_values_MADS = Vector{Float64}([]) #modified R values for MADS algorithm.

    for i in 1:N #for each UAV.
        push!(x_positions, convert(Float64, MAVs[i].StateHistory[end][1]))
        push!(y_positions, convert(Float64, MAVs[i].StateHistory[end][2]))
        local radius =  MAVs[i].StateHistory[end][3] * tan(FOV/2)
        push!(R_values, convert(Float64, radius))

        #R value for MADS will be modified if the height goes beyond the r_max value.
        if radius > r_max 
            push!(R_values_MADS, r_max)
            #global above_max_counter += 1
            #push!(R_values, r_max)
        else
            push!(R_values_MADS, convert(Float64, radius))
           # push!(R_values, convert(Float64, radius))
        end
    end
    xyR = [x_positions;y_positions;R_values]
    xyR_MADS = [x_positions;y_positions;R_values_MADS]
    #global pre_optimized_circles = Base_Functions.make_circles(xyR) #for next timestep.

    #Update circles pool, with the new position of drone, to say that this area has now been covered.
    #global circles_pool = union(circles_pool, pre_optimized_circles)

    #Give (modified) input for next MADS iteration.
    global pre_optimized_circles_MADS = AreaCoverageCalculation.make_circles(xyR_MADS) #for next timestep.

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

    #Document the list of drone positions and areas.
    push!(single_input_pb, single_input)
    #Document the list of targets given to the drone.
    push!(single_output_pb, single_output)

end




#println(length(points_of_interest))



#1. Plotting the circles and positions of the drones at each timestep.
p1 = plot()
#history_TDM = cir_domain.Domain_History
# store the trajectory of center
all_center_x=[]
all_center_y=[]
all_center_z=[]
all_center_R=[]

for j in 1:N                                    #N is the number of UAVs.
    xs_circle = []
    ys_circle = []
    Rs_circle = []
    zs_circle = []
    for i in eachindex(single_input_pb)
        circle_in = single_input_pb[i]
        push!(xs_circle, circle_in[j])
        push!(ys_circle, circle_in[j + N])
        push!(Rs_circle, circle_in[j + 2*N])
        z = circle_in[j + 2*N]/tan(FOV/2)
        push!(zs_circle, z)
    end
    push!(all_center_x, xs_circle)
    push!(all_center_y, ys_circle)
    push!(all_center_z, zs_circle)
    push!(all_center_R, Rs_circle)
end



global palettes = ["blue", "orange", "green", "purple", "cyan", "pink", "gray", "olive"]
for j in 1:N
    # palette = ["blue", "orange", "green", "purple", "cyan"]
    this_color = palettes[mod1(j,length(palettes))]

    for i in eachindex(single_input_pb)
        Base_Functions.plot_circle(all_center_x[j][i], all_center_y[j][i], all_center_R[j][i], p1)
    end


    # plot!(p1, all_center_x[i][1:end], all_center_y[i][1:end], 
    #     aspect_ratio=1, color = this_color,
    #     markershape =:cross, 
    #     markersize =:1,
    #     markerstrokestyle = :solid,
    #     label =:none, 
    #     legend =:none)
end

# for rnd in eachindex(history_TDM)
#     this_domain = history_TDM[rnd]
#     TDM_Functions.show_coverage(circles_pool, this_domain, rnd, Nt_sim, N)
# end

plot!(p1,
    legend=false,
    size=(600, 650),
    xlabel="x [m]", xguidefontsize=15, xtickfontsize= 10, 
    ylabel="y [m]", yguidefontsize=15, ytickfontsize= 10, 
    xlims = (0, 100),
    ylims = (0, 100),
    title = "Coverage Area Plot of UAVs"
)


savefig("output1.png") 




#4. Plotting the difference between the target set and current position of UAV at each timestep.
all_center_x=[]
all_center_y=[]
all_center_z=[]
all_center_R=[]

for j in 1:N                                    #N is the number of UAVs.
    xs_circle = []
    ys_circle = []
    Rs_circle = []
    zs_circle = []
    for i in eachindex(single_output_pb)
        circle_in = single_output_pb[i]
        push!(xs_circle, circle_in[j])
        push!(ys_circle, circle_in[j + N])
        push!(Rs_circle, circle_in[j + 2*N])
        z = circle_in[j + 2*N]/tan(FOV/2)
        push!(zs_circle, z)
    end
    push!(all_center_x, xs_circle)
    push!(all_center_y, ys_circle)
    push!(all_center_z, zs_circle)
    push!(all_center_R, Rs_circle)
end

# p4 = plot()
# p5 = plot()

# distance = zeros(N,Nt_sim)
# for t in 1:Nt_sim
#     local this_X = Xs[t]
#     for i=1:N
#         distance[i,t] = sqrt((all_center_x[i][t] - this_X[i][end,1])^2 + (all_center_y[i][t] - this_X[i][end,2])^2 + (all_center_z[i][t] - this_X[i][end,3])^2)
#     end
# end


# timesteps = range(1, stop=Nt_sim)

# plot!(p4, timesteps, distance[1,:], label = "UAV 1", color = "blue")

# plot!(p5, timesteps, distance[2,:], label = "UAV 2", color = "orange")

# plot(p4,p5, layout=(2, 1), title = "Overall difference between target position and actual position.")
# savefig("output4.png") 









#5. Plotting the targets at each timestep but separately for x,y,z positions.
timesteps = range(1, stop=Nt_sim)
for i in 1:N
    local p4 = plot() #x values.
    local p5 = plot() #y values.
    local p6 = plot() #z values.

    this_color = palettes[mod1(i,length(palettes))]
    local xvalues = []
    local yvalues = []
    local zvalues = []

    local droneX = []
    local droneY = []
    local droneZ = []

    for t in 1:Nt_sim
        local this_X = Xs[t]
        push!(xvalues, all_center_x[i][t])
        push!(yvalues, all_center_y[i][t])
        push!(zvalues, all_center_z[i][t])
        
        push!(droneX, this_X[i][end,1])
        push!(droneY, this_X[i][end,2])
        push!(droneZ, this_X[i][end,3])

    end

    plot!(p4, timesteps, xvalues,  color = "black")
    plot!(p4, timesteps, droneX,  color = this_color, title = "X")

    plot!(p5, timesteps, yvalues,  color = "black")
    plot!(p5, timesteps, droneY,  color = this_color, title = "Y")

    plot!(p6, timesteps, zvalues,  color = "black")
    plot!(p6, timesteps, droneZ,  color = this_color, title = "Z")

    plot(p4,p5,p6, layout=(3, 1))

    savefig("outputs5-$i")

end




#2. Plotting trajectory.
# Plot 3D trajectories
p2 = plot()
global palettes = ["blue", "orange", "green", "purple", "cyan", "pink", "gray", "olive"]

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

        #Record all position and attitude states for UAV plotting.
        if i==1 #choose which UAV to record all states for.
            push!(X_data, this_X[i][end,1])
            push!(Y_data, this_X[i][end,2])
            push!(Z_data, this_X[i][end,3])

            push!(a_data, this_X[i][end,4])
            push!(b_data, this_X[i][end,5])
            push!(c_data, this_X[i][end,6])
            push!(d_data, this_X[i][end,7])
        end



        #Plot only the position.
        local this_color = palettes[mod1(i,length(palettes))]
        if j!= Nt_sim
            plot!(p2, this_X[i][:,1],this_X[i][:,2],this_X[i][:,3],
                linewidth = 5,
                color = this_color, 
                label=:none, 
            )
        else
            plot!(p2, this_X[i][:,1],this_X[i][:,2],this_X[i][:,3], 
                linewidth = 5,
                color = this_color,
                label="UAV $i",
            )
            println(this_X[i][:,1])
        end
    end
end
scal = 100
plot!(p2, grid = true, gridwidth = 3, 
    legend=:outertopright,
    legendfontsize=10,
    xlims=(-scal,scal), ylims=(-scal,scal), zlims=(0,h_max+10),  
    xlabel="x [m]", xguidefontsize=14, xticks = -scal+30:40:scal-30, xtickfontsize= 10,
    ylabel="y [m]", yguidefontsize=14, yticks = -scal+30:40:scal-30,  ytickfontsize= 10,
    zlabel="z [m]", zguidefontsize=14, zticks = 0:5:35, ztickfontsize= 10, zrotation = -90,
    size=(800, 800),
    camera=(45, 30),
)





# #3. Plotting the target positions for the drones at each timestep.

# #p3 = plot()
# # history_TDM = cir_domain.Domain_History
# # # store the trajectory of center
# # all_center_x=[]
# # all_center_y=[]
# # all_center_z=[]
# # for j in 1:N   
# #     xs_circle = []
# #     ys_circle = []
# #     zs_circle = []
# #     for i in eachindex(single_output_pb)  
# #         circle_out = single_output_pb[i].circles[j]
# #         push!(xs_circle, circle_out.x)
# #         push!(ys_circle, circle_out.y)
# #         z = circle_out.R/tan(FOV/2)
# #         push!(zs_circle, z)
# #     end
# #     push!(all_center_x, xs_circle)
# #     push!(all_center_y, ys_circle)
# #     push!(all_center_z, zs_circle)

# #     # println("From single input history: For Drone $j")
# #     # println(xs_circle[1])
# #     # println(xs_circle[end])
# #     # println(ys_circle[1])
# #     # println(ys_circle[end])
# #     # println(Rs_circle[1])
# #     # println(Rs_circle[end])
# # end




# # global palettes = ["blue", "orange", "green", "purple", "cyan", "pink", "gray", "olive"]
# for i in 1:N
#     # palette = ["blue", "orange", "green", "purple", "cyan"]
    
#     this_color = palettes[i+2]
#     plot!(p2, all_center_x[i][1:end], all_center_y[i][1:end], all_center_z[i][1:end], 
#         aspect_ratio=1, color = this_color,
#         markershape = :cross, 
#         markersize =:1,
#         markerstrokestyle = :solid,
#         label =:none, 
#         legend =:none)

#         plot!(p2, [all_center_x[i][1]], [all_center_y[i][1]], [all_center_z[i][1]], 
#         aspect_ratio=1, color = this_color,
#         markershape = :square, 
#         markersize =:2,
#         markerstrokestyle = :solid,
#         label =:none, 
#         legend =:none)

#         plot!(p2, [all_center_x[i][end]], [all_center_y[i][end]], [all_center_z[i][end]], 
#         aspect_ratio=1, color = this_color,
#         markershape = :circle, 
#         markersize =:2,
#         markerstrokestyle = :solid,
#         label =:none, 
#         legend =:none, title = "Trajectory and Targets")


# end




#6. Write data to Excel sheet for attitude and position plotting.
using XLSX
data = [
    X_data,
    Y_data,
    Z_data,
    a_data,
    b_data,
    c_data,
    d_data
]

# Specify the file path
filename = "Quadrotor_States.xlsx"
labels = ["x", "y", "z", "a", "b", "c", "d"] #positions and attitudes(in quartenion representation)

XLSX.openxlsx(filename, mode="w") do xf
    sheet = xf[1]
    XLSX.writetable!(sheet, data, labels, anchor_cell=XLSX.CellRef("A1"))
end








