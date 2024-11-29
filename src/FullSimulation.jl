include("TDM_STATIC_opt.jl")
include("Base_Functions.jl")
include("TDM_Constraints.jl")
include("Plotter.jl")
include("AreaCoverageCalculation.jl")
include("ORCA.jl")
include("TDM_TRAJECTORY_opt.jl")
include("ObstacleModelling.jl")
include("CellFunctions.jl")

using StaticArrays, Rotations, LinearAlgebra
using RobotZoo: Quadrotor
using RobotDynamics
using Plots
using Plots.PlotMeasures
using Random
using XLSX

default(show = true)
plotlyjs() #offers better interactivity than GR.


function run_simulation(cells, starting_circles, cons_ext, cons_prog, N, r_max, obstacles_included)
    #Initialise variables.
    single_input_pb = []                        # document the circles at each epoch
    single_output_pb = []

    #global vels = [] # for measuring average speed of drone. ##Purely for testing purposes.
    velocities = [zeros(Float64, 3) for _ in 1:N]

    target_location_memory = [[0.0], [0.0]] #For recording target location from [t-2, t-1]. For calculating previous target velocity.

    runtime_data_MADS = []
    runtime_data_ALTRO = []

    collision_avoidance_manoeuvres = 0
    distance_between_obstacles = [] #To compare closest approach during collision avoidance maneuvres.

    pre_optimized_circles_MADS = starting_circles

    ##Main Simulation Loop over time.
    for t in 1:Nt_sim
        println("Starting iteration $t")
        #print(pre_optimized_circles_MADS)

        # if t != 1
        #     target_location_memory[1] = single_output
        # end

        if environment_type == "dynamic"
            cells = CellFunctions.update_POI(cells, t)
        end

        ##Perform Area Maximization Optimization.
        # 0. Remove covered area.
        STATIC_input_MADS = AreaCoverageCalculation.make_MADS(pre_optimized_circles_MADS) #output is of the form: [x;y;R]. Holds previous timestep location of drones for MADS.
        drone_locs = STATIC_input_MADS #[x;y;R] location of UAVs from previous timestep. (i.e. beginning location for this iteration.)

        #Remove the covered area from the list of points to explore.
        #global points_of_interest = AreaCoverageCalculation.rmvCoveredPOI(drone_locs, points_of_interest)
        cells = CellFunctions.rmvCoveredPOI(cells, drone_locs)


        #Enforce altitude constraint if UAV comes within the region of high interest.
        if t != 1
            for i in 1:N
                if abs(15 - drone_locs[i+2N] / tan(FOV/2)) < 1
                    check = drone_locs[i] .< x_UB.+ h_max* tan(FOV/2) .&& drone_locs[i] .> x_LB.-h_max*tan(FOV/2)  .&& drone_locs[i+N] .< y_UB.+h_max* tan(FOV/2) .&& drone_locs[i+N] .> y_LB.-h_max* tan(FOV/2)
                    if any(check)
                        r_max[i] = 15 * tan(FOV/2)
                    else
                        r_max[i] = h_max * tan(FOV/2)
                    end
                end
            end
        end

        cons3 = create_cons3(pre_optimized_circles_MADS, FOV, d_lim)
        
        # 1. Set up the input at each timestep. (using previous MADS output to warm-start)
        # 2. Main area coverage optimization function.
        if t < 3
            single_input = drone_locs
            area_objective_func = TDM_STATIC_opt.createObjective(cells,N, r_max)
            cons4 = create_cons4(velocities, single_input)
            STATIC_output, runtime = TDM_STATIC_opt.optimize(single_input, area_objective_func, [cons_ext,cons3], cons_prog, N_iter) #output is of the form: [x;y;R].
        else
            single_input = single_output_pb[end]

            #Pre-check if constraint is violated by initial point.
            if cons1(single_input) == false || cons3(single_input) == false
                single_input = drone_locs
            end

            area_objective_func = TDM_STATIC_opt.createObjective(cells,N, r_max)
            cons4 = create_cons4(velocities, single_input)
            STATIC_output, runtime = TDM_STATIC_opt.optimize(single_input, area_objective_func, [cons_ext, cons3], cons_prog, N_iter) #output is of the form: [x;y;R]. And we add the 4th constraint.
        end
        
        push!(runtime_data_MADS, runtime)

        if STATIC_output == false
            println("The problem cannot be well solved s.t. all constraints")
            break
        end

        single_output = STATIC_output # Document current optimized target circles. Stores in the form [x;y;R]

        # if t != 1
        #     target_location_memory[2] = single_output
        # end


        ##Feed this optimization output to the trajectory optimization.
        x_start = Base_Functions.MADS_to_ALTRO(STATIC_input_MADS, FOV)
        x_final = Base_Functions.MADS_to_ALTRO(STATIC_output, FOV)        


        ##Perform Trajectory Optimization.
        #Define vector of Trajectory Problem objects.
        MAVs = Vector{TDM_TRAJECTORY_opt.Trajectory_Problem}()

        # Define variables depending on whether collision avoidance is required or not.
        for i in 1:N
            if obstacles_included

                neighbours = []
                neighbours_radii = []
                responsibility_shares = []
                
                local_neighbours = []
                local_neighbours_radii = []
                local_responsibility_shares = []

                for j in 1:N
                    if j != i
                        if norm(x_start[i][1:3] - x_start[j][1:3]) < 10
                            push!(local_neighbours, x_start[j])
                            push!(local_neighbours_radii, radius)
                            push!(local_responsibility_shares, 0.5)

                            distance = norm(x_start[i][1:3] - x_start[j][1:3]) - 2*0.25 
                            push!(distance_between_obstacles, distance)
                        end
                    end
                end

                for o in 1:length(R_static_obs)
                    if norm(x_start[i][1:2] - pos_static_obs[o][1:2]) < 20
                        static_obs = RBState([pos_static_obs[o][1], pos_static_obs[o][2], x_start[i][3]], UnitQuaternion(I), [0.0, 0.0, 0.0], zeros(3))
                        push!(local_neighbours, static_obs)
                        push!(local_neighbours_radii, R_static_obs[o])
                        push!(local_responsibility_shares, 1) #Drone needs to take full responsibility in avoiding static obstacles.
                        
                        distance = norm(x_start[i][1:2] - pos_static_obs[o][1:2]) - 0.25 - R_static_obs[o]
                        push!(distance_between_obstacles, distance)
                    end
                end

                push!(neighbours, local_neighbours)
                push!(neighbours_radii, local_neighbours_radii)
                push!(responsibility_shares, local_responsibility_shares)
                
            
                
                if isempty(neighbours[i])
                    collision_avoidance_mode = true

                    V_pref =  normalize(x_final[i][1:3] .- x_start[i][1:3]) * 2
                

                    x_start[i] = RBState(x_start[i][1:3], UnitQuaternion(I), [velocities[i][1], velocities[i][2], velocities[i][3]], zeros(3)) 
                    x_final[i] =  RBState(x_final[i][1:3], UnitQuaternion(I), [V_pref[1], V_pref[2], V_pref[3]], zeros(3)) 

                else
                    collision_avoidance_manoeuvres += 1
                    collision_avoidance_mode = true

                    #Populate the V_pref.
                    V_pref =  normalize(x_final[i][1:3] .- x_start[i][1:3]) * 2

                    #Create neighbours info arrays
                    P_Bs = Vector{Matrix{Float64}}()
                    V_Bs = Vector{Matrix{Float64}}()
                    for j in 1:length(neighbours[i])
                        push!(P_Bs, reshape(neighbours[i][j][1:3],1,3))
                        push!(V_Bs, reshape(neighbours[i][j][8:10],1,3))
                    end

                    x_start[i] = RBState(x_start[i][1:3], UnitQuaternion(I), [velocities[i][1], velocities[i][2], velocities[i][3]*0.0001], zeros(3)) 

                    V_optimal, collision_status = ORCA.ORCA_3D(radius, neighbours_radii[i], reshape(x_start[i][1:3], 1,3), P_Bs, reshape(x_start[i][8:10],1,3), V_Bs, responsibility_shares[i], V_pref)

                    println("V_optimal: ", V_optimal)
                    x_final_pos = reshape(x_start[i][1:3],1,3) + V_optimal*0.5

                    x_final[i] =  RBState([x_final_pos[1],x_final_pos[2],x_final_pos[3]], UnitQuaternion(I), [V_optimal[1], V_optimal[2], V_optimal[3]], zeros(3)) 
                end
                
            
            # Trajectory Optimization without collision avoidance.
            else
                
                collision_avoidance_mode = false
    
                V_pref =  normalize(x_final[i][1:3] .- x_start[i][1:3]) * 2
                    
                x_start[i] = RBState(x_start[i][1:3], UnitQuaternion(I), [velocities[i][1], velocities[i][2], velocities[i][3]], zeros(3)) 
                x_final[i] =  RBState(x_final[i][1:3], UnitQuaternion(I), [V_pref[1], V_pref[2], V_pref[3]], zeros(3)) 
               
            end
    
            #Run Trajectory Optimization.
            push!(MAVs, TDM_TRAJECTORY_opt.Trajectory_Problem(mass,J,gravity,motor_dist,kf,km,x_start[i],x_final[i], r_max[i], d_lim[i], FOV))
    
            MAV = MAVs[i]
            output = TDM_TRAJECTORY_opt.optimize(MAV,hor,Nt_horizon,Nm,collision_avoidance_mode, true) #Will append the next state of each UAV into their StateHistory.
    
            #Running the optimization again to record time causes issues.
            elapsed_time_altro = @elapsed  TDM_TRAJECTORY_opt.optimize(MAV,hor,Nt_horizon,Nm,collision_avoidance_mode, false) 
            push!(runtime_data_ALTRO,elapsed_time_altro)
    
            x_start[i] = output
            velocities[i] = output[8:10]
            #push!(vels, norm(output[8:10]))
    
        end

        

        ##Prepare for next timestep. (Use the next state to document the initial value for next time step.)
        #Give new 'pre_optimized_circles' (i.e. vector of Circle objects.)
        x_positions = Vector{Float64}([])
        y_positions = Vector{Float64}([])
        #R_values = Vector{Float64}([])   
        R_values_MADS = Vector{Float64}([]) #modified R values for MADS algorithm.

        for i in 1:N #for each UAV.
            push!(x_positions, convert(Float64, MAVs[i].StateHistory[end][1]))
            push!(y_positions, convert(Float64, MAVs[i].StateHistory[end][2]))
            radius =  MAVs[i].StateHistory[end][3] * tan(FOV/2)
            #push!(R_values, convert(Float64, radius))
            push!(R_values_MADS, convert(Float64, radius))
            #    # push!(R_values, convert(Float64, radius))
            # end
        end
        #xyR = [x_positions;y_positions;R_values]
        xyR_MADS = [x_positions;y_positions;R_values_MADS]

        #Give (modified) input for next MADS iteration.
        pre_optimized_circles_MADS = AreaCoverageCalculation.make_circles(xyR_MADS) #for next timestep.


        #Extract Trajectories.
        X = []
        for i in 1:N  # UAV index i
            MAV = MAVs[i]
            x = zeros(Float64, (length(MAV.StateHistory),13))
            # col1-3 (position); col4-7 (quaternion); col8-10(linear velocity); col11-13(angular velocity)
            for j in 1:length(MAV.StateHistory) # trajectory optimization index j (row)
                x[j,:] = MAV.StateHistory[j]    # StateHistory content(column)
            end
            push!(X,x)
            # "x" is the trajectory for each UAV
            # "X" contains all the individual "x"
        end
        push!(Xs,X)

        #Document the list of drone positions and areas.
        push!(single_input_pb, single_input)
        #Document the list of targets given to the drone.
        push!(single_output_pb, single_output)

    end

    return single_input_pb, single_output_pb, runtime_data_ALTRO, runtime_data_MADS, distance_between_obstacles

end


function plot_simulation(single_input_pb, single_output_pb, R_static_obs, cells)
    palettes = ["blue", "green", "orange", "purple", "red", "gray", "pink", "olive"]

    ####PLOTTING FIGURES.
    #1. Plotting the target circles  at each timestep.
    p1 = plot()
    # #history_TDM = cir_domain.Domain_History
    # # store the trajectory of center
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
    



    for j in 1:N
        this_color = palettes[mod1(j,length(palettes))]
        plot!(p1, all_center_x[j], all_center_y[j], legend = false, color=this_color)


        for i in 1:5:length(single_input_pb)
            Base_Functions.plot_circle(all_center_x[j][i], all_center_y[j][i], all_center_R[j][i], p1, this_color)
        end
    end

    plot!(p1,
        legend=false,
        size=(600, 600),
        xlabel="x [m]", xguidefontsize=15, xtickfontsize= 10, 
        ylabel="y [m]", yguidefontsize=15, ytickfontsize= 10, 
        xlims = (0, 500),
        ylims = (0, 500),
        title = "Coverage Area Plot of UAV Targets"
    )


    savefig(p1, "Outputs\\AreaCoveragePlot.png") 


    #2. Plotting the targets at each timestep but separately for x,y,z positions.
    x_target = []
    y_target = []
    z_target = []

    timesteps = range(1, stop=Nt_sim)
    for i in 1:N
        p2 = plot(label=false) #x values.
        p3 = plot(label=false) #y values.
        p4 = plot(label=false) #z values.

        this_color = palettes[mod1(i,length(palettes))]
        xvalues = []
        yvalues = []
        zvalues = []

        droneX = []
        droneY = []
        droneZ = []

        for t in 1:Nt_sim
            this_X = Xs[t]
            push!(xvalues, all_center_x[i][t])
            push!(yvalues, all_center_y[i][t])
            push!(zvalues, all_center_z[i][t])
            
            push!(droneX, this_X[i][end,1])
            push!(droneY, this_X[i][end,2])
            push!(droneZ, this_X[i][end,3])

        end

        if i == 1
            x_target = xvalues
            y_target = yvalues
            z_target = zvalues
        end

        plot!(p2, timesteps, xvalues,  color = "red", label=false)
        plot!(p2, timesteps, droneX,  color = this_color, ylabel="X (m)")

        plot!(p3, timesteps, yvalues,  color = "red", label=false)
        plot!(p3, timesteps, droneY,  color = this_color, ylabel="Y (m)", label=false)

        plot!(p4, timesteps, zvalues,  color = "red", label=false)
        plot!(p4, timesteps, droneZ,  color = this_color, ylabel="Z (m)", xlabel="Timestep", label=false)

        p = plot(p2,p3,p4, layout=(3, 1), margin=2*Plots.mm,  label=["UAV$i Targets" "UAV$i Positions" ])

        #Remove duplicate legend labels.
        for h in 3:length(p.series_list)
            p.series_list[h][:label] = ""
        end

        plot(p, legend = :outertopright)

        savefig(p, "Outputs\\XYZ_States-UAV$i")

    end

    #5. Plotting trajectory.
    #Plot 3D trajectories
    p5 = plot()

    X_data = []
    Y_data = []
    Z_data = []
    a_data = []
    b_data = []
    c_data = []
    d_data = []

    for j in eachindex(Xs)                         # for each timestep
        local this_X = Xs[j]
        for i in 1:N                               # for each UAV.

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
        plot!(p5, X_data[i:N:end], Y_data[i:N:end], Z_data[i:N:end],
        linewidth = 5,
        color = this_color, label="UAV $i")
    end


    for o in 1:length(R_static_obs)

        r = R_static_obs[o]
        h = 40
        m, n =200, 200
        u = range(0, 2pi, length=m)
        v = range(0, h, length=n)
        us = ones(m)*u'
        vs = v*ones(n)'
        #Surface parameterization
        X = r*cos.(us) .+ pos_static_obs[o][1]
        Y = r*sin.(us) .+ pos_static_obs[o][2]
        Z = vs

        if o==length(R_static_obs)
            surface!(p5, X,Y,Z, label = "Static Obstacles", color=:white, cbar=false, aspect_ratio=1.0)
        else
            surface!(p5, X,Y,Z, color=:white, cbar=false, aspect_ratio=1.0)
        end
    end


    scal = 500
    plot!(p5, grid = true, gridwidth = 3, 
        legend=:outertopright,
        legendfontsize=10,
        xlims=(0,scal), ylims=(0,scal), zlims=(0,h_max+10),  
        xlabel="x (m)", #xguidefontsize=14, xticks = -scal+30:40:scal-30, xtickfontsize= 10,
        ylabel="y (m)", #yguidefontsize=14, yticks = -scal+30:40:scal-30,  ytickfontsize= 10,
        zlabel="z (m)", #zguidefontsize=14, zticks = 0:5:35, ztickfontsize= 10, zrotation = -90,
        size=(800, 800),
        camera=(45, 30),
    )

    savefig(p5, "Outputs\\FullSimulationTrajectories.html")


    #6. lot birds-eye view of the trajectories.
    p6 = plot()


    for i in 1:N
        local this_color = palettes[mod1(i,length(palettes))]
        plot!(p6, X_data[i:N:end], Y_data[i:N:end],
        linewidth = 5,
        color = this_color, label="UAV $i")
    end

    scal = 500

    ##Plot area of high interest.
    x_values = [x_LB[1], x_LB[1], x_UB[1], x_UB[1],x_LB[1]]  # Replace x1, x2, x3, x4 with your x coordinates
    y_values = [y_LB[1], y_UB[1], y_UB[1], y_LB[1], y_LB[1]]  # Replace y1, y2, y3, y4 with your y coordinates

    # Plot the rectangle for high interest region.
    plot!(p6,x_values, y_values, legend=false, linewidth=5, color="cyan", label=false)

    # x_values = [x_LB[2], x_LB[2], x_UB[2], x_UB[2],x_LB[2]]  # Replace x1, x2, x3, x4 with your x coordinates
    # y_values = [y_LB[2], y_UB[2], y_UB[2], y_LB[2], y_LB[2]]  # Replace y1, y2, y3, y4 with your y coordinates

    # # Plot the rectangle for high interest region.
    # plot!(p8,x_values, y_values, legend=false, linewidth=5, color="cyan", label=false)

    for i in 1:length(R_static_obs)
        if i== length(R_static_obs)
            Base_Functions.plot_circle_obstacles(pos_static_obs[i][1], pos_static_obs[i][2], R_static_obs[i], p6, :gray, true)
        else
            Base_Functions.plot_circle_obstacles(pos_static_obs[i][1], pos_static_obs[i][2], R_static_obs[i], p6, :gray, false)
        end
    end

    plot!(p6, grid = true, gridwidth = 3, 
        legend=:outertopright,
        legendfontsize=10,
        xlims=(0,scal), ylims=(0,scal),
        xlabel="x (m)", #xguidefontsize=14, xticks = -scal+30:40:scal-30, xtickfontsize= 10,
        ylabel="y (m)", #yguidefontsize=14, yticks = -scal+30:40:scal-30,  ytickfontsize= 10,
        size=(800, 600),
        camera=(45, 30),
        left_margin=7mm
    )

    savefig(p6, "Outputs\\FullSimulationTrajectoriesBirdseyeView.html")


    #7. Plot Fire and Area Covered.

    p7 = plot()

    # Sample data for the heatmap
    x_heatmap = LinRange(0, 500, 100)
    y_heatmap = LinRange(0, 500, 100)
    z_heatmap = zeros(length(y_heatmap), length(x_heatmap))#[0 for x in x_heatmap, y in y_heatmap]

    z_heatmap[1,5] = 1

    grid_size = [100,100]

    # Initialize the grid
    grid = ones(Int(grid_size[1]),Int(grid_size[2]))

    total_high_interest_fire_points = 0
    uncovered_high_interest_fire_points = 0
    for p in cells.fire_point_xy_ccordinates #The full set of points, even the covered ones.
        # println(p[1:2])
        grid[Int((p[1]+(5/2))/5), Int((p[2]+(5/2))/5)] = 2
        check = p[1] .< x_UB .&& p[1] .> x_LB  .&& p[2] .< y_UB .&& p[2] .> y_LB
        if any(check)
            total_high_interest_fire_points += 1
        end
    end

    for p in cells.points_of_interest #The uncovered points.
        # println(p[1:2])
        check = p[1] .< x_UB .&& p[1] .> x_LB  .&& p[2] .< y_UB .&& p[2] .> y_LB
        if any(check)
            uncovered_high_interest_fire_points += 1
        end
    end


    println("Percentage of Area Covered: ", 100 - (uncovered_high_interest_fire_points/total_high_interest_fire_points)*100)

    xrange = (0, 500)
    yrange = (0, 500)
    large_grid = kron(grid, ones(Int, 5, 5))
    c = cgrad([:green,:red], [0.5], categorical = true)

    # Create the heatmap
    heatmap!(p7, 1:grid_size[2], 1:grid_size[1], large_grid',color=c, legend=false, xlims=xrange, ylims=yrange, size=(1000, 1000), cbar=false)

    #Plot circles to show area covered.
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


    for j in 1:N
        this_color = palettes[mod1(j,length(palettes))]


        for i in 1:5:length(single_input_pb)
            Base_Functions.plot_circleblack(all_center_x[j][i], all_center_y[j][i], all_center_R[j][i], p7, this_color)
        end

    end

    ##Plot area of high interest.
    x_values = [x_LB[1], x_LB[1], x_UB[1], x_UB[1],x_LB[1]]  # Replace x1, x2, x3, x4 with your x coordinates
    y_values = [y_LB[1], y_UB[1], y_UB[1], y_LB[1], y_LB[1]]  # Replace y1, y2, y3, y4 with your y coordinates

    # Plot the rectangle for high interest region.
    plot!(p7,x_values, y_values, legend=false, linewidth=5, color="cyan", label=false)

    plot!(p7,
        legend=false,
        size=(800, 600),
        xlabel="x (m)", xguidefontsize=15, xtickfontsize= 10, 
        ylabel="y (m)", yguidefontsize=15, ytickfontsize= 10, 
        xlims = (0, 500),
        ylims = (0, 500)
    )


    plot!(p7, [-10], [-10], label="Area of Interest", color=:red, lw=2)
    #plot!(p7, [-10], [-10], label="Area of High Interest", color=:cyan, lw=2)
    plot!(p7, [-10], [-10], label="Covered Area", color=:black, lw=2)

    plot!(p7, legend=:outertopright, legendfontsize=10, left_margin=7mm)

    savefig(p7, "Outputs\\AreaCoverage.html")

    return X_data, Y_data, Z_data, a_data, b_data, c_data, d_data, x_target, y_target, z_target

end


function record_data(X_data, Y_data, Z_data, a_data, b_data, c_data, d_data, x_target, y_target, z_target, runtime_data_ALTRO, runtime_data_MADS, distance_between_obstacles, R_static_obs)
    #Write data to Excel sheet for attitude and position plotting.
    for i in 1:N
        data = [
        X_data[i:N:end],
        Y_data[i:N:end],
        Z_data[i:N:end],
        a_data[i:N:end],
        b_data[i:N:end],
        c_data[i:N:end],
        d_data[i:N:end]
        ]

        # Specify the file path
        local filename = "Quadrotor_States$i.xlsx"
        local labels = ["x", "y", "z", "a", "b", "c", "d"] #positions and attitudes(in quartenion representation)

        XLSX.openxlsx(filename, mode="w") do xf
            sheet = xf[1]
            XLSX.writetable!(sheet, data, labels, anchor_cell=XLSX.CellRef("A1"))
        end

    end


    #7. Write target data to Excel Sheet.
    data = [
        x_target,
        y_target,
        z_target
    ]

    # Specify the file path
    filename = "Quadrotor_Targets.xlsx"
    labels = ["x", "y", "z"] #positions

    XLSX.openxlsx(filename, mode="w") do xf
        sheet = xf[1]
        XLSX.writetable!(sheet, data, labels, anchor_cell=XLSX.CellRef("A1"))
    end



    #7. Write MADS runtime data to Excel file.
    filename = "MADS_Runtime.xlsx"
    labels = ["MADS Runtime"] #positions
    data = [runtime_data_MADS]

    XLSX.openxlsx(filename, mode="w") do xf
        sheet = xf[1]
        XLSX.writetable!(sheet, data , labels, anchor_cell=XLSX.CellRef("A1"))
    end

    #7. Write ALTRO runtime data to Excel file.
    filename = "ALTRO_Runtime.xlsx"
    labels = ["ALTRO Runtime"] #positions
    data = [runtime_data_ALTRO]

    XLSX.openxlsx(filename, mode="w") do xf
        sheet = xf[1]
        XLSX.writetable!(sheet, data , labels, anchor_cell=XLSX.CellRef("A1"))
    end


    #7. Write collision avoidance closest distance between obstacles.
    if length(distance_between_obstacles) != 0

        filename = "CollisionAvoidance_ScenarioData_stepsOf5.xlsx"
        labels = ["Distance"] #positions
        data = [distance_between_obstacles]


        println("Number of Static Obstacles: ", length(R_static_obs))
        number_of_collisions = count(x -> x < 0, distance_between_obstacles)
        println("Number of Collisions: ", number_of_collisions)

        # Filter to get only positive numbers
        positive_numbers = filter(x -> x > 0, distance_between_obstacles)

        # Find the smallest positive number
        minimum_separation = minimum(positive_numbers)
        println("Minimum Separation: ", minimum_separation)


        XLSX.openxlsx(filename, mode="rw") do xf
            sheet = xf[Int(number_of_obstacles/5)]
            row = obstacle_scenario
            sheet[row, 1] = number_of_collisions
            sheet[row, 2] = minimum_separation
        end
    end

end

# Simulation Parameters.
tf = 20          #How many seconds to run for.
Xs= []              #Contains the trajectories for each UAV at each timestep.
N = 5               #Number of UAVs.
dt_sim = 0.5          #Timestep of whole simulation.
Nt_sim = convert(Int64, tf/dt_sim)  #Number of timesteps in simulation.
R1 = 150.0           # (user-defined) Initial radius 
ΔR = 5.0             # Expanding rate: 5m/update
FOV = 100/180*π       # FOV in radians
h_min = 5            # (user-defined) Flying altitude lower bound (exclude initialization)
h_max = 30.0           # (user-defined) Flying altitude upper bound
r_min = h_min * tan(FOV/2) # (user-defined, replaced later)
r_max = h_max * tan(FOV/2) * ones(N)
d_lim = 10 * ones(N)           # (user-defined) limitations on displacement of group UAV induced from optimization. 
N_iter = 100         # (use-defined) set the limit of iterations for coverage maximization

# Drone Parameters
mass = 0.5                                       # mass of quadrotor
J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix                    
gravity = SVector(0,0,-9.81)                     # gravity vector
motor_dist = 0.1750                              # distance between motors
kf = 5.0                                         # motor force constant (motor force = kf*u)
km = 0.245                                      # motor torque constant (motor torque = km*u)

#Receding Horizon control parameters.
hor = 3.0          # Prediction horizon length
dt_horizon = dt_sim           # Time-step length per horizon
Nt_horizon = Int(hor/dt_horizon)+1  # Number of timesteps per horizon
R_D = 10.0          # Danger radius
R_C = 1.0           # Collision radius
Nm = 5              # Number of applied time-steps

#Define Known Interesting region boundaries.
x_LB = [2500]
x_UB = [3500] 
y_LB = [1000]
y_UB = [2000]

#Define Static or Dynamic Environment.
environment_type = "static"

#Define whether obstacles are present or not.
obstacles_included = false

#Define any known obstacle properties. (or create random allocation of obstacles).
if obstacles_included == true
    ###Create evenly spread out obstacles in a rectangular region.
    # Define the dimensions of the rectangular region
    x_min, x_max = 200.0, 250.0
    y_min, y_max = 150.0, 350.0

    # Initialise the obstacles.
    number_of_obstacles = 1
    num_points = number_of_obstacles
    pos_static_obs = generate_points(num_points, x_min, x_max, y_min, y_max)

    R_static_obs = fill(10, length(pos_static_obs))
    number_of_obstacles = length(pos_static_obs)
    radius = 0.25*1.2#for cooperative UAVs in swarm.
   
else
    R_static_obs = []
end


#Initialise search environment with either dynamic (e.g. forest fire) or static scenario.
cells = CellFunctions.Cells(Vector{Vector{Float64}}(),Vector{Vector{Float64}}()) #Creates the struct instance.
cells = CellFunctions.initialise_POI(cells, environment_type)


# Define extreme and progressive constraints
cons_ext = cons1
cons_prog = []

#Allocate the initial circles. (i.e. UAV starting positions).
#global STATIC_input_MADS = allocate_even_circles_in_a_line(0.0, 30.0, N, 10 * tan(FOV/2), 100 , 180)
STATIC_input_MADS = Base_Functions.allocate_even_circles(15.0, N, 10 * tan(FOV/2), 250.0, 250.0)#returns vector of [x;y;R] values.
pre_optimized_circles_MADS = AreaCoverageCalculation.make_circles(STATIC_input_MADS) #returns vector of Circle objects.

#Run the simulation.
single_input_pb, single_output_pb, runtime_data_ALTRO, runtime_data_MADS, distance_between_obstacles = run_simulation(cells, pre_optimized_circles_MADS, cons_ext, cons_prog, N, r_max, obstacles_included)

X_data, Y_data, Z_data, a_data, b_data, c_data, d_data, x_target, y_target, z_target = plot_simulation(single_input_pb, single_output_pb, R_static_obs, cells)


record_data(X_data, Y_data, Z_data, a_data, b_data, c_data, d_data, x_target, y_target, z_target, runtime_data_ALTRO, runtime_data_MADS, distance_between_obstacles, R_static_obs)



