# Modified based on Logan's codes: https://github.com/Logan1904/FYP_Optimization
include("Base_Functions.jl")
using .Base_Functions

include("AreaCoverageCalculation.jl")


# Extreme Constraint 1: Radius of drone has to be between r_min and r_max (get based on FOV and h_min & h_max)
function cons1(x)
    for i in range(1,stop=Int(length(x)/3))
        R_val = x[N*2 + i]
        val = (R_val > r_min)# && R_val <= r_max[i])
        if !val
            return false
        end
    end
    return true

end


# Extreme Constraint 2: Could not be contained by or contain the same UAV
#function cons2(x)
    # this_group = AreaCoverageCalculation.make_circles(x)
    
    # for i in eachindex(this_group)
    #     if Base_Functions.pure_contained(this_group[i], pre_optimized_circles_MADS[i]) !== nothing
    #         return false
    #     end
    # end
    #return true
#end


# Extreme Constraint 3: Limits on the displacement between "pre-optimized" and "post-optimized" position for each UAV
function cons3(x)
    this_group = AreaCoverageCalculation.make_circles(x)
    for i in eachindex(this_group)
        x1 = pre_optimized_circles_MADS[i].x
        y1 = pre_optimized_circles_MADS[i].y
        z1 = pre_optimized_circles_MADS[i].R / tan(FOV/2)
        # z1 = pre.R / tan(FOV/2)
        
        x2 = this_group[i].x
        y2 = this_group[i].y
        z2 = this_group[i].R / tan(FOV/2)
        
        if sqrt((x1-x2)^2+(y1-y2)^2+(z1-z2)^2) > d_lim[i]
            return false
        end
    end
    return true
end

# Extreme Constraint 4: Limits the target velocity direction to reduce oscillatory target setting. Only activate after one iteration of the full simulation has been completed.
function cons4(x)
    
    for i in 1:N
        #Velocity vector of UAV i.
        UAV_vel = velocities[i][1:2] #only for x-y plane vector.

        #Velocity vector of target i.
        Target_vel = [x[i] - single_output[i], x[i+N] - single_output[i+N]]#, x[i+2*N] - single_output[i+2*N]]

        #Compute Angle between the two vectors.
        angle = abs(acosd(dot(UAV_vel, Target_vel)/ (norm(UAV_vel) * norm(Target_vel))))

        if angle > 90 #user-defined maximum angle of cone.
            return false
        end
    end

    return true

end

#Extreme Constraint 5: Limit velocity of target.
function cons5(x)
    
    for i in 1:N
        #Velocity vector of target i.
        Target_vel = [x[i] - single_output[i], x[i+N] - single_output[i+N], x[i+2*N] - single_output[i+2*N]]

        if norm(Target_vel) > 2 
            return false
        end
    end

    return true

end

# Extreme Constraint 6: Control the acceleration of the target.
function cons6(x)
    
    for i in 1:N
        #Velocity vector of UAV i.
        Target_vel_previous = [target_location_memory[2][i] - target_location_memory[1][i], target_location_memory[2][i+N] - target_location_memory[1][i+N], target_location_memory[2][i+2*N] - target_location_memory[1][i+2*N]]

        #Velocity vector of target i.
        Target_vel = [x[i] - single_output[i], x[i+N] - single_output[i+N], x[i+2*N] - single_output[i+2*N]]

        if abs(norm(Target_vel)-norm(Target_vel_previous)) > 1 #user-defined maximum angle of cone.
            return false
        end
    end

    return true

end


# Extreme Constraint 7: Control altitude for better resolution in certain regions.
function cons7(x)
    #If y coordinate is below 250, then the height should be dropped to 15m.
    for i in 1:N
        if x[i+N] < 200
            if x[i+2*N] > 19 * tan(FOV/2)
                return false
            end
        end
    end

    return true

end


#Progressive Constraint 1: Limit altitude for better resolution.
function cons1_progressive(x)

    violation = 0

    for i in range(1,stop=Int(length(x)/3))
        R_val = x[N*2 + i]
        violation += R_val - resolution_height[i]
    end

    return violation*1000

end