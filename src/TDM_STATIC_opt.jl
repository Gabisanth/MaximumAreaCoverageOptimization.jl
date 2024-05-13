# Modified based on Logan's codes: https://github.com/Logan1904/FYP_Optimization


module TDM_STATIC_opt

using DirectSearch

include("Base_Functions.jl")
using .Base_Functions

using Random
using LinearAlgebra

"""
    CustomPoll()

Return an empty CustomPoll object.

CustomPoll returns directions only in the xy plane. To be used when no height changes need to be explored. (i.e. when height constraint is already being satisfied.)
"""
mutable struct CustomPoll{T} <: DirectSearch.AbstractPoll
    b::Dict{T,Vector{T}}
    i::Dict{T,Int}
    maximal_basis::Bool

    CustomPoll(;kwargs...) = CustomPoll{Float64}(;kwargs...)

    function CustomPoll{T}(;basis = :maximal) where T
        g = new()
        g.b = Dict{T, Vector{T}}()
        g.i = Dict{T, Int}()

        if basis == :maximal
            g.maximal_basis = true
        elseif basis == :minimal
            g.maximal_basis = false
        else
            error( "Unknown option for basis type" )
        end

        return g
    end
end

"""
    GenerateDirections(p::AbstractProblem, DG::LTMADS{T})::Matrix{T}

Generates columns and forms a basis matrix for direction generation.
"""
function DirectSearch.GenerateDirections(p::DirectSearch.AbstractProblem, DG::CustomPoll{T})::Matrix{T} where T
    # B = LT_basis_generation(p.config.mesh, p.N, DG)
    # Dₖ = _form_basis_matrix(p.N, B, DG.maximal_basis)

    N=3
    directions = Matrix{Float64}(undef, 3*N, 20)
    for i in 1:20
        angle = 2π * i / 20  # Angle in radians
        x_value = cos(angle)      # x = cos(angle)
        y_value = sin(angle)      # y = sin(angle)
        
        dir = [x_value, x_value, x_value, y_value, y_value, y_value, 0.0, 0.0, 0.0]
        #dir[end-N+1:end] .= 0.0
        directions[:, i] = dir
    end

    return directions
end






"""
    optimize(input, obj, cons_ext, cons_prog, N_iter, R_lim, domain_x, domain_y)

Optimize the problem using the Mesh Adaptive Direct Search solver

# Arguments:

    - 'input': Initial point
    - 'obj': Objective function
    - 'cons_ext': Extreme constraints
    - 'cons_prog': Progressive constraints
    - 'N_iter': Number of iterations
    - 'N': Number of UAVs
"""
function optimize(input, obj, cons_ext, cons_prog, N_iter)

 
    # Define optimization problem
    #global p = DSProblem(length(input), poll = CustomPoll()) #if we want to use our own poll stage.
    global p = DSProblem(length(input))
    SetInitialPoint(p, input)
    SetObjective(p, obj)
    SetIterationLimit(p, N_iter)
    
    #DirectSearch.SetMinimumMeshSize(p, 5.0)
    SetMaxEvals(p)

    for i in 1:length(input)
        #if i < 9
        SetGranularity(p, i, 1.0)
        # else
        #     SetGranularity(p, i, 0.1)
        # end
    end

    


    # progressive_constraint = cons_prog[1]
    # barrier_threshold = progressive_constraint(input)

    # # #Add progressive constraint
    # progressive_collection_index = AddProgressiveCollection(p; h_max = barrier_threshold^2) #have to square this because that is how h is calculated by the MADS algorithm.



    # Add constraints to problem
    for i in cons_ext
        AddExtremeConstraint(p, i)
    end
    for i in cons_prog
        # progressive_constraint = i
        # barrier_threshold = progressive_constraint(input)
        # progressive_collection_index = AddProgressiveCollection(p; h_max = barrier_threshold^2)
        # AddProgressiveConstraint(p, i; index = progressive_collection_index)
    end


    Optimize!(p)

    # After optimization, return feasible or infeasible solution
    if p.x === nothing #tests object identity.
        global result = p.i
    else
        global result = p.x
    end


    # iter = 0

    # while true
    #     iter += 1
    #     Optimize!(p)

    #     # After optimization, return feasible or infeasible solution
    #     if p.x === nothing #tests object identity.
    #         global result = p.i
    #     else
    #         global result = p.x
    #     end

    #     if iter > 20
    #         println("Loop ended with the number of iterations ($iter), which is more than 10")
    #         # break
    #         return false
    #     end

    #     # check if any circles are contained
    #     # if length(new_inputs)==0
    #     #var,new_input = check(input, result, r_min, r_max, d_lim, circles_pool)
    #     # else
    #     #     println("The length of new_inputs is: $(length(new_inputs))")
    #     #     var,new_input = check(new_inputs[end], result, r_min, r_max, d_lim, circles_pool)
    #     # end
        

    #     if false
    #         # SetInitialPoint(p, new_input)
    #         # BumpIterationLimit(p, i=N_iter)
    #         # println("Non-optimal solution (a circle is contained). REINITIALIZING...")

    #         # push!(new_inputs, new_input)
    #         # println("New input added in !!!!!")

    #         # println("Epoch $(k) -> Non-optimal solution (a circle is contained). REIALLOCATED...")
    #         #TDM_Functions.show_epoch(make_circles(new_input), nothing)

    #         return new_input
    #     else
    #         println("Timesetp $(k) -> Optimal solutions to all UAVs found... Optimization ended at Iteration: ", iter)
    #         break
    #     end

    # end

    runtime = p.status.runtime_total #Export runtime_data for analysis.
    return result, runtime

end


# """
#     check(input,output,R_lim,domain_x,domain_y)

# Checks if any Circle objects are contained by other circles, and regenerates them (randomly)

# # Arguments:
#     - 'input': Pre-optimized circles
#     - 'output': The current solution of post-optimized circles 
#                 (Concatenated vector of (x,y,R) values, obtained from MADS solver output)
#     - 'r_min': Minimum radius for detection circle
#     - 'r_max': Maximum radius for detection circle
#     - 'd_lim': displacement limit
# """
# function check(input, output, r_min, r_max, d_lim, circles_pool)
#     input_circles = make_circles(input)
#     output_circles = make_circles(output)

#     N = length(output_circles) # input and output should have the same number of circles

#     is_contained = Vector{Bool}([false for i in 1:N])
#     # is_pure_contained = Vector{Bool}([false for i in 1:N])
#     no_movement = Vector{Bool}([false for i in 1:N])

#     # Should not contained by the other UAVs at current epoch
#     for i in 1:N
#         for j in i+1:N
#             if contained(output_circles[i], output_circles[j]) == output_circles[i]
#                 is_contained[i] = true
#             elseif contained(output_circles[i], output_circles[j]) == output_circles[j]
#                 is_contained[j] = true
#             end
#         end
#     end

#     # Should not contained by the detection cirlce of same UAV at previous epoch
#     for i in 1:N
#         if output_circles[i].R<=input_circles[i].R || 
#             (output_circles[i].x==input_circles[i].x && output_circles[i].y==input_circles[i].y)
#             no_movement[i] = true
#         end
#     end

#     # for i in 1:N
#     #     output[i] = input_circles[i].x + 30                           # x
#     #     output[N + i] = input_circles[i].y                   # y
#     #     output[2*N + i] = r_max  # R
#     # end




#     # if any(is_contained) || any(no_movement)
#     #     println("is_contained: ",is_contained)
#     #     println("no_movement: ",no_movement)
#     #     global FOV = 80 /180 *pi
#     #     for i in 1:N
#     #         if is_contained[i] == true || no_movement[i] == true 

#     #             global success = true
#     #             for angle in 1:1:360
#     #                 success = true
#     #                 movement = 0.9*d_lim

#     #                 this_x = output[i] + movement * √2/√3 * cosd(angle)
#     #                 this_y = output[N + i] + movement * √2/√3 * sind(angle)
#     #                 this_r = output[2*N + i] + movement * 1/√3 * tan(FOV/2)

#     #                 global this_circle = Base_Functions.make_circles([this_x, this_y, this_r])
    
#     #                 for m in eachindex(circles_pool)
#     #                     if Base_Functions.intersection(this_circle[1], circles_pool[m]) !== nothing ||
#     #                         Base_Functions.contained(this_circle[1], circles_pool[m]) !== nothing
#     #                         success =  false
#     #                         break
#     #                     end
#     #                 end

#     #                 if success == true
#     #                     output[i] = this_x
#     #                     output[N + i] = this_y
#     #                     output[2*N + i] = this_r
#     #                     println("Reallocate successful!")
#     #                     break
#     #                 end
#     #             end


#     #             # if success == false
#     #             #     println("Reallocate fail! Interpolate outwards!")
#     #             #     direction = [output[i], output[N+i], output[2*N+i]/tan(FOV/2)]/sqrt((output[i])^2 + (output[N+i])^2+ (output[2*N+i]/tan(FOV/2))^2)
#     #             #     increment = d_lim *direction

#     #             #     output[i] = output[i] + increment[1]                             # x
#     #             #     output[N + i] = output[N + i] + increment[2]                    # y
#     #             #     output[2*N + i] = min(output[2*N + i] + increment[3] *tan(FOV/2) *0.9 ,  r_max)   # R
#     #             # end
#     #         end
#     #     end

#     #     return true, output                          
#     #     # true: has contained circles; output the current circles group in MADS format 
#     # else
#     #     return false, output #[]
#     #     # true: has no contained circle; output blank array
#     # end


    
#     return true, output
# end

end # module end