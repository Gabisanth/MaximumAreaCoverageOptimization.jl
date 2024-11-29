# Modified based on Logan's codes: https://github.com/Logan1904/FYP_Optimization


module TDM_STATIC_opt

using DirectSearch

include("Base_Functions.jl")
using .Base_Functions
include("AreaCoverageCalculation.jl")

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
    AreaMaxObjective(x)

Define Objective Function for area coverage optimization step.

# Arguments:

    - 'x': Vector of UAV locations in the form [x;y;r]
"""
# Define Area maximization objective function.

function createObjective(cells, N, r_max)
    function AreaMaxObjective(x) #x is the vector of UAV locations in the form [x;y;r]
        #global points_of_interest = AreaCoverageCalculation.createPOI(1.0,1.0,N,N) #Create initial set of points of interest.
        objective_circles = AreaCoverageCalculation.make_circles(x) #output is of the form: Vector of Circle objects
        objective_circles = AreaCoverageCalculation.make_MADS(objective_circles) #output is of the form: Vector of circle coordinates and radii [x;y;r]
        area_covered = AreaCoverageCalculation.calculateArea(objective_circles,cells.points_of_interest) #ouputs the area covered by circles.
    
        violation = 0.0
        for i in 1:N
            #violation += max((x[i+2*N] - r_max[i]), 0.0) #violate only if it goes above the height limit.
            violation += abs(x[i+2*N] - r_max[i]) #violate if UAV is not held at the max height.
        end
    
    
        #global points_of_interest = AreaCoverageCalculation.rmvCoveredPOI(objective_circles, points_of_interest)
        return -area_covered + violation*1e5
    end
    return AreaMaxObjective
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
    #SetMaxEvals(p) #Uses parallelisation by using number of threads available.

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


end # module end