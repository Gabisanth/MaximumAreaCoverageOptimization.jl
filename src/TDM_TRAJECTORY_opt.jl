module TDM_TRAJECTORY_opt

export Trajectory_Problem
export converge
export optimize
export GreensPb_to_ALTRO
export GreensPb_to_MADS
export MADS_to_ALTRO
export collide
export find_d_max



using TrajectoryOptimization
using RobotDynamics
using RobotZoo: Quadrotor
using StaticArrays, Rotations, LinearAlgebra
using Altro
include("TDM_Functions.jl")


"""
    struct Trajectory_Problem

An object describing an MAV

Attributes:
    - 'Mass::Float64': Mass
    - 'J::Diagonal': Diagonal mass inertia matrix
    - 'Gravity::SVector': Gravity vector
    - 'Motor_Distance::Float64': Motor distance
    - 'kf::Float64': Motor force constant
    - 'km::Float64': Motor moment constant
    - 'Model::Quadrotor': Dynamics model
    - 'StateHistory::Vector{RBState}': Vector of the state history
    - 'PredictedStates::Vector{RBState}': Vector of the future predicted states
    - 'TargetState::RBState': Final intended state
"""
mutable struct Trajectory_Problem
    Mass::Float64           # CONST
    J::Diagonal             # CONST
    Gravity::SVector        # CONST
    Motor_Distance::Float64 # CONST
    kf::Float64             # CONST
    km::Float64             # CONST
    Model::Quadrotor

    StateHistory::Vector{RBState}
    PredictedStates::Vector{RBState}

    TargetState::RBState
    

    function Trajectory_Problem(Mass::Float64,J::Diagonal,Gravity::SVector,Motor_Distance::Float64,kf::Float64,km::Float64,InitialState::RBState,TargetState::RBState)
        Model = Quadrotor(mass=Mass, J=J, gravity=Gravity, motor_dist=Motor_Distance, kf=kf, km=km)

        new(Mass, J, Gravity, Motor_Distance, kf, km, Model, [InitialState], [], TargetState)
    end

    function Trajectory_Problem(Mass::Float64,J::Diagonal,Gravity::SVector,Motor_Distance::Float64,kf::Float64,km::Float64,InitialState::RBState)
        Model = Quadrotor(mass=Mass, J=J, gravity=Gravity, motor_dist=Motor_Distance, kf=kf, km=km)

        new( Mass, J, Gravity, Motor_Distance, kf, km, Model, [InitialState], [] )
    end

end



"""
    converge(MAV::Trajectory_Problem)

Returns the Euclidean distance of the MAV from its TargetState
"""
function converge(MAV::Trajectory_Problem)
    current_position = MAV.StateHistory[end][1:3]
    final_position = MAV.TargetState[1:3]
    
    distance = sqrt(sum((current_position-final_position).^2))

    return distance
end



"""
    find_d_max(MAV::Trajectory_Problem, tf::Float64, Nf::Int64, sin_phi, cos_phi, sin_theta, cos_theta)

Returns the X(predicted states), prob (problem object), obj (objective function object) of the d bar determination
"""
function find_d_max(MAV, tf::Float64, Nf::Int64, d::Float64, sin_phi, cos_phi, sin_theta, cos_theta)
    n,m = size(MAV.Model)       # n: number of states 13; m: number of controls 4
    num_states = n

    # xf = SVector(MAV.StateHistory[end]); # however it is the given x0, 20230810
    weight_Q = 1.0 #1e-10
    weigth_R = 1.0 #1e-10
    weigth_Qf = 1.0
    Q = Diagonal(@SVector fill(weight_Q, num_states))
    # Q = Diagonal(SA[weight_Q, weight_Q, weight_Q, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    R = Diagonal(@SVector fill(weigth_R, m))
    Qf = Diagonal(@SVector fill(weigth_Qf, num_states)) #xf: 0,0,0, Qf 1,1,1
    # Qf = Diagonal(SA[weigth_Qf, weigth_Qf, weigth_Qf, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) #xf: 0,0,0, Qf 1,1,1


    x0 = SVector(MAV.StateHistory[end])
    xf = [x0[1]+ d*sin_phi*cos_theta, x0[2]+ d*sin_phi*sin_theta, x0[3]+ d*cos_phi, 
          1.0, 0.0, 0.0, 0.0, 
          0.0, 0.0, 0.0, 
          0.0, 0.0, 0.0];

    obj = LQRObjective(Q, R, Qf, xf, Nf)  # only about the 3D position

    cons = ConstraintList(num_states, m, Nf)


    x_min = [-100.0,-100.0,9.0,  -1.0,-1.0,-1.0,-1.0,  -2.0,-2.0,-2.0,  -1.5,-1.5,-1.5]
    x_max = [100.0,100.0,11.0,  1.0,1.0,1.0,1.0,  2.0,2.0,2.0,  1.5,1.5,1.5]
    add_constraint!(cons, BoundConstraint(n, m, x_min=x_min, x_max=x_max), 1:Nf-1)


    prob = Problem(MAV.Model, obj, x0, tf, xf=xf, constraints=cons);

    hover = zeros(MAV.Model)[2]

    state_guess = zeros(Float64, (num_states,Nf))
    control_guess = zeros(Float64, (m, Nf-1))
    for i in 1: Nf-1  
        state_guess[:,i] = x0 + (xf-x0)*(i-1)/(Nf-1)
        control_guess[:,i] = hover                    # 13 * number of (timesteps-1)
    end
    state_guess[:,Nf] = xf 

    initial_states!(prob, state_guess)
    initial_controls!(prob, control_guess)


    # solver = ALTROSolver(prob);

    solver = ALTROSolver(prob, show_summary=false);

    solve!(solver);


    X = states(solver);

    return X, x0, xf, solver, prob, obj
end




"""
    optimize(MAV::Trajectory_Problem, tf::Float64, Nt::Int64, Nm::Int64, collision::Vector{Any})

Performs a trajectory optimization of the MAV from StateHistory[end] (current state) to TargetState

# Arguments:
    - 'tf::Float64': Time horizon
    - 'Nt::Int64': Number of timesteps
    - 'collision::Vector{Any}': Vector of form [Boolean,[x,y,z]], where the Boolean value describes if a collision is imminent with any other MAVs at the location (x,y,z)
"""
function optimize(MAV::Trajectory_Problem, tf::Float64, Nt::Int64, Nm::Int64, collision::Vector{Any})

    x0 = SVector(MAV.StateHistory[end])  # initial 3D positions of MAV
    xf = SVector(MAV.TargetState)         # final 3D positions of MAV


    n,m = size(MAV.Model)       # n: number of states 13; m: number of controls 4
    num_states = n
    num_controls = m#Add slack control variable for max_height soft constraint.
    # xf = SVector(MAV.StateHistory[end]); # however it is the given x0, 20230810
    weight_Q = 1 #1e-10 #Penalise the sum of state errors in the states.
    weigth_R = 1.0 #1e-10 #Penalise controller effort.
    weigth_Qf = 1.0 #Penalise current state error.
    Q = Diagonal(@SVector fill(weight_Q, num_states))
    # Q = Diagonal(SA[weight_Q, weight_Q, weight_Q, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    R = Diagonal(@SVector fill(weigth_R, num_controls))
    Qf = Diagonal(@SVector fill(weigth_Qf, num_states)) #xf: 0,0,0, Qf 1,1,1
    # Qf = Diagonal(SA[weigth_Qf, weigth_Qf, weigth_Qf, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) #xf: 0,0,0, Qf 1,1,1
    objective = LQRObjective(Q, R, Qf, xf, Nt)


    # Constraints
    cons = ConstraintList(num_states, num_controls, Nt)
    x_min = [-200.0,-200.0,0.0,  -1.0,-1.0,-1.0,-1.0,  -2.0,-2.0,-2.0,  -1.5,-1.5,-1.5]
    x_max = [200.0,200.0, 20.0,  1.0,1.0,1.0,1.0,  2.0,2.0,2.0,  1.5,1.5,1.5]

    u_min = [0.0, 0.0, 0.0, 0.0]
    u_max = [2.0,2.0,2.0,2.0]

    add_constraint!(cons, BoundConstraint(num_states,num_controls, x_min=x_min, x_max=x_max, u_min = u_min, u_max=u_max), 1:Nt)

    # Add collision constraints if present
    if collision[1] == true
        x,y,z = collision[2]
        add_constraint!(cons, SphereConstraint(n, [x], [y], [z], [1.5]), 1:Nt)
    end

    # #Add goal constraint.
    # goalcon = GoalConstraint(xf, 1:3)
    # add_constraint!(cons, goalcon, Nt)  # add to the last time step

    # With random initial positions (with x0=x0)
    prob = Problem(MAV.Model, objective, x0, tf, xf = xf, constraints=cons)


    # State initialization: linear trajectory from start to end
    state_guess = zeros(Float64, (num_states,Nt))
    # Control initialization: hover
    control_guess = zeros(Float64, (num_controls,Nt))
    hover = zeros(MAV.Model)[2]

    for i in 1:Nt
        state_guess[:,i] = x0 + (i-1)*(xf-x0)/(Nt-1)   # assume linear interpolation; this is the initial trajectory plan.
        control_guess[:,i] = hover                     # 13 * number of (timesteps-1)
    end

    #state_guess[:,Nt] = xf                             # 13 * number of timesteps

    initial_states!(prob, state_guess)
    #initial_controls!(prob, control_guess)
    
    altro = ALTROSolver(prob)

    altro = ALTROSolver(prob,show_summary=false);

    solve!(altro);

    X = states(altro);
    #U = controls(altro);

    MAV.PredictedStates = X

    # for i in 2:Nm
    #     push!(MAV.StateHistory, X[i])
    # end

    push!(MAV.StateHistory, X[2]) # We apply the next suggested control input, so we want the next state after this control input has been applied.
    return X #altro.stats.tsolve # Returns the total save time in milliseconds.
end






function GreensPb_to_ALTRO(GreensPb)
    #returns -> RBState([x_val, y_val, z_val], UnitQuaternion(I), zeros(3), zeros(3))
    return MADS_to_ALTRO(GreensPb_to_MADS(GreensPb)) #convert GreensPb to MADS, then convert the MADS to ALTRO
end

function GreensPb_to_MADS(GreensPb)
    cirs = GreensPb.circles
    return TDM_Functions.make_MADS(cirs)
end

"""
    MADS_to_ALTRO(z::Vector{Float64})

Transforms our MADS data structure to the ALTRO data structure
"""
function MADS_to_ALTRO(z::Vector{Float64})
    N = Int(length(z)/3)
    FOV = 80/180*pi

    x = Vector{RBState}()
    for i in range(1,stop=N)
        x_val = z[i]
        y_val = z[N + i]
        R_val = z[N*2 + i]

        z_val = R_val/tan(FOV/2)

        push!(x, RBState([x_val, y_val, z_val], UnitQuaternion(I), zeros(3), zeros(3))); 
        # positions(3), orientation(4), velocity(3), angular velocity(3)

        # push!(x, RBState([x_val, y_val, z_val], zeros(3), zeros(3), zeros(3))); 
        # ---- modified on 2023.08.09 -----
        # positions(3), 
        # Euler angles(3: phi (roll)|theta (pitch)|psi (yaw)), 
        # Linear velocity(3), 
        # Angular velocity(3) 
    end

    return x
end



"""
    collide(MAV1::Trajectory_Problem, MAV2::Trajectory_Problem, R_C::Float64, Nt::Int64)

Checks if MAV1 is going to collide (defined by R_C) based on their future predicted states
Returns a Boolean value describing if collision is imminent, and the (x,y,z) positions of the 2 MAVs -> to be inserted in the 'collision' vector
"""
function collide(MAV1::Trajectory_Problem, MAV2::Trajectory_Problem, R_C::Float64, Nt::Int64)
    for k in 1:Nt   
        predicted_distance = sqrt(sum((MAV1.PredictedStates[k][1:3] - MAV2.PredictedStates[k][1:3]).^2))
        if predicted_distance <= R_C
            return true, MAV1.PredictedStates[k][1:3], MAV2.PredictedStates[k][1:3]
        end
    end

    return false, [], []
end

end # module end