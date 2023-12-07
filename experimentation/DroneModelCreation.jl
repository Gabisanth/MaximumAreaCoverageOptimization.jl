#Attempt at creating my own quadcopter model, using code from RobotZoo package for Quadrotor model.
#This is to enable the addition of a soft constraint through adding a slack variable to the control inputs.
using StaticArrays
using RobotDynamics
using Rotations
using LinearAlgebra
using ForwardDiff, FiniteDiff

using RobotDynamics: ContinuousDynamics, RigidBody, LieGroupModel, @autodiff
import RobotDynamics: dynamics!, dynamics, jacobian!, forces, moments, wrenches, inertia, inertia_inv, orientation
import RobotDynamics: state_dim, control_dim


@autodiff struct Quadrotor{R} <: RigidBody{R}
    mass::Float64
    J::SMatrix{3,3,Float64,9}
    Jinv::SMatrix{3,3,Float64,9}
    gravity::SVector{3,Float64}
    motor_dist::Float64
    kf::Float64
    km::Float64
    bodyframe::Bool  # velocity in body frame?
    ned::Bool
end
#control_dim(::Quadrotor) = 4

state_dim(::Quadrotor) = 13
control_dim(::Quadrotor) = 5

function Quadrotor{R}(;
        mass=0.5,
        J=Diagonal(@SVector [0.0023, 0.0023, 0.004]),
        gravity=SVector(0,0,-9.81),
        motor_dist=0.1750,
        kf=1.0,
        km=0.0245,
        bodyframe=false,
        ned=false,
    ) where R
    @assert issymmetric(J)
    Quadrotor{R}(mass,J,inv(J),gravity,motor_dist,kf,km,bodyframe,ned)
end

(::Type{Quadrotor})(;kwargs...) = Quadrotor{UnitQuaternion{Float64}}(;kwargs...)

@inline RobotDynamics.velocity_frame(model::Quadrotor) = model.bodyframe ? :body : :world

function trim_controls(model::Quadrotor)
    @SVector fill(-model.gravity[3]*model.mass/4.0, size(model)[2])
end

function forces(model::Quadrotor, x, u)
    q = orientation(model, x)
    kf = model.kf
    g = model.gravity
    m = model.mass

    w1 = u[1]
    w2 = u[2]
    w3 = u[3]
    w4 = u[4]

    F1 = max(0,kf*w1);
    F2 = max(0,kf*w2);
    F3 = max(0,kf*w3);
    F4 = max(0,kf*w4);
    F = @SVector [0., 0., F1+F2+F3+F4] #total rotor force in body frame
    if model.ned
        F = SA[0,0,-F[3]]
        g = -g
    end

    f = m*g + q*F # forces in world frame
    return f
end

function moments(model::Quadrotor, x, u)

    kf, km = model.kf, model.km
    L = model.motor_dist

    w1 = u[1]
    w2 = u[2]
    w3 = u[3]
    w4 = u[4]

    F1 = max(0,kf*w1);
    F2 = max(0,kf*w2);
    F3 = max(0,kf*w3);
    F4 = max(0,kf*w4);

    M1 = km*w1;
    M2 = km*w2;
    M3 = km*w3;
    M4 = km*w4;
    tau = @SVector [L*(F2-F4), L*(F3-F1), (M1-M2+M3-M4)] #total rotor torque in body frame
    if model.ned
        tau = SA[tau[1], -tau[2], -tau[3]]
    end
    return tau
end

function wrenches(model::Quadrotor, x, u)
    F = forces(model, x, u)
    M = moments(model, x, u)
    return [F; M]

    q = orientation(model, x)
    C = forceMatrix(model)
    mass, g = model.mass, model.gravity

    # Calculate force and moments
    w = max.(u, 0)  # keep forces positive
    fM = forceMatrix(model)*w
    f = fM[1]
    M = @SVector [fM[2], fM[3], fM[4]]
    e3 = @SVector [0,0,1]
    F = mass*g - q*(f*e3)
    return F,M
end

function forceMatrix(model::Quadrotor)
    kf, km = model.kf, model.km
    L = model.motor_dist
    @SMatrix [
        kf   kf   kf   kf;
        0    L*kf 0   -L*kf;
       -L*kf 0    L*kf 0;
        km  -km   km  -km;
    ]
end


RobotDynamics.inertia(model::Quadrotor) = model.J
RobotDynamics.inertia_inv(model::Quadrotor) = model.Jinv
RobotDynamics.mass(model::Quadrotor) = model.mass

function Base.zeros(model::Quadrotor{R}) where R
    x = RobotDynamics.build_state(model, zero(RBState))
    u = @SVector fill(-model.mass*model.gravity[end]/4, 5)
    return x,u
end





model = Quadrotor()
n,m = RobotDynamics.dims(model)


















# # Example from RobotDynamics github.
# # Define the model struct with parameters
# struct Cartpole{T} <: AbstractModel
#     mc::T
#     mp::T
#     l::T
#     g::T
# end

# Cartpole() = Cartpole(1.0, 0.2, 0.5, 9.81)

# # Define the continuous dynamics
# function RobotDynamics.dynamics(model::Cartpole, x, u)
#     mc = model.mc  # mass of the cart in kg (10)
#     mp = model.mp   # mass of the pole (point mass at the end) in kg
#     l = model.l   # length of the pole in m
#     g = model.g  # gravity m/s^2

#     q = x[ @SVector [1,2] ]
#     qd = x[ @SVector [3,4] ]

#     s = sin(q[2])
#     c = cos(q[2])

#     H = @SMatrix [mc+mp mp*l*c; mp*l*c mp*l^2]
#     C = @SMatrix [0 -mp*qd[2]*l*s; 0 0]
#     G = @SVector [0, mp*g*l*s]
#     B = @SVector [1, 0]

#     qdd = -H\(C*qd + G - B*u[1])
#     return [qd; qdd]
# end

# # Specify the state and control dimensions
# RobotDynamics.state_dim(::Cartpole) = 4
# RobotDynamics.control_dim(::Cartpole) = 1

# # Create the model
# model = Cartpole()
# n,m = RD.dims(model)
