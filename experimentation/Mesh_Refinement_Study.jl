import RobotDynamics as RD
using RobotZoo
using StaticArrays, Rotations, LinearAlgebra

# Drone Parameters
Mass = 0.5                                       # mass of quadrotor
J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix                    
Gravity = SVector(0,0,-9.81)                     # gravity vector
Motor_Distance = 0.1750                              # distance between motors
kf = 1.0                                         # motor force constant (motor force = kf*u)
km = 0.0245                                      # motor torque constant (motor torque = km*u)

Model = RobotZoo.Quadrotor(mass=Mass, J=J, gravity=Gravity, motor_dist=Motor_Distance, kf=kf, km=km)
n,m = RD.dims(Model)

# Generate random state and control vector
x,u = rand(Model)

dt = 0.1  # time step into future (s)
t = 0.0   # current time (s)

# # Evaluate the continuous dynamics and Jacobian
# ẋ = RD.dynamics(Model, x, u)
# print(ẋ)
# # ∇f = zeros(n, n + m)
# #RD.jacobian!(RD.StaticReturn(), RD.ForwardAD(), Model, ∇f, ẋ, Model, z)

# Evaluate the discrete dynamics and Jacobian
dmodel = RD.DiscretizedDynamics{RD.RK4}(Model)
x′ = RD.discrete_dynamics(dmodel, x, u, t, dt) #outputs 








#RD.discrete_jacobian!(RD.StaticReturn(), RD.ForwardAD(), dmodel, ∇f, x′, z)
