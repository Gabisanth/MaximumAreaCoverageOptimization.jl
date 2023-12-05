#Study to check the discretisation of UAV dynamics.
import RobotDynamics as RD
using RobotZoo
using StaticArrays, Rotations, LinearAlgebra
using Plots

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
solutions = []
time_mesh_size = []


u = [1.0,1.0,1.0,1.0]
dt = range(0.01, stop=0.5, step=0.005)  # time step into future (s) <--Vary this to perform mesh refinement study.
t = 0   # current time (s)
global T = 0.0 # Total time elapsed (s)
T_sim = 10.0 #Simulation Time.

# Evaluate the discrete dynamics and Jacobian
dmodel = RD.DiscretizedDynamics{RD.RK4}(Model)

for i in eachindex(dt)
    global x = [0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    global T = 0.0
    while T <= T_sim
        global x = RD.discrete_dynamics(dmodel, x, u, t, dt[i]) #outputs
        global T += dt[i] 
    end
    push!(solutions, norm(x[1:4]))
    push!(time_mesh_size, dt[i])
end

print(solutions)

plot(time_mesh_size, solutions, label="Distance Travelled in 10s", xlabel="Timestep Size (s)", ylabel="Solution (m)")












#RD.discrete_jacobian!(RD.StaticReturn(), RD.ForwardAD(), dmodel, ∇f, x′, z)
