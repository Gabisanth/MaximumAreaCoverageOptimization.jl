####Using Trixi
using OrdinaryDiffEq
using Trixi
using LinearAlgebra

advection_velocity = (5, 5)
equations = LinearScalarAdvectionEquation2D(advection_velocity)

initial_condition = initial_condition_gauss
solver = DGSEM(polydeg=3, surface_flux=flux_lax_friedrichs)

coordinates_min = (-5.0, -5.0)
coordinates_max = ( 5.0,  5.0)
mesh = TreeMesh(coordinates_min, coordinates_max,
                initial_refinement_level=4,
                n_cells_max=30_000)

semi = SemidiscretizationHyperbolic(mesh, equations, initial_condition, solver)



tspan = (0.0, 10)
ode = semidiscretize(semi, tspan);

amr_indicator = IndicatorMax(semi, variable=first)

amr_controller = ControllerThreeLevel(semi, amr_indicator,
                                      base_level=2,
                                      med_level=4, med_threshold=0.1,
                                      max_level=6, max_threshold=0.6)

amr_callback = AMRCallback(semi, amr_controller,
                           interval=5,
                           adapt_initial_condition=true,
                           adapt_initial_condition_only_refine=true)

stepsize_callback = StepsizeCallback(cfl=0.9)

callbacks = CallbackSet(amr_callback, stepsize_callback);


sol = solve(ode, CarpenterKennedy2N54(williamson_condition=false),
            dt=1.0, # solve needs some value here but it will be overwritten by the stepsize_callback
            save_everystep=false, callback=callbacks);






using Plots
pd = PlotData2D(sol)
plot(pd)
plot!(getmesh(pd))