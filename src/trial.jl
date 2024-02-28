# p = [[1,2,3]; [4,5,6]; [7,8,9]]


# # for i in 1:3.0
# #     print(i)
# # end

# points = Vector{Vector{Float64}}()
    
# for i in 1:10
#     for j in 1:10
#         push!(points, [i-1/2, j-1/2, 1, 1.0, 0.0]) #[x coordinate, y coordinate, area represented, Importance level, Area Covered? (Boolean)]
#     end
# end

# x = [[1,2,3,4,5];[6,7,8,9,10]]
# y = [1,2,3,4,5]
# #push!(x,y)


# function rmv(arr)
#     for i in eachindex(arr)
#         global arr[i][3] = 11
#     end
# end

# print(y[2:4])

# using LinearAlgebra
# v = [1,2,3]
# u = [3,4,5]

# print(dot(v,u))
# print(norm(u))

# print(rad2degrees(acos(1.2)))


using Plots

p1 = plot()
function plot_shaded_circles(centers, radii; kwargs...)
    #scatter([center[1] for center in centers], [center[2] for center in centers], markersize=0, aspect_ratio=:equal, legend=false, xlabel="X", ylabel="Y")
    for i in 1:length(centers)
        θ = LinRange(0, 2π, 100)
        x = centers[i][1] .+ radii[i] .* cos.(θ)
        y = centers[i][2] .+ radii[i] .* sin.(θ)
        plot!(p1,x, y, fill=true, color=:blue, linecolor=:blue; kwargs...)
    end
end

# Example centers and radii
centers = [[0, 0], [2, 1], [-1, 2]]
radii = [1, 0.5, 1.5]

# Plotting
plot_shaded_circles(centers, radii)