module MaximumAreaCoverageOptimization

export greet_my_package
include("functions.jl")

global x_start = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]]
print(x_start[1])

EMPTY,TREE, FIRE = 0,1,2
prob_tree_ignition = 0.01

# Grid size and parameters
grid_size = (4,4)

# Initialize the grid
grid = rand(EMPTY:TREE, grid_size) #distribute trees and empty space randomly.
grid[1, :] .= FIRE
println(grid)

l = 0
indexArray = findall(x -> x == TREE, grid)
for i in indexArray
    global l = i
end

println(l)
print(l[1]-1)

println(atan(0/0))

end
