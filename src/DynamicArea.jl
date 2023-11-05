using Random
using Plots

include("Base_Functions.jl")

# Constants
EMPTY,TREE, FIRE = 0,1,2
num_iterations = 100
grid_size = [200,200]
forest_density = 0.7

# Initialize the grid
grid = ones(grid_size[1],grid_size[2]) #distribute trees and empty space randomly.
for i in 1:grid_size[1], j in 1:grid_size[2]
    if rand() < forest_density #Value can be chosen based on the density of the forest. If set to 0.7, it means 70% of forest is trees.
        grid[i,j] = TREE
    else
        grid[i,j] = EMPTY
    end
end

grid[45:55,45:55] .= FIRE  # Spawn fire in a location on the grid.

# Wind parameters
wind_speed = 1  # Random wind speed from 0 to 5
wind_direction =  pi/4# Random wind direction in radians


# Function to update the grid using cellular automata rules
function update_grid(grid)
    new_grid = copy(grid)
    for i in 2:size(grid, 1)-1, j in 2:size(grid, 2)-1
        if grid[i, j] == TREE
            if any(grid[i-1:i+1, j-1:j+1] .== FIRE)
                
                #Wind contribution.
                #Find which neighbours are on fire.
                indexArray = findall(x -> x == FIRE, grid[i-1:i+1, j-1:j+1])
                for index in indexArray
                    if wind_speed * cos(wind_direction - atan((2-index[1]),(2-index[2]))) > rand()
                        new_grid[i, j] = FIRE
                    end
                end
            end
        end
    end
    return new_grid
end

# Simulate fire propagation
heatmap(1:grid_size[2], 1:grid_size[1], grid, c=:viridis, clim=(0, 2), color=:grays)
for t in 1:num_iterations
    global grid = update_grid(grid)
    heatmap(1:grid_size[2], 1:grid_size[1], grid, c=:viridis, clim=(0, 2), color=:grays)
end
current()


