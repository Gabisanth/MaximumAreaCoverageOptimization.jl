using Random
using Plots


export_data = []

# Constants
EMPTY,TREE, FIRE = 0,1,2
num_iterations = 100
grid_size = [100,100]
forest_density = 0.7
prob_spread = 0.5

# Initialize the grid
grid = ones(grid_size[1],grid_size[2]) #distribute trees and empty space randomly.
for i in 1:grid_size[1], j in 1:grid_size[2]
    if rand() < forest_density #Value can be chosen based on the density of the forest. If set to 0.7, it means 70% of forest is trees.
        grid[i,j] = TREE
    else
        grid[i,j] = EMPTY
    end
end

grid[45:55,45:55] .= FIRE  # Spawn fire in a location on the grid. [y,x] in terms of grid indexing.

initial_points = []
for y in 45:55
    for x in 45:55
        push!(initial_points, x - 1/2 , y - 1/2, 1.0, 1.0, false)
    end
end
push!(export_data, initial_points)


# Wind parameters
wind_speed = 1  # Random wind speed from 0 to 5
global wind_direction = 3*pi/2# Random wind direction in radians


# Function to update the grid using cellular automata rules
function update_grid(grid)
    points = []
    new_grid = copy(grid)
    for i in 2:size(grid, 1)-1, j in 2:size(grid, 2)-1
        if grid[i, j] == TREE
            if any(grid[i-1:i+1, j-1:j+1] .== FIRE)
                
                #Wind contribution.
                #Find which neighbours are on fire.
                indexArray = findall(x -> x == FIRE, grid[i-1:i+1, j-1:j+1])
                for index in indexArray
                    if wind_speed * cos(wind_direction - atan((2-index[2]),(2-index[1]))) * prob_spread > rand() #|| prob_spread > rand() #indices changed to transform the coordinate system.
                        new_grid[i, j] = FIRE #the i is the y coordinate, and j is the x coordinate. In terms of grid, i=row and j=column.
                        push!(points, i - 1/2 , j - 1/2, 1.0, 1.0, false)
                    end
                end
            end
        end
    end
    return new_grid, points
end

# Simulate fire propagation
heatmap(1:grid_size[2], 1:grid_size[1], grid', c=:viridis, clim=(0, 2), color=:grays)
for t in 1:num_iterations
    global grid, points = update_grid(grid)
    # if t == 50
    #     global wind_direction = -pi/4
    # end
    p = heatmap(1:grid_size[2], 1:grid_size[1], grid', c=:viridis, clim=(0, 2), color=:grays) #The transpose is done on grid, in order to change the coordinates system with x on x axis and y on y axis.
    display(p)
    #sleep(0.5)

    push!(export_data, points)
end
current()
savefig("ForestFire")



#print(length(export_data))
#Sample code for writing to Excel file. 
using XLSX
XLSX.openxlsx("FirePoints.xlsx", mode="w") do xf
    sheet = xf[1]

    for i in eachindex(export_data)
        sheet["A$i"] = export_data[i]
    end

end
