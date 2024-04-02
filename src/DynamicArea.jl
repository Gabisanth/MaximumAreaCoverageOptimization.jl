using Random
using Plots


export_data = []
dx = 5
dy = 5

X = 500
Y = 500
x_start1 = 200
x_start2 = 300
y_start1 = 245
y_start2 = 255

# Constants
EMPTY,TREE, FIRE = 0,1,2
num_iterations = 100
grid_size = [round(X/dx),round(Y/dy)]
forest_density = 0.7
prob_spread = 0.5



# Initialize the grid
grid = ones(Int(grid_size[1]),Int(grid_size[2])) #distribute trees and empty space randomly.
for i in 1:Int(grid_size[1]), j in 1:Int(grid_size[2])
    if rand() < forest_density #Value can be chosen based on the density of the forest. If set to 0.7, it means 70% of forest is trees.
        grid[i,j] = TREE
    else
        grid[i,j] = EMPTY
    end
end

grid[Int(round(x_start1/dx)):Int(round(x_start2/dx)),Int(round(y_start1/dy)):Int(round(y_start2/dy))] .= FIRE  # Spawn fire in a location on the grid. [x,y] in terms of grid indexing.

initial_points = []
for y in round(y_start1/dy):round(y_start2/dy)
    for x in round(x_start1/dx):round(x_start2/dx)
        push!(initial_points, x*dx - dx/2 , y*dy - dy/2, dx*dy, dx*dy, false)
    end
end
push!(export_data, initial_points)


# Wind parameters
wind_speed = 4  # Random wind speed from 0 to 5
global wind_direction = deg2rad(270)#3*pi/2# Random wind direction in radians


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
                    if wind_speed * cos(wind_direction - atan((2-index[2]),(2-index[1]))) * prob_spread > rand()# || prob_spread > rand() #indices changed to transform the coordinate system.
                        new_grid[i, j] = FIRE #the i is the y coordinate, and j is the x coordinate. In terms of grid, i=row and j=column.
                        push!(points,  i*dx - dx/2 , j*dy - dy/2, dx*dy, dx*dy, false)
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
    p = heatmap(1:grid_size[2], 1:grid_size[1], grid', c=:viridis, clim=(0, 2), color=:grays, xticks = (1:25:grid_size[2], 0:25*dx:grid_size[2]*dx), yticks = (1:25:grid_size[1], 0:25*dy:grid_size[1]*dy)  ) #The transpose is done on grid, in order to change the coordinates system with x on x axis and y on y axis.
    display(p)
    #sleep(0.5)

    push!(export_data, points)
end

# Adjusting the ticks and labels
# xticks!(1:2:size(data, 2), 0:2:size(data, 2)-1)
# yticks!(1:2:size(data, 1), 0:2:size(data, 1)-1)


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
