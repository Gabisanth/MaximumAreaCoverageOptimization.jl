module ObstacleModelling

# Function to generate evenly spaced points
function generate_points(num_points, x_min, x_max, y_min, y_max)
    # Determine the aspect ratio of the rectangular region
    width = x_max - x_min
    height = y_max - y_min
    aspect_ratio = width / height

    # Calculate an approximate number of columns and rows
    num_columns = ceil(Int, sqrt(num_points * aspect_ratio))
    num_rows = ceil(Int, num_points / num_columns)

    # Calculate the spacing between points
    spacing_x = width / (num_columns - 1)
    spacing_y = height / (num_rows - 1)

    # Create an array to store the points
    points = []

    # Generate the points
    for i in 0:num_columns-1
        for j in 0:num_rows-1
            x = x_min + i * spacing_x + rand([-10, 0, 10])
            y = y_min + j * spacing_y + rand([-10, 0, 10])
            push!(points, [x, y])
            # Break if we reach the desired number of points
            if length(points) == num_points
                return points
            end
        end
    end
    return points
end

end