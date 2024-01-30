module AreaCoverageCalculation

include("Base_Functions.jl")
using .Base_Functions

export createPOI

#Collect/ create Info about the areas of interest. (Coordinates of points which represent a specific area size at the location.)
#This function is used only for testing purposes. The list of points should come from the forest fire model.
function createPOI(dx::Float64, dy::Float64, x_length::Float64, y_length::Float64)
    points = Vector{Vector{Float64}}()
    
    for i in 1:x_length
        for j in 1:y_length
            push!(points, [i-dx/2, j-dy/2, dx*dy, 1.0, false]) #[x coordinate, y coordinate, area represented, Importance level, Area Covered? (Boolean)]
        end
    end

    return points
end


"""
    make_circles(arr::Vector{Float64})

Returns a Vector of Circle objects

# Arguments:

    - 'arr::{Vector{Float64}}': Vector of x,y coordinates with radii values, in the order [x;y;R]
"""
function make_circles(arr::Vector{Float64})
    N = Int(length(arr)/3)

    circles = Vector{Circle}()
    for i in 1:N
        x = arr[i]
        y = arr[N + i]
        R = arr[2*N + i]
        push!(circles, Circle(x,y,R))
    end
    
    return circles
end

#Makes the array of circle coordinates and radii.
function make_MADS(circles)::Vector{Float64}  
    x_ = []
    y_ = []
    r_ = []
    for i in eachindex(circles)
        this_cir = circles[i]
        push!(x_, this_cir.x)
        push!(y_, this_cir.y)
        push!(r_, this_cir.R)
    end
    return [x_; y_; r_]
end


#Iterate through these points, and for each point, iterate through number of UAVs to see if any of the UAVs cover it. If covered, then add the area to the total area covered.
function calculateArea(circles::Vector{Float64}, points::Vector{Vector{Float64}})
    area_covered = 0.0
    N_circles = Int(length(circles)/3)

    for p in eachindex(points)
        for c in 1:N_circles
            #if points[p][5] == false
            if sqrt((points[p][1] - circles[c])^2 + (points[p][2]-circles[N_circles+c])^2) < circles[2*N_circles+c]
                area_covered += points[p][3]
                #points[p][5] = true
                break #if the current point in iteration is captured then can continue to next point.
            end
            #end
        end
    end

    #Output the total area covered.
    return area_covered
end

#Before running the Area Optimization algorithms, any points that have been covered by UAV shall be removed.
function rmvCoveredPOI(circles::Array, points::Array)

    #points_copy = points
    N_circles = Int(length(circles)/3)
    indices_to_delete = []

    for p in eachindex(points)
        for c in 1:N_circles
            if points[p][5] == false
                if sqrt((points[p][1] - circles[c])^2 + (points[p][2]-circles[N_circles+c])^2) < circles[2*N_circles+c]
                    push!(indices_to_delete, p)
                    break #If a specific point has been calculated to be captured then move onto the next point.
                end
            end
        end
    end

    deleteat!(points, indices_to_delete) #remove captured points from the list.

    #Output the total area covered.
    return points

end


end