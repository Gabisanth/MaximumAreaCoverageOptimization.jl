module AreaCoverageCalculation


#Collect/ create Info about the areas of interest. (Coordinates of points which represent a specific area size at the location.)
function createPOI(dx::Float64, dy::Float64)
    points = []
    
    for i in 1:50
        for j in 1:50
            push!(points, [i-dx/2, j-dy/2, dx*dy, 1, false]) #[x coordinate, y coordinate, area represented, Importance level, Area Covered? (Boolean)]
        end
    end

    return points
end

#Iterate through these points, and for each point, iterate through number of UAVs to see if any of the UAVs cover it. If covered, then add the area to the total area covered.
#And remove it from the list of areas of interest.


area_covered = 0.0
circles = [[5.5,6.2,10.0], [30.2, 25, 20.0]]


for p in points
    for c in circles
        if p[5] == false
            if sqrt((p[1]-c[1])^2 + (p[2]-c[2])^2) < c[3]
                global area_covered += p[3]
            end
        end
    end
end

#Output the total area covered.
print(area_covered)

end