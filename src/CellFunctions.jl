module CellFunctions

include("AreaCoverageCalculation.jl")

mutable struct Cells
    points_of_interest::Vector{Vector{Float64}}
    fire_point_xy_ccordinates::Vector{Vector{Float64}}

    #Inner Constructor
    function Cells(points_of_interest, fire_point_xy_ccordinates)
        self = new(points_of_interest, fire_point_xy_ccordinates)

        return self
    end

end



function initialise_POI(self::Cells, environment_type::String)
    if environment_type == "dynamic"
        #Import datapoints (for dynamically changing area of interest.)
        # Path to your Excel file
        file_path = "C://Users//gabis//Desktop//FYP Repos//MaximumAreaCoverageOptimization//src//FirePoints.xlsx"

        # Load the Excel file
        xlsx_file = XLSX.readxlsx(file_path)

        # Access the first sheet in the Excel file
        sheet = xlsx_file["Sheet1"]  # Replace "Sheet1" with the name of your sheet, if different.
        self.points_of_interest = Vector{Vector{Float64}}()
        self.fire_point_xy_ccordinates = Vector{Vector{Float64}}() #only for visualisation purposes.
        subarray_size = 5

        for row in 1:10 #number of rows (i.e. timesteps) to advance into the fire progression initially.
            new_points = vec(sheet[row,:])
            filter!(!ismissing, new_points)


            for i in 1:subarray_size:length(new_points)
                new_point = new_points[i:min(i+subarray_size-1, end)]
                check = new_point[1] .< x_UB .&& new_point[1] .> x_LB  .&& new_point[2] .< y_UB .&& new_point[2] .> y_LB
                if any(check)
                    new_point[4] = (h_max* tan(FOV/2))^2 * pi
                end
                push!(points_of_interest, new_point)
                push!(fire_point_xy_ccordinates, new_point[1:2])
            end
        end

    else
        #Initialise the points of interest. (for static environment)
        self.points_of_interest = AreaCoverageCalculation.createPOI(5.0,5.0,100.0,100.0) #Create initial set of points of interest.
    end

    return self
end

function update_POI(self::Cells, t::Int)
    sheet = xlsx_file["Sheet1"]

    if t != 1 && t%1==0
        new_points = vec(sheet[Int(t/1 + 10),:])
        filter!(!ismissing, new_points)

        for i in 1:subarray_size:length(new_points)
            new_point = new_points[i:min(i+subarray_size-1, end)]
            check = new_point[1] .< x_UB .&& new_point[1] .> x_LB  .&& new_point[2] .< y_UB .&& new_point[2] .> y_LB

            if any(check)
                new_point[4] = (h_max* tan(FOV/2))^2 * pi #largest FOV area possible.
            end
            push!(self.points_of_interest, new_point)
            push!(self.fire_point_xy_ccordinates, new_point[1:2])
        end
    end

    return self
end

function rmvCoveredPOI(self::Cells, circles::Array)

    #points_copy = points
    N_circles = Int(length(circles)/3)
    indices_to_delete = []
    points = self.points_of_interest

    for p in eachindex(points)
        for c in 1:N_circles
            if sqrt((points[p][1] - circles[c])^2 + (points[p][2]-circles[N_circles+c])^2) < circles[2*N_circles+c]
                # if points[p][4] == 250.0 && abs(circles[2*N_circles+c] - 10 * tan((80/180*π)/2)) > 1.0
                #     break
                # else
                push!(indices_to_delete, p)
                break #If a specific point has been calculated to be captured then move onto the next point.
                #end
            end
        end
    end

    deleteat!(points, indices_to_delete) #remove captured points from the list.

    self.points_of_interest

    #Output the total area covered.
    return self

end



end