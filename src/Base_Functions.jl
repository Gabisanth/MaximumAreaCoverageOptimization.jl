module Base_Functions

export Circle
export allocate_even_circles
export plot_circle



export draw
export distance
export coincident
export contained
export intersection
export outside
export sort_asc_angle!
export point_on_circle
export shoelace
export associate

using LinearAlgebra, Rotations
using Random
using RobotDynamics
using Plots

"""
    struct Circle

A circle object

# Attributes:

    - 'x::Float64':             x-position of the circle centre on a 2D Cartesian grid
    - 'y::Float64':             y-position of the circle centre on a 2D Cartesian grid
    - 'R::Float64':             Radius
"""
mutable struct Circle
    x::Float64
    y::Float64
    R::Float64
end


function allocate_even_circles(r_centering_cir::Float64, N, r_uav::Float64, center_x, center_y)
    # Generate circles
    x_arr = Vector{Float64}(undef,0)
    y_arr = Vector{Float64}(undef,0)
    r_arr = Vector{Float64}(undef,0)

    for i in 1:N

        ref_angle = 2*π/N * (i-1)
        
        x = r_centering_cir * cos(ref_angle) + center_x
        y = r_centering_cir * sin(ref_angle) + center_y
        r = r_uav

        push!(x_arr,x)
        push!(y_arr,y)
        push!(r_arr,r)
    end

    return [x_arr;y_arr;r_arr]

end

function MADS_to_ALTRO(z::Vector{Float64})

    N = Int(length(z)/3)
    FOV = 80/180*pi

    x = Vector{RBState}()
    for i in range(1,stop=N)
        x_val = z[i]
        y_val = z[N + i]
        R_val = z[N*2 + i]

        z_val = R_val/tan(FOV/2)

        push!(x, RBState([x_val, y_val, z_val], UnitQuaternion(I), zeros(3), zeros(3))); 
        # positions(3), orientation(4), velocity(3), angular velocity(3)

        # push!(x, RBState([x_val, y_val, z_val], zeros(3), zeros(3), zeros(3))); 
        # positions(3), 
        # Euler angles(3: phi (roll)|theta (pitch)|psi (yaw)), 
        # Linear velocity(3), 
        # Angular velocity(3) 
    end

    return x
end



function plot_circle(center_x, center_y, radius, plot_object, this_color)
    θ = LinRange(0, 2π, 20)
    x = center_x .+ radius * cos.(θ)
    y = center_y .+ radius * sin.(θ)

    plot!(plot_object, x, y, legend = false, color=this_color, fill = true)
end










































"""
    struct Point

A point object

# Attributes:

    - 'x::Float64':             x-position of the point on a 2D Cartesian grid
    - 'y::Float64':             y-position of the point on a 2D Cartesian grid
    - 'AreaRep::Float64':       Amount of area represented by this point
    - 'Importance::Int8':      Scale to show how important this area is to capture
    - 'Captured::Bool           Boolean to state whether this point has been captured or not.
"""
mutable struct Point
    x::Float64
    y::Float64
    AreaRep::Float64
    Importance::Int8
    Captured::Bool
end


"""
    draw(A::Circle, theta1::Float64, theta2::Float64)

Returns 2 vectors of discretised x and y coordinates of points on the circle between the angles

# Arguments:

    - 'A::Circle': A Circle object
    - 'theta1::Float64': First angle
    - 'theta2::Float64': Second angle
"""
# returns x and y vectors of a Circle object (for plotting)
function draw(A,theta1::Float64,theta2::Float64)
    if theta1 > theta2
        theta2 = theta2 + 2*pi
    end
    arr = LinRange(theta1,theta2,101)
    return A.x .+ A.R*cos.(arr), A.y .+ A.R*sin.(arr)
end


"""
    distance(A::Circle, B::Circle)

Returns the Euclidean distance between 2 Circle centres

# Arguments:

    - 'A::Circle': First Circle object
    - 'B::Circle': Second Circle object
"""
# function distance(A::Circle,B::Circle)
function distance(A , B)
    return sqrt((A.x-B.x)^2 + (A.y-B.y)^2)
end

"""
    distance(x1::Float64, y1::Float64, x2::Float64, y2::Float64)

Returns the Euclidean distance between the 2 points (x1,y1) and (x2,y2)

# Arguments:

    - 'x1::Float64': x-coordinate of the first point
    - 'y1::Float64': y-coordinate of the first point
    - 'x2::Float64': x-coordinate of the second point
    - 'y2::Float64': y-coordinate of the second point
"""
function distance(x1::Float64,y1::Float64,x2::Float64,y2::Float64)
    return sqrt((x1-x2)^2 + (y1-y2)^2)
end

"""
    coincident(A::Circle, B::Circle)

Returns 'true' if the 2 Circles are coincident, 'false' otherwise

# Arguments:

    - 'A::Circle': First Circle object
    - 'B::Circle': Second Circle object
"""
# function coincident(A::Circle,B::Circle)
function coincident(A,B)
    d = round(distance(A,B), digits=1)
    if d == 0 && A.R == B.R
        return true
    else
        return false
    end
end

"""
    contained(A::Circle, B::Circle)

Checks if A is entirely contained inside B and vice-versa

If A is contained in B, return A. If B is contained in A, return B.

# Arguments:

    - 'A::Circle': First Circle object
    - 'B::Circle': Second Circle object
"""
# function contained(A::Circle,B::Circle)
function contained(A,B)
    d = distance(A,B)
    if d <= abs(A.R - B.R)
    # if d < abs(A.R - B.R)
        if A.R < B.R
            return A
        else
            return B
        end
    end
    return
end


function pure_contained(A,B)
    d = distance(A,B)
    # if d <= abs(A.R - B.R)
    if d < abs(A.R - B.R)
        if A.R < B.R
            return A
        else
            return B
        end
    end
    return
end

"""
    intersection(A::Circle, B::Circle)

Returns the intersection coordinates of 2 Circles, in order 'x1,y1,x2,y2'

Returns 'nothing' if the 2 Circles do not intersect or are tangent

# Arguments:

    - 'A::Circle': First Circle object
    - 'B::Circle': Second Circle object
"""
function intersection(A,B)
    d = distance(A,B)

    if d > A.R + B.R                        # non-intersecting
        return nothing
    elseif d <= abs(A.R - B.R)              # one circle within another
        return nothing
    else
        a = (d^2+A.R^2-B.R^2)/(2*d)
        h = sqrt(abs(A.R^2-a^2))

        varx = A.x + a*(B.x-A.x)/d
        vary = A.y + a*(B.y-A.y)/d

        # Original
        # x1 = varx + h*(B.y-A.y)/d
        # y1 = vary - h*(B.x-A.x)/d
        # x2 = varx - h*(B.y-A.y)/d
        # y2 = vary + h*(B.x-A.x)/d

        # Defined by LZD on 2023.06.08
        x1 = varx - h/a*(vary - A.y)
        y1 = vary + h/a*(varx - A.x)
        x2 = varx + h/a*(vary - A.y)
        y2 = vary - h/a*(varx - A.x)

        if isapprox(distance(x1,y1,x2,y2), 0.0, atol=0.01) # tangent circles -> we take it as non-intersecting
            return nothing
        end

        return x1,y1,x2,y2
        
    end
end

"""
    boundary(A::Circle, P::Point)

Returns 'true' if Point P is inside Circle A, 'false' otherwise

# Arguments:

    - 'A::Circle': The Circle object
    - 'P::Point': The Point object
"""
function outside(A::Circle,P::Point)
    if round((P.x-A.x)^2 + (P.y-A.y)^2 - A.R^2, digits=2) >= 0.0
        return true
    else 
        return false
    end
end

"""
    sort_asc_angle!(A::Circle, Array::Vector{Point})

Sorts a Vector of Points relative to a Circle (centre) in ascending order of polar angle

# Arguments:

    - 'A::Circle': The Circle object (The reference)
    - 'Array::Vector{Point}': The vector of Point objects
"""
function sort_asc_angle!(A::Circle, Array::Vector{Point})
    mean_x = A.x
    mean_y = A.y

    sort_acw!(Array,mean_x,mean_y)
end

"""
    sort_acw!(Array::Any, x::Float64, y::Float64)

Sorts a Vector of Points and/or Circles around the (x,y) coordinates in anticlockwise order

# Arguments:

    - 'Array::Any': A vector of either Circle or Point objects
    - 'x::Float64': x-coordinate (reference point)
    - 'y::Float64': y-coordinate (reference point)
"""
function sort_acw!(Array::Any,x::Float64,y::Float64)
    N = Int(length(Array))

    angles = [mod(atan(point.y-y, point.x-x),2*pi) for point in Array] #???
    for i in 1:N
        for j in i+1:N
            if angles[j] < angles[i]
                tmp = angles[i]
                angles[i] = angles[j]
                angles[j] = tmp

                tmp2 = Array[i]
                Array[i] = Array[j]
                Array[j] = tmp2
            end
        end
    end
end

"""
    point_on_circle(A::Circle, Theta::Float64)

Returns a Point on a Circle given a polar angle

# Arguments:

    -'A::Circle': The Circle object
    -theta::Float64': The angle value (rad)
"""
function point_on_circle(A::Circle,theta::Float64)
    x = A.x + A.R*cos(theta)
    y = A.y + A.R*sin(theta)
    return Point(x,y,[])
end

"""
    shoelace(Array::Vector{Point})

Returns the area of a polygon described by a sorted Vector of Points using the Shoelace algorithm

# Arguments:

    -Array::Vector{Point}': A vector of Point objects
"""
function shoelace(Array::Vector{Point})
    xarr = [point.x for point in Array]
    yarr = [point.y for point in Array]

    dum1 = dot(xarr,circshift(yarr,1))
    dum2 = dot(yarr,circshift(xarr,1))

    area = 0.5*broadcast(abs,(dum1-dum2))
    
    return area
end

"""
    associate(Array::Vector{Any})

Returns a vector, with each row containing rows in Array that have links between the Association_Object(s)

# Arguments:

    'Array::Vector{Any}': A vector with each row of the form [[Association_Object(s)], Link_Object]
    
# Example:

Input = [
    [[1,2,3],A],
    [[2,4,6],B],
    [[6,7,8],C],
    [[10,11,12],D],
    [[12,24,36],E]
]

Output = [
    [[[1,2,3],A], [[2,4,6],B], [[6,7,8],C]],
    [[[10,11,12],D], [[12,24,36],E]]
]

## Explanation:
[1,2,3] is 'associated' with [2,4,6] through element '2', and [2,4,6] is 'associated' with [6,7,8] through element '6'

[10,11,12] is 'associated' with [12,24,36] through element '12'

Note that [1,2,3] is not directly 'associated' with [6,7,8]; this function considers implicit 'association' through the intermediary [2,4,6]
"""
function associate(Array::Vector{Any})

    global dummy = []
    push!(dummy,Array[1])   ## Note: push!() <-> splice!()
    splice!(Array,1)

    final = []

    while size(Array)[1] != 0
        super_break = false

        for i in range(1,stop=size(dummy)[1])
            for j in range(1,stop=size(Array)[1])
                var1 = dummy[i][1]
                var2 = Array[j][1]

                common = intersect(var1,var2)

                if size(common)[1] != 0 # there exists a common object
                    push!(dummy,Array[j])
                    splice!(Array,j)
                    super_break = true
                    break
                end

                if i == size(dummy)[1] && j == size(Array)[1] # no more common Association_Object(s) in Array
                    push!(final,dummy)
                    global dummy = []
                    push!(dummy,Array[1])
                    splice!(Array,1)
                end
                
            end

            if super_break
                break
            end
        end

        if size(Array)[1] == 0
            push!(final,dummy)
        end

    end

    return final
end


end #module end