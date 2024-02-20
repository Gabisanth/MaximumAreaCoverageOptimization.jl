##Trial with 2 agents A and B.
using LinearAlgebra

using JuMP
import Ipopt
using LinearAlgebra

using Plots
plotly()  # Set the backend to Plotly
# import Random
# import Statistics
# import Test

#Input: R_A, R_B (radius of each agent), P_A, P_B (coordinates of their centres), V_A, V_B (current velocity vectors for each agent).
R_A = 2
R_B = 2

P_A = [0 0 0]
P_B = [5 5 5]

V_A = [1 1 1]
V_B = [-1 -1 -1]





function angle_between_vectors(v1, v2) 
    #returns angles between 0 and 180.
    angle = acosd(dot(v1, v2)/ (norm(v1) * norm(v2)))
    return angle
end

function perpendicular_vectors(vector) 
    # Normalize the input vector
    normalized_vector = normalize(vector)
    
    # Unit vector along x axis
    unit_x = [1.0 0.0 0.0]
    
    # Compute cross product of normalized vector with unit x vector
    cross_product = cross(vec(normalized_vector), vec(unit_x))
    
    # If cross product is zero vector, use unit y vector instead
    if cross_product == [0.0, 0.0, 0.0]
        unit_y = [0.0, 1.0, 0.0]
        cross_product = cross(vec(normalized_vector), vec(unit_y))
    end
    
    # Take cross product of normalized vector with the first cross product
    v1 = cross(vec(normalized_vector), vec(cross_product))
    
    return cross_product, v1
end

function rotate_about_origin_spherical(vector, psi, elevation)
    # Convert spherical coordinates to Cartesian coordinates
    r = norm(vector)
    x = vector[1]
    y = vector[2]
    z = vector[3]

    # Compute azimuthal angle (theta)
    theta = atand(y, x) #takes into account the correct quadrant.
    if theta == 0
        if x == -1
            theta = -180
        end
    end


    if x == 0 && y == 0
        theta = 0
    end

    if theta < 0
        theta += 360
    end
    
    # Compute polar elevation angle (phi)
    phi = acosd(z / r)

    #print(phi)
    # if x < 0
    #     phi = -phi
    # end
    
    # Perform the rotation in spherical coordinates
    theta += psi

    if x < 0
        phi -= elevation
    else
        phi += elevation
    end

    #print(theta)
    
    # Convert back to Cartesian coordinates
    #r = sqrt(x^2 + y^2 + z^2)
    #phi = clamp(phi, 0, π)  # Clamp phi within [0, π] range to ensure it stays within spherical coordinates
    #theta = atand(y/ x)
    
    # Convert back to vector
    rotated_vector = [r * sind(phi) * cosd(theta), r * sind(phi) * sind(theta), r * cosd(phi)]
    
    return rotated_vector
end


function rotate_about_arbitrary_axis(vector, axis, angle)
    # Normalize the axis vector
    axis_norm = normalize(axis)
    
    # Calculate components of rotation matrix
    cos_angle = cos(angle)
    sin_angle = sin(angle)
    ux = axis_norm[1]
    uy = axis_norm[2]
    uz = axis_norm[3]
    ux2 = ux^2
    uy2 = uy^2
    uz2 = uz^2
    uxy = ux * uy
    uyz = uy * uz
    uzx = uz * ux
    
    # Define the rotation matrix using Rodrigues' rotation formula
    rotation_matrix = [
        cos_angle + ux2*(1-cos_angle)       ux*uy*(1-cos_angle) - uz*sin_angle   ux*uz*(1-cos_angle) + uy*sin_angle;
        ux*uy*(1-cos_angle) + uz*sin_angle  cos_angle + uy2*(1-cos_angle)        uy*uz*(1-cos_angle) - ux*sin_angle;
        ux*uz*(1-cos_angle) - uy*sin_angle  uy*uz*(1-cos_angle) + ux*sin_angle    cos_angle + uz2*(1-cos_angle)
    ]
    
    # Convert vector to column matrix
    vector_matrix = reshape(vector, 3, 1)
    
    # Rotate the vector
    rotated_vector_matrix = rotation_matrix * vector_matrix
    
    # Convert back to vector
    rotated_vector = reshape(rotated_vector_matrix, 3)
    
    return normalize(rotated_vector)
end


function angle_between_vector_and_plane(vector, plane_normal)
    # Normalize the vectors
    vector_norm = normalize(vector)
    plane_normal_norm = normalize(plane_normal)
    
    # Calculate the dot product between the vector and the plane normal
    dot_product = dot(vector_norm, plane_normal_norm)
    
    # Calculate the angle using the arccosine function
    angle = 90 - acosd(dot_product)

    
    return angle
end

function angle_between_planes(plane1_normal, plane2_normal)
    # Normalize normal vectors
    plane1_normal_normalized = normalize(plane1_normal)
    plane2_normal_normalized = normalize(plane2_normal)
    
    # Compute dot product
    dot_product = dot(plane1_normal_normalized, plane2_normal_normalized)
    
    # Calculate angle 
    angle = acosd(dot_product)
    
    return angle
end

function vector_between_planes(vector, normal1, normal2)
    dot_product1 = dot(vector, normal1)
    dot_product2 = dot(vector, normal2)
    
    # Check if the dot products have same signs
    if dot_product1 > 0 && dot_product2 > 0
        return true  # Vector lies between the planes
    else
        return false  # Vector does not lie between the planes
    end
end

function shortest_vector_to_plane(point, normal)
    ##NEED TO FIX!!!
    #project the position vector onto the (negative) normal vector.
    alpha = angle_between_vectors(point, normal)
    theta = 90 - alpha

    projection_magnitude = norm(point)*sind(theta)
    projection = projection_magnitude * -1 * normal
    
    return projection
end


function rotate_x(vector, angle)
    # Convert angle to radians
    angle_rad = deg2rad(angle)
    
    # Define the rotation matrix
    rotation_matrix = [
        1      0           0;
        0      cos(angle_rad)  -sin(angle_rad);
        0      sin(angle_rad)   cos(angle_rad)
    ]
    
    # Convert vector to column matrix
    vector_matrix = reshape(vector, 3, 1)
    
    # Rotate the vector
    rotated_vector_matrix = rotation_matrix * vector_matrix
    
    # Convert back to vector
    rotated_vector = reshape(rotated_vector_matrix, 3)
    
    return rotated_vector
end

function rotate_y(vector, angle)
    # Convert angle to radians
    angle_rad = deg2rad(angle)
    
    # Define the rotation matrix
    rotation_matrix = [
        cos(angle_rad)   0   sin(angle_rad);
        0                1   0;
        -sin(angle_rad)  0   cos(angle_rad)
    ]
    
    # Convert vector to column matrix
    vector_matrix = reshape(vector, 3, 1)
    
    # Rotate the vector
    rotated_vector_matrix = rotation_matrix * vector_matrix
    
    # Convert back to vector
    rotated_vector = reshape(rotated_vector_matrix, 3)
    
    return rotated_vector
end

function rotate_z(vector, angle)
    # Convert angle to radians
    angle_rad = deg2rad(angle)
    
    # Define the rotation matrix
    rotation_matrix = [
        cos(angle_rad)  -sin(angle_rad)  0;
        sin(angle_rad)   cos(angle_rad)  0;
        0                0               1
    ]
    
    # Convert vector to column matrix
    vector_matrix = reshape(vector, 3, 1)
    
    # Rotate the vector
    rotated_vector_matrix = rotation_matrix * vector_matrix
    
    # Convert back to vector
    rotated_vector = reshape(rotated_vector_matrix, 3)
    
    return rotated_vector
end





function ORCA_2D(R_A, R_B, P_A, P_B, V_A, V_B) #atm it only works if the relative velocity vector is within the velocity obstacle.

    #Get edges of velocity obstacle for A (and B).
    a = P_B[2] - P_A[2]
    b = P_B[1] - P_A[1]
    r = R_A + R_B
    m1 = (-4*a*b + sqrt(64*(a^2)*(r^2) + 64*(b^2)*(r^2) - 64*(r^4))) / (8*(r^2) - 8*(b^2))
    m2 = (-4*a*b - sqrt(64*(a^2)*(r^2) + 64*(b^2)*(r^2) - 64*(r^4))) / (8*(r^2) - 8*(b^2))


    d1 = [1 m1]
    d2 = [1 m2]
    d1_unit = d1/norm(d1)
    d2_unit = d2/norm(d2) 

    #Get u vector. Smallest vector to edge of velocity obstacle cone.
    p = V_A - V_B

    u_array = [dot(p,d1_unit)*d1_unit - p, dot(p,d2_unit)*d2_unit - p]
    u_norm_array = [norm(u_array[1]), norm(u_array[2])]
    u_min_index = argmin(u_norm_array)
    u = u_array[u_min_index]
    n = u/norm(u)

    #Perform the optimization for velocity of A (and B).
    model = Model(Ipopt.Optimizer)
    set_silent(model)
    @variable(model, Vx)
    @variable(model, Vy)
    @objective(model, Min, (Vx-V_A[1])^2 + (Vy-V_A[2])^2)
    @constraint(model, c1, dot(([Vx Vy] - V_A - 0.5*u), n) >= 0)
    optimize!(model)

    #Output: New Velocity Vector for A (and B).
    V_opt = [value(Vx) value(Vy)]
    return V_opt

end


function ORCA_3D(R_A, R_B, P_A, P_B, V_A, V_B)
    ##1. Get edges of velocity obstacle for A induced by B.
    centre = P_B - P_A #position vector to the centre of the sphere.
    r = R_A + R_B

    cone_angle = asind(r / norm(centre))

    #Need 2 sets of edges to (approximate) the cone. (actually will be a square based pyramid instead of a cone).
    axis1, axis2 = perpendicular_vectors(centre)

    cone_edge_1 = rotate_about_arbitrary_axis(centre, axis1, cone_angle*pi/180)
    cone_edge_2 = rotate_about_arbitrary_axis(centre, axis1, -cone_angle*pi/180)

    cone_edge_3 = rotate_about_arbitrary_axis(centre, axis2, cone_angle*pi/180)
    cone_edge_4 = rotate_about_arbitrary_axis(centre, axis2, -cone_angle*pi/180)

    ##2. Check if relative velocity vector is inside velocity obstacle.
    #Find normal vector to each of the cone edge planes.
    n1 = -1*cone_edge_1*(norm(centre)*cosd(cone_angle)) + vec(centre) #normal vector is the centre to where the cone_edge touches the sphere.
    n2 = -1*cone_edge_2*(norm(centre)*cosd(cone_angle)) + vec(centre)
    n3 = -1*cone_edge_3*(norm(centre)*cosd(cone_angle)) + vec(centre)
    n4 = -1*cone_edge_4*(norm(centre)*cosd(cone_angle)) + vec(centre)

    rel_vel = V_A - V_B

    # println(angle_between_planes(-n1,n2))
    # println(angle_between_vector_and_plane(rel_vel, n1))
    # println(angle_between_vector_and_plane(rel_vel,n2))

    # println(angle_between_planes(-n3,n4))
    # println(angle_between_vector_and_plane(rel_vel, n3))
    # println(angle_between_vector_and_plane(rel_vel,n4))

    #check if relative velocity is within the velocity obstacle.
    if vector_between_planes(rel_vel, n1, n2) && vector_between_planes(rel_vel, n3, n4)
        collision = true
    else
        collision = false
    end

    #Get u vector. Smallest vector to edge of velocity obstacle cone.
    
    u1 = shortest_vector_to_plane(rel_vel,n1)
    u2 = shortest_vector_to_plane(rel_vel,n2)
    u3 = shortest_vector_to_plane(rel_vel,n3)
    u4 = shortest_vector_to_plane(rel_vel,n4)


    u_array = [u1,u2,u3,u4]
    u_norm_array = [norm(u_array[1]), norm(u_array[2]), norm(u_array[3]), norm(u_array[4])]
    u_min_index = argmin(u_norm_array)
    u = u_array[u_min_index]
    n = u/norm(u)
    u = [u[1] u[2] u[3]]
    n = [n[1] n[2] n[3]]


  

    #Perform the optimization for velocity of A (and B).
    model = Model(Ipopt.Optimizer)
    set_silent(model)
    @variable(model, Vx)
    @variable(model, Vy)
    @variable(model, Vz)
    @objective(model, Min, (Vx-V_A[1])^2 + (Vy-V_A[2])^2 + (Vz-V_A[3])^2)
    @constraint(model, c1, dot([Vx Vy Vz] - V_A - 0.5*u, n) >= 0)
    optimize!(model)

    #Output: New Velocity Vector for A (and B).
    V_opt = [value(Vx) value(Vy) value(Vz)]
    return V_opt, collision


    # println(n1,n2)
    # println(n3,n4)
    # println(u1,u2,u3,u4)
    
    #println(angle_between_vectors([1 1 1], [3 3 3]))

    #println(angle_between_vectors([], cone_edge_1))

    #return(angle_between_vectors(n3, cone_edge_3))

    


end

# print(ORCA_3D(R_A, R_B, P_A, P_B, V_A, V_B))

# # println(rotate_about_origin_spherical([-1.0 -1.0 -1.0], -45.0, 0.0))
# # println(rotate_about_origin_spherical([-1.0 -1.0 -1.0], 45.0, 0.0))

# # println(angle_between_vectors([-1.4142135623730945, 0.0, -1.0000000000000007], [0.0, -1.4142135623730945, -1.0000000000000007]))


function ORCA_3DCone(R_A, R_B, P_A, P_B, V_A, V_B, time_horizon, time_step)
    invTimeHorizon = 1.0/time_horizon

    relativePosition = P_B - P_A
    relativeVelocity = V_A - V_B
    distSq = norm(relativePosition)^2
    combinedRadius = R_A + R_B
    combinedRadiusSq = norm(combinedRadius)

    if distSq > combinedRadiusSq
        #No collision.
        w = relativeVelocity - invTimeHorizon * relativePosition

        #Vector from cutoff centre to relative velocity.
        wLengthSq = norm(w)^2

        dotProduct = dot(w, relativePosition)

        if dotProduct < 0.0 && dotProduct^2 > combinedRadiusSq * wLengthSq
            #Project on cut-off circle.
            wLength = sqrt(wLengthSq)
            unitW = w/wLength

            plane_normal = unitW
            u = (combinedRadius * invTimeHorizon - wLength) * unitW
        else
            #Project on cone.
            a = distSq
            b = dot(relativePosition,relativeVelocity)
            c = norm(relativeVelocity)^2 - norm(cross(vec(relativePosition), vec(relativeVelocity)))^2 / (distSq - combinedRadiusSq)
            t = (b + sqrt(b^2 - a * c)) / a;
            ww = relativeVelocity - t * relativePosition;
            wwLength = norm(ww);
            unitWW = ww / wwLength;

            plane_normal = unitWW;
            u = (combinedRadius * t - wwLength) * unitWW;
        end
        
    else
        #Collision.
        invTimeStep = 1.0 / time_step
        w = relativeVelocity - invTimeStep * relativePosition
        wLength = norm(w)
        unitW = w / wLength

        plane_normal = unitW
        u = (combinedRadius * invTimeStep - wLength) * unitW

    end

    println(u)

    #Perform the optimization for velocity of A (and B).
    model = Model(Ipopt.Optimizer)
    set_silent(model)
    @variable(model, Vx)
    @variable(model, Vy)
    @variable(model, Vz)
    @objective(model, Min, (Vx-V_A[1])^2 + (Vy-V_A[2])^2 + (Vz-V_A[3])^2)
    @constraint(model, c1, dot([Vx Vy Vz] - V_A - 1*u, plane_normal) >= 0)
    optimize!(model)

    #Output: New Velocity Vector for A (and B).
    V_opt = [value(Vx) value(Vy) value(Vz)]
    return V_opt
    
    
end


#println(ORCA_3DCone(R_A, R_B, P_A, P_B, V_A, V_B, 0.5, 0.5))

v1 = (ORCA_3D(R_A, R_B, P_A, P_B, V_A, V_B))
v2 = (ORCA_3D(R_B, R_A, P_B, P_A, V_B, V_A))

print(v1,v2)



