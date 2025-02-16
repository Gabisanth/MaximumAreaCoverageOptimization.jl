module ORCA

export ORCA_3D

using LinearAlgebra
using JuMP
import Ipopt
using LinearAlgebra

#Input: R_A, R_B (radius of each agent), P_A, P_B (coordinates of their centres), V_A, V_B (current velocity vectors for each agent).
R_A = 1
R_B = 1
R_C = 1
R_D = 1

P_A = [0 0 5]
P_B = [10 0 5]
P_C = [5 5 5]
P_D = [5 -5 5]


V_A = [1.3800686994026599 0.00013780006245030148 0.48575274994122086] + reshape(rand(-0.001:0.0000001:0.001, 3), 1, 3)
V_B = [-1.1396914990757603 -0.3395463989085548 -0.47977725204214267] + reshape(rand(-0.001:0.0000001:0.001, 3), 1, 3)
V_C = [0.2515407486958495 -0.46056250126052556 0.0003948999930730791]+ reshape(rand(-0.001:0.0000001:0.001, 3), 1, 3)
V_D = [-0.3539174409687131 0.7378751280079052 0.2885905518767432] + reshape(rand(-0.001:0.0000001:0.001, 3), 1, 3)



function angle_between_vectors(v1, v2) 
    #returns angles between 0 and 180.
    angle = acosd(dot(v1, v2)/ (norm(v1) * norm(v2)))
    return angle
end

function perpendicular_vectors(vector) 
    # #1. Original way: Rotation axis are arbitrarily chosen.
    # # Normalize the input vector
    # normalized_vector = normalize(vector)
    
    # # Unit vector along x axis
    # unit_x = [1.0 0.0 0.0]
    
    # # Compute cross product of normalized vector with unit x vector
    # cross_product = cross(vec(normalized_vector), vec(unit_x))
    
    # # If cross product is zero vector, use unit y vector instead
    # if cross_product == [0.0, 0.0, 0.0]
    #     unit_y = [0.0, 1.0, 0.0]
    #     cross_product = cross(vec(normalized_vector), vec(unit_y))
    # end
    
    # # Take cross product of normalized vector with the first cross product
    # v1 = cross(vec(normalized_vector), vec(cross_product))

    #2. Modified way: Rotation axis chosen, such that one is on the xy plane.
    # Normalize the input vector
    normalized_vector = normalize(vector)
    
    # # Unit vector along z axis
    unit_z = [0.0 0.0 1.0]
    
    # Compute cross product of normalized vector with unit z vector: gives vector on xy plane, perpendicular to position vector of obstacle.
    #Vertical/elevation rotations about this axis.
    axis1 = cross(vec(normalized_vector), vec(unit_z))
    
    
    # Take cross product of normalized vector with the first cross product: gives vector perpendicular to position vector of obstacle.
    #Horizontal/ azimuthal rotations about this axis.
    axis2 = cross(vec(normalized_vector), vec(axis1))

    
    return axis1, axis2
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
    #project the position vector onto the (negative) normal vector.
    alpha = angle_between_vectors(point, normal)
    theta = 90 - alpha

    projection_magnitude = norm(point)*sind(theta)
    projection = projection_magnitude * -1 * normal
    
    return projection
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


function ORCA_3D(R_A, R_B, P_A, P_B, V_A, V_B, responsibility_shares, V_pref)

    u_all = []
    n_all = []
    collision_status = []


    for i in eachindex(R_B)
        ##1. Get edges of velocity obstacle for A induced by B.
        centre = P_B[i] - P_A #position vector to the centre of the sphere.

        r = R_A + R_B[i]


        #Can allow small violations if safety margin has been added to the radii.
        if r > norm(centre)
            cone_angle = 90
        else
            cone_angle = asind(r / norm(centre))
        end

        if R_B[i] != 0.25*1.1
            if norm(centre) < R_B[i] + 0.25 #Actual separation requied between UAV and static obstacle.
                ("Collision with Static Obstacle Detected!")
            end
        else
            if norm(centre) < 0.5 #This is the actual minimum separation required between two UAVs. 
                print("Collision with other UAV Detected!")
            end
        end


        #cone_angle = asind(r / norm(centre))

        #Need 2 sets of edges to (approximate) the cone. (actually will be a square based pyramid instead of a cone).
        axis1, axis2 = perpendicular_vectors(centre)

        if R_B[i] != 0.25*1.1 #i.e. it is not another drone. Will use this to distinguish static obstacles from our cooperative drones.
            cone_edge_1 = rotate_about_arbitrary_axis(centre, axis1, 90*pi/180)
            cone_edge_2 = rotate_about_arbitrary_axis(centre, axis1, -90*pi/180)
        else
            cone_edge_1 = rotate_about_arbitrary_axis(centre, axis1, cone_angle*pi/180)
            cone_edge_2 = rotate_about_arbitrary_axis(centre, axis1, -cone_angle*pi/180)
        end

        cone_edge_3 = rotate_about_arbitrary_axis(centre, axis2, cone_angle*pi/180)
        cone_edge_4 = rotate_about_arbitrary_axis(centre, axis2, -cone_angle*pi/180)

        ##2. Check if relative velocity vector is inside velocity obstacle.
        #Find normal vector to each of the cone edge planes.
        n1 = -1*cone_edge_1*(norm(centre)*cosd(cone_angle)) + vec(centre) #normal vector is the centre to where the cone_edge touches the sphere.
        n2 = -1*cone_edge_2*(norm(centre)*cosd(cone_angle)) + vec(centre)
        n3 = -1*cone_edge_3*(norm(centre)*cosd(cone_angle)) + vec(centre)
        n4 = -1*cone_edge_4*(norm(centre)*cosd(cone_angle)) + vec(centre)

        rel_vel = V_A - V_B[i]

        # println(angle_between_planes(-n1,n2))
        # println(angle_between_vector_and_plane(rel_vel, n1))
        # println(angle_between_vector_and_plane(rel_vel,n2))

        # println(angle_between_planes(-n3,n4))
        # println(angle_between_vector_and_plane(rel_vel, n3))
        # println(angle_between_vector_and_plane(rel_vel,n4))

        #check if relative velocity is within the velocity obstacle.
        if vector_between_planes(rel_vel, n1, n2) && vector_between_planes(rel_vel, n3, n4)
            collision = true

            u1 = shortest_vector_to_plane(rel_vel,n1)
            u2 = shortest_vector_to_plane(rel_vel,n2)
            u3 = shortest_vector_to_plane(rel_vel,n3)
            u4 = shortest_vector_to_plane(rel_vel,n4)
        
            u_array = [u1,u2,u3,u4]
            u_norm_array = [norm(u_array[1]), norm(u_array[2]), norm(u_array[3]), norm(u_array[4])]
            u_min_index = argmin(u_norm_array)
            u = u_array[u_min_index]
        else
            collision = false
            #If relative velocity is outside of the collision cone, then the u will need to be calculated differently.
            if dot(rel_vel, centre) < 0
                u = -1 * rel_vel
            else
                u1 = shortest_vector_to_plane(rel_vel,n1)
                u2 = shortest_vector_to_plane(rel_vel,n2)
                u3 = shortest_vector_to_plane(rel_vel,n3)
                u4 = shortest_vector_to_plane(rel_vel,n4)
            
                u_array = [u1,u2,u3,u4]
                u_norm_array = [norm(u_array[1]), norm(u_array[2]), norm(u_array[3]), norm(u_array[4])]
                u_min_index = argmin(u_norm_array)
                u = u_array[u_min_index]
            end


        end




        # ###Alternative way to find u.
        # centre = P_B[i] - P_A
        # centre = vec([centre[1]; centre[2]; centre[3]])
        # rel_vel = V_A - V_B[i]
        # rel_vel = vec([rel_vel[1]; rel_vel[2]; rel_vel[3]])
        # r = R_A + R_B[i]
        # a = norm(centre)^2
        # b = dot(centre, rel_vel)
        # c = norm(rel_vel)^2 - (norm((cross(centre, rel_vel)))^2)/(a - r^2)
        # t = (b+sqrt(b^2 - a*c))/a
        # ww = rel_vel - t * centre
        # wwLength = norm(ww)
        # unitWW = ww/wwLength
        # u = (r*t - wwLength) * unitWW

        # collision = true


        #Get u and n vector. Smallest vector to edge of velocity obstacle cone.
        n = u/norm(u)
        u = [u[1] u[2] u[3]]
        n = [n[1] n[2] n[3]]
        push!(u_all, u)
        push!(n_all, n)
        push!(collision_status, collision)

    end
  
    #print(collision_status)

    #Perform the optimization for velocity of A (and B).
    model = Model(Ipopt.Optimizer)
    set_silent(model)
    @variable(model, Vx)
    @variable(model, Vy)
    @variable(model, Vz)
    @objective(model, Min, 0.25*((Vx-V_pref[1])^2 + (Vy-V_pref[2])^2 + (Vz-V_pref[3])^2) + 0.75*((Vx-V_A[1])^2 + (Vy-V_A[2])^2 + (Vz-V_A[3])^2))


    for i in eachindex(R_B)
        #@constraint(model, dot([Vx Vy Vz] - V_A - responsibility_shares[i]*u_all[i], n_all[i]) >= 0)

        if collision_status[i]
            @constraint(model, dot([Vx Vy Vz] - V_A - responsibility_shares[i]*(u_all[i]), n_all[i])  >= 0) #can add extra margin to keep it out.
        else
            @constraint(model, dot([Vx Vy Vz] - V_A - responsibility_shares[i]*(u_all[i]), n_all[i])  <= 0) #can add extra margin to keep it out.
        end
    end
    #@constraint(model, ((Vx-V_A[1])^2 + (Vy-V_A[2])^2 + (Vz-V_A[3])^2) <= (1)^2) #Physical limits included.

    optimize!(model)
    

    #Output: New Velocity Vector for A (and B).
    V_opt = [value(Vx) value(Vy) value(Vz)]

    return V_opt, collision_status


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

# vAnew = (ORCA_3D(R_A, [R_B, R_C, R_D], P_A, [P_B, P_C, P_D], V_A, [V_B, V_C, V_D], 0.5, V_A))
# vBnew = (ORCA_3D(R_B, [R_A, R_C, R_D], P_B, [P_A, P_C, P_D], V_B, [V_A, V_C, V_D], 0.5, V_B))
# vCnew = (ORCA_3D(R_C, [R_A, R_B, R_D], P_C, [P_A, P_B, P_D], V_C, [V_A, V_B, V_D], 0.5, V_C))
# vDnew = (ORCA_3D(R_D, [R_A, R_B, R_C], P_D, [P_A, P_B, P_C], V_D, [V_A, V_B, V_C], 0.5, V_D))



# println(V_A)
# println(vAnew)

# println(V_B)
# println(vBnew)

# println(V_C)
# println(vCnew)

# println(V_D)
# println(vDnew)

# println(P_A)
# println(P_B)
# println(P_C)
# println(P_D)

end