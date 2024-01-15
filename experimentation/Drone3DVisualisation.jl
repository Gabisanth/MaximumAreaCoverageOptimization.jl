#####METHOD 1: USING MESHCAT
using MeshCat, GeometryBasics, CoordinateTransformations, Rotations

# Define rotation angles
roll_angle = π / 4  # Example rotation angles (in radians)
pitch_angle = π / 6
yaw_angle = π / 3

# # Create rotation matrices for roll, pitch, and yaw
# roll_rot = LinearMap(RotX(roll_angle))
# pitch_rot = LinearMap(RotY(pitch_angle))
# yaw_rot = LinearMap(RotZ(yaw_angle))

# print(yaw_rot)

# # Combine the rotation matrices
# total_rotation = roll_rot * pitch_rot * yaw_rot


# Create a MeshCat visualizer
vis = Visualizer()

# Open the visualizer in your default browser
open(vis)

# setobject!(vis["/object"], 
#     Rect(Vec(0., 0, 0), Vec(0.1, 0.2, 0.3)))

# obj_path = "Drone Frame v2.obj"  # Replace with your object file path
# mesh = Mesh(load(obj_path))
# setobject!(vis["/object"], mesh)

quadGeometry = HomogenousMesh(
  vertices = [Point(0,0,0), Point(0,1,0), Point(0, 0, 1), Point(0, 1, 1)],
  normals = [Point(1,0,0),Point(1,0,0),Point(1,0,0),Point(1,0,0)],
  faces = [Rect(Vec(0., 0, 0), Vec(0.1, 0.2, 0.3)), Rect(Vec(0., 0, 0), Vec(0.1, 0.2, 0.3))],
  texturecoordinates = [Point(0.0f0,0), Point(1.0f0,0), Point(0.0f0,1), Point(1.0f0,1)]
)
setobject!(vis["/object"], quadGeometry)


anim = Animation()

atframe(anim, 0) do
    # within the context of atframe, calls to 
    # `settransform!` and `setprop!` are intercepted
    # and recorded in `anim` instead of having any
    # effect on `vis`.
    settransform!(vis["/object"], Translation(0., 0, 0))
end

for i in 1:5

    atframe(anim, i*20) do
        settransform!(vis["/object"], Translation(0., i, 0))
    end

end
# `setanimation!()` actually sends the animation to the
# viewer. By default, the viewer will play the animation
# right away. To avoid that, you can also pass `play=false`. 
setanimation!(vis, anim)



