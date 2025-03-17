# Update the robot meshes

To change the robot meshes, you can use [Blender](https://www.blender.org/), loading the [`ar4.blend`](../meshes/ar4.blend)


After the modifications, you'll need to update the meshes used in [`ar4.urdf.xacro`](../urdf/ar4.urdf.xacro). If new meshes are added, you'll need to add that in the `link` macro, as well as it's material.


If a new material is created, please create an entry in [`materials.xacro`](../urdf/include/materials.xacro).

Before exporting the new meshes as `STL` reset the transforms of all objects with: `ALT+G`.
