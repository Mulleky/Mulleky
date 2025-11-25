# How to make custom terrain in Gazebo

First, one of the ways in which you can make a custom terrain is using a mesh (of such terrain). To obtain the mesh you can use blender or MeshLabs and use different types of files like `.tif`,`.xyzi`,etc

For the current SITL sim we're using a `.xyzi` terrain file of the lunar south pole (site 01) obtained here: https://pgda.gsfc.nasa.gov/products/78

## How to get the mesh in STL?
* Open MeshLabs from the terminal and import the terrain file
* Compute the normals as follows:

    * Filters -> normals, curvature, and orientation -> compute normals for point sets
* Then do the surface reconstruction as follows:

    *  Filters -> remeshing, simplification, and reconstruction -> Surface reconstruction: screened poisson
    *  Can use other reconstruction algorithms, but at the moment the others don't recreate the faces of the mesh
* Export mesh as STL
