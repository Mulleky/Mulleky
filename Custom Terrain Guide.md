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

# How to actually get the STL mesh into Gazebo?

To import the mesh into gazebo we need: a custom model terrain folder, and a custom world terrain file. 

To create the custom model terrain folder do this:
* Go here: `/home/carlos/PX4-Autopilot/Tools/simulation/gz/models` and make a new folder named "custom_terrain" (or something else)
* Inside the custom_terrain folder make a meshes folder and add the stl mesh file
* Inside custom_terrain make a `.sdf` and a `.config` file of the terrain

To create the custom world terrain file do this:
* Go here: `/home/carlos/PX4-Autopilot/Tools/simulation/gz/worlds`
* Inside the worlds folder make a `.sdf` file for the custom terrain, maybe called "lunar_terrain.sdf"
* When making the custom terrain .sdf file you can clone the default world and adjust it to use the custom terrain mesh
     * Inside the .sdf file mention this: `<uri>model://custom_terrain</uri>`. "custom_terrain" is the name of folder that has the custom mesh (the custom model terrain folder in the above step)
