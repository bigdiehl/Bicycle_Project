PACKAGE DESCRIPTION: This packages contain the following:
1. The urdf (or xacro) files that describe the physical bicycle models
2. The mesh (i.e. .stl) files that are used for contact/collision modeling
3. Rviz files 
4. Launch files

MODELS:
1. Basic Collision Model - The first model I made before learning about parameterization 
that was compatible with Whipple model. Might still be useful in future.
2. Baseline Model - New CAD model that is geometrically parameterized by wheel base, trail, 
wheel radius, and steering axis tilt. These define the shape of the bicycle. With these 
geometric parameters defined properly, one can define the mass and inertial parameters independently, 
or use values from the CAD model. 
3. Baseline Model with Gyro - TBD. 
4. Electrical Bicycle Model - TBD. 

LAUNCH FILES:
1. bicycle_gazebo - generic launch into gazebo. 
2. bicycle_rviz - generic launch in rviz
3. bicycle_rviz_test - launch into rviz with joint controller bars