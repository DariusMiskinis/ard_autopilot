# ard_autopilot

  ROS package to control ARDrone landing on ARToolkit fiducial marker  
  Can be run with gazebo simulation or actual ARDrone drone  

Usage:

    $ sh gazebo.sh - to run simulation
    
    $ roslaunch real_world_drone.launch - to run all tools with ARDrone driver
    
    $ sh autopilot.sh - to run landing control module

Before using:

Replace local paths in world files for landing marker description file

  <geometry>
    <mesh>
      <uri>file:///home/dariusm/cws4/src/ard_autopilot/tum_simulator/marker/meshes/artag_01.dae</uri>
      <scale>0 0 0</scale>
    </mesh>
  </geometry>
    
    
