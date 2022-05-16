Components and main C++ classes
===============================
Overview and main C++ classes for components. 
Please check detail explanation of those class and other classes in `C++ Documentation <doxygen_generated/html/index.html>`_

Drives
------

| Drives provides feature to control robot. 
| Custom drive class should be created as child class of *RobotVehicleMovementComponent*

**example of class relations**

.. image:: doxygen_generated/html/d9/d8d/class_u_differential_drive_component__coll__graph.png

**main C++ classes**

- `URobotVehicleMovementComponent <doxygen_generated/html/d7/d01/class_u_robot_vehicle_movement_component.html>`_:  Base Robot vehicle movement class which is used as part of #ARobotVehicle. Move robot without considering physics and publish odometry. This supports adapting the robot pose to floor complex surfaces, and following moving platforms such as elevators.
- `UDifferentialDriveComponent <doxygen_generated/html/db/df5/class_u_differential_drive_component.html>`_: Simulate differential drive with 2 wheels considering physics.

Sensors
-------

| Sensor classes which can be used as stand alone actor or component of Robot.
| *RRROS2<sensor name>Component* class is designed to used as part of robot.
| *RRROS2<sensor name>Actor* class is designed to used as stand alone actor and ROS node.

**example of class relations**

.. image:: doxygen_generated/html/dd/da7/class_a_r_r_r_o_s2_camera_actor__coll__graph.png

**main C++ classes**

- `URRROS2BaseSensorComponent <doxygen_generated/html/d0/d58/class_u_r_r_r_o_s2_base_sensor_component.html>`_: Base ROS2 Sensor Component class. Other sensors class should inherit from this class. Uses BaseSensorPublisher class to publish ROS2 msg.
- `URRROS2CameraComponent <doxygen_generated/html/d9/d91/class_u_r_r_r_o_s2_camera_component.html>`_: ROS2 Camera component. Uses USceneCaptureComponent2D.
- `ARRROS2CameraActor  <doxygen_generated/html/d2/d18/class_a_r_r_r_o_s2_camera_actor.html>`_: Standalone camera actor which can be placed in the level with URRROS2CameraComponent.
- `URR2DLidarComponent  <doxygen_generated/html/d4/d87/class_u_r_r2_d_lidar_component.html>`_: ROS2 Lidar component.

**main Blueprint classes classes**

- BP_Lidar_* : Set parameters for specific lidar products.

Robots
------

| Robots has robot vehicle class and controller class.
| Custom robot class should be created as child class of *RobotVehicle* class. 
| *RRRobotVehicleROSController* create ROS2Node and control all drive and sensor components in the *RobotVehicle* class.

**main C++ classes**

- `ARobotVehicle <doxygen_generated/html/d7/d80/class_a_robot_vehicle.html>`_: Base RobotVehicle class. Other robot class should inherit from this class. Has URobotVehicleMovementComponent and initialize sensors.
- `ARRRobotVehicleROSController <doxygen_generated/html/d6/d83/class_a_r_r_robot_vehicle_r_o_s_controller.html>`_: Base Robot ROS controller class. Other robot controller class should inherit from this class. This class owns ROS2Node and provide ROS2 interfaces to control robot such as Twist msg.
- `ATurtlebotBurger <doxygen_generated/html/de/d76/class_a_turtlebot_burger.html>`_: Example of ARobotVehicle
- `ATurtlebotROSController <doxygen_generated/html/dd/d8f/class_a_turtlebot_r_o_s_controller.html>`_: Example of ARRRobotVehicleROSController

**main Blueprint classes classes**

- Turtlebot3: example of robots.
    - BP_TurtlebotROSController.uasset: inherit from C++ ARRRobotVehicleROSController. Example of construct robot controller with Blueprint.  
    - Models: Static and skeletal meshes and materials.
    - Physics: physics model with UDifferentialDriveComponent
        - BP_TurtlebotBurger: inherit from C++ ATurtlebotBurger and set static meshes and parameters.
        - BP_TurtlebotWaffle: inherit from C++ ARobotVehicle. Example of construct robot from Blueprint. 
    - Kinematics: kinematic model with URobotVehicleMovementComponent 
        - BP_TurtlebotBurgerVehicle: inherit from C++ ARobotVehicle and set static meshes and parameters.
        - BP_TurtlebotWaffleVehicle: inherit from C++ ARobotVehicle. Example of construct robot with Blueprint. 

Core
----

| Core has base classes which are directly or indirectly used by other components in RapyutaSimulationPlugins Plugins.
| Core also has util classes as well. 

**main C++ classes(todo)**

- `ARRBaseActor <doxygen_generated/html/d9/d3d/class_a_r_r_base_actor.html>`_: Base actor class for all Rapyuta Sim actors
- `URRActorCommon <doxygen_generated/html/df/d29/class_u_r_r_actor_common.html>`_: todo
- Game objects
    - `ARRGameMode <doxygen_generated/html/d1/dbb/class_a_r_r_game_mode.html>`_: GameMode with specific setting, asset loading and ROS2 interface via ClockPublisher and ASimulationState.
    - `ARRGameState <doxygen_generated/html/d5/d01/class_a_r_r_game_state.html>`_: todo
    - `URRGameInstance <doxygen_generated/html/d1/d8e/class_u_r_r_game_instance.html>`_: This is a globally accessible instanced UObject that can store run-time data to be commonly accessed between levels and Scene instances.
    - `URRGameSingleton <doxygen_generated/html/d8/def/class_u_r_r_game_singleton.html>`_: todo
- Mesh
    - `ARRMeshActor <doxygen_generated/html/dd/de7/class_a_r_r_mesh_actor.html>`_: todo
    - `URRProceduralMeshComponent <doxygen_generated/html/dc/d1a/class_u_r_r_procedural_mesh_component.html>`_: Procedural mesh components. this class is used to spawn robot and object from ROS2 service.
- Scene
    - `ARRSceneDirector <doxygen_generated/html/d6/d2f/class_a_r_r_scene_director.html>`_: todo
    - `URRSceneInstance <doxygen_generated/html/d7/d37/class_u_r_r_scene_instance.html>`_: todo
- Utils
    - `URRCoreUtils <doxygen_generated/html/de/db6/class_u_r_r_core_utils.html>`_: todo
    - `URRConversionUtils <doxygen_generated/html/d4/dc1/class_u_r_r_conversion_utils.html>`_: Data Conversion between ROS and UE. m <-> cm and Left handed <-> Right handed.
    - `URRGeneralUtils <doxygen_generated/html/d5/d98/class_u_r_r_general_utils.html>`_: Other utils which do not belong to other utils.

Tools
-----

| Offline and runtime tools. 
| *URRROS2<msg name or others>Publisher* class is publisher of a specific msg type or purpose. Child class of `UROS2Publisher <https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dd4/class_u_r_o_s2_publisher.html>`_.

**main C++ classes**

- `ASimulationState <doxygen_generated/html/d2/dde/class_a_simulation_state.html>`_: Provide spawn/delete/attach/set/get ROS2 interfaces. Typically this class is initialized from GameMode.
- `URRROS2ClockPublisher <doxygen_generated/html/d5/dc2/class_u_r_r_r_o_s2_clock_publisher.html>`_: Publish /clock topic. Typically this class is initialized from GameMode.
- `OccupancyMapGenerator <doxygen_generated/html/d2/dde/class_a_occupancy_map_generator.html>`_: Generate 2D occupancy map for navigation/localization.
- `URRROS2BaseSensorPublisher <doxygen_generated/html/d5/d69/class_u_r_r_r_o_s2_base_sensor_publisher.html>`_: Base Sensor Publisher class. Other sensor publisher class should inherit from this class.