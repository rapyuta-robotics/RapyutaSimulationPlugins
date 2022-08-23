Components and main C++ classes
===============================
Overview and main C++ classes for components. 
Please check detail explanation of those class and other classes in `C++ Documentation <doxygen_generated/html/index.html>`_

Drives
------

| Drives provides feature to control robot. 
| Custom robot movment class should be created as child class of *RobotVehicleMovementComponent*
| Custom joint class should be created as child class of *RRJointComponent*

**example of class relations**

.. image:: doxygen_generated/html/d9/d8d/class_u_differential_drive_component__coll__graph.png

**main C++ classes**

- `URobotVehicleMovementComponent <doxygen_generated/html/d7/d01/class_u_robot_vehicle_movement_component.html>`_:  Base Robot vehicle movement class which is used as part of #ARobotVehicle. Move robot without considering physics and publish odometry. This supports adapting the robot pose to floor complex surfaces, and following moving platforms such as elevators.
- `UDifferentialDriveComponent <doxygen_generated/html/db/df5/class_u_differential_drive_component.html>`_: Simulate differential drive with 2 wheels considering physics.
- `URRJointComponent <doxygen_generated/html/de/dfa/class_u_r_r_joint_component.html>`_: Base Joint class which is used as part of #ARobotVehicle.
- `URRKinematicJointComponent <doxygen_generated/html/d2/d69/class_u_r_r_kinematic_joint_component.html>`_: Kinematics Joint Component which has pose and velocity control interface.


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

| Robots has robot vehicle class, ROS2 interface class and controller class.
| Custom robot class should be created by creating child classes of those classes.
| RobotVehicle has movement component, joints, and sensors.
| RobotROS2Interface has ROS2 subscribers and publsihers and control RobotVehicle from ROS2Interface.
| Robot controller controls start/stop ROS2Node. 

**main C++ classes**

- `ARobotBaseVehicle <doxygen_generated/html/df/dbc/class_a_robot_base_vehicle.html>`_: Base RobotBaseVehicle class. Most robot classes should inherit from this class. Has URobotVehicleMovementComponent and initializes sensors.
- `ARobotVehicle <doxygen_generated/html/d7/d80/class_a_robot_vehicle.html>`_: RobotVehicle class that inherits from RobotBaseVehicle and uses a Skeletal Mesh as root component. Other robot class can inherit from this class.
- `URRRobotROS2Interface <>`_  Class provides ROS2Interfaces.  This class has `AROS2Node <https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dcb/class_a_r_o_s2_node.html>`_ and provide ROS2 interfaces to control robot such as Twist msg.
- `ARRRobotVehicleROSController <doxygen_generated/html/d6/d83/class_a_r_r_robot_vehicle_r_o_s_controller.html>`_: Base Robot ROS controller class. Other robot controller class should inherit from this class. 
- `ATurtlebotBurger <doxygen_generated/html/de/d76/class_a_turtlebot_burger.html>`_: Example of ARobotVehicle
- `URRTurtlebotROS2Interface <>`_: Example of URRRobotROS2Interface

**main Blueprint classes classes**

- Turtlebot3: example of mobile robot.
    - Models: Static and skeletal meshes and materials.
    - Physics: physics model with UDifferentialDriveComponent
        - BP_TurtlebotBurger: inherit from C++ ATurtlebotBurger and set static meshes and parameters.
        - BP_TurtlebotWaffle: inherit from C++ ARobotVehicle. Example of construct robot from Blueprint. 
    - Kinematics: kinematic model with URobotVehicleMovementComponent 
        - BP_TurtlebotBurgerVehicle: inherit from C++ ARobotVehicle and set static meshes and parameters.
        - BP_TurtlebotWaffleVehicle: inherit from C++ ARobotVehicle. Example of construct robot with Blueprint. 
- Skeletal Turtlebot3: Physics-enabled skeletal mesh component-based turtlebot3 robots of types: 
    - BallCasterSphereWheeled : With ball caster sphere wheel
    - ConvexWheeled : Wheels have convex-hull collision
    - SphereWheeled : Wheels have sphere collision
    - StaticMeshConstrained : Built from separate static mesh components connected to one another by physics constraints
    - FullLockConstrained : All physics constraints are locked
    - WheeledVehicle : Utilize SimpleWheeledVehicleMovement
    - SkeletalTurtlebot3Examples level: Have all example skeletal robots being put to automatically move forward upon Play
- SampleArm: example of robot arm with joint.
    - BP_KinematicSimpleArm: inherit from C++ ARobotVehicle and have joint and link setting. Please check construction script for joints settings. You can control arm by 
    .. code-block:: bash

       $ ros2 topic pub /arm/joint_states sensor_msgs/msg/JointState  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, name: ['joint12', 'joint23', 'joint34'], position: [1.57,1.57,1.57], velocity: [], effort: []}"
    
    - BP_SampleArmROSController: inherit from C++ ARRRobotVehicleROSController. Subscribes 'joint_state' topic and control robot joints. 

Core
----

| Core has base classes which are directly or indirectly used by other components in RapyutaSimulationPlugins Plugins.
| Core also has util classes as well. 

**main C++ classes(todo)**

- `ARRBaseActor <doxygen_generated/html/d9/d3d/class_a_r_r_base_actor.html>`_: Base actor class for all Rapyuta Sim actors
- `URRActorCommon <doxygen_generated/html/df/d29/class_u_r_r_actor_common.html>`_: todo
- Game objects
    - `ARRROS2GameMode <>`_: Basic GameMode which provides basic ROS2 interfaces such as clock publisher and spawn services.
    - `ARRGameMode <doxygen_generated/html/d1/dbb/class_a_r_r_game_mode.html>`_: GameMode with specific setting, asset loading.
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

- `URRROS2SimulationstateClient <>`_: Provide spawn/delete/attach/set/get ROS2 interfaces. Typically this class is initialized from GameMode.
- `ASimulationState <doxygen_generated/html/d2/dde/class_a_simulation_state.html>`_: Has implementation of spawn/delete/attach/set/get ROS2 interfaces. Communicate with URRROS2SimulationstateClient to execute manipulation.
- `URRROS2ClockPublisher <doxygen_generated/html/d5/dc2/class_u_r_r_r_o_s2_clock_publisher.html>`_: Publish /clock topic. Typically this class is initialized from GameMode.
- `OccupancyMapGenerator <doxygen_generated/html/d2/dde/class_a_occupancy_map_generator.html>`_: Generate 2D occupancy map for navigation/localization.
- `URRROS2BaseSensorPublisher <doxygen_generated/html/d5/d69/class_u_r_r_r_o_s2_base_sensor_publisher.html>`_: Base Sensor Publisher class. Other sensor publisher class should inherit from this class.
