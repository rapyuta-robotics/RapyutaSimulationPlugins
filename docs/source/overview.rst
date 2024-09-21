Overview: Robot and GameMode
=============================

.. image:: images/robot_and_gamemode.png

Above figure shows typical way of creating Robot and GameMode with RapyutaSimulationPlugins and how expose ROS 2 interfaces.
Please check each components overview in :doc:`components` and :doc:`ue_api`.


Robot
-----------------

Robot Actor is composed of
`ARRBaseRobot <doxygen_generated/html/df/d13/class_a_r_r_base_robot.html>`_,
`URRRobotROS2Interface <doxygen_generated/html/d6/d64/class_u_r_r_robot_r_o_s2_interface.html>`_ and
`ARRBaseRobotROSController <doxygen_generated/html/d1/d45/class_a_r_r_base_robot_r_o_s_controller.html>`_ .
You can create your own robot by creating child class of following components.
Robot can be placed from UE interface, e.g drag and drop from UE Editor, or spawned from ROS 2 /SpawnEntity srv.

Features
^^^^^^^^^^^^^^
- ROS 2 interface
    ARRBaseRobot is designed to be controlled from ROS 2 via URRRobotROS2Interface.
    
    ARRBaseRobot has only basic interface such as `/cmd_vel`, `/odom`, `/joint_states` , `/joint_commands`.
    Please create child class and add necessary ROS 2 APIs.

- Spawning from ROS 2 with parameters
    ARRBaseRobot and URRRobotROS2Interface can be spawnd from `/SpawnEntity <https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnEntity.srv>`_ 
    service. You can pass Robot Name, Namespace and etc.

    SpawnEntity.srv also has a field named `json_parameters` to pass random parameters.
    Please add your own custom parser in child class by overwriting 
    `InitPropertiesFromJSON() <doxygen_generated/html/df/d13/class_a_r_r_base_robot.html#a214c5936450e3b17dffaad40e944bea6>`_ 
    which is called in 
    the `PreInitializeComponents() <https://docs.unrealengine.com/5.1/en-US/unreal-engine-actor-lifecycle/>`_
    
- Distributed simulation
    ARRBaseRobot has a setting to be used for client-server. Please check :doc:`distributed_simulation` for detailed information.



Components
^^^^^^^^^^
- `URRRobotROS2Interface <doxygen_generated/html/d6/d64/class_u_r_r_robot_r_o_s2_interface.html>`_ 
    Controls ROS 2 node, publisher/subscriber, service client/server and action via 
    `UROS2NodeComponent <https://rclue.readthedocs.io/en/latest/doxygen_generated/html/d7/d68/class_u_r_o_s2_node_component.html>`_ 

    ROS2Interface is created in the PreInitializeComponents if ROS2InterfaceClass is specified and initialized by ROS2Controller.

    ROS2Interface can be override from Bluepritn as well.
   
- `ARRBaseRobot <doxygen_generated/html/df/d13/class_a_r_r_base_robot.html>`_
    Base Robot class which has basic ROS 2 topic interfaces such as Twist msg, JointState and etc via ROS2Interface.
    
    - `URRROS2BaseSensorComponent <doxygen_generated/html/d0/d58/class_u_r_r_r_o_s2_base_sensor_component.html>`_  
        Base class of ROS 2 sensors. Other ROS 2 sensor should be child class of this class, e.g. lidar, camera, etc. 

        URRROS2BaseSensorComponent has `URRROS2BaseSensorPublisher <doxygen_generated/html/d5/d69/class_u_r_r_r_o_s2_base_sensor_publisher.html>`_ 
        which publish Sensor topic and is initialized from URRROS2Interface.

    - `URobotVehicleMovementComponent <doxygen_generated/html/d7/d01/class_u_robot_vehicle_movement_component.html>`_ 
        Base class of Robot movement, which is controlled by 
        `ROS 2 Twist Msg <https://docs.ros2.org/galactic/api/geometry_msgs/msg/Twist.html>`_ 
        via `URRRobotROS2Interface <doxygen_generated/html/d6/d64/class_u_r_r_robot_r_o_s2_interface.html>`_ . 
        
        Other movement class should be child class of this class, e.g. differential drive, ackermann drive, etc.

    - `URRJointComponent <doxygen_generated/html/de/dfa/class_u_r_r_joint_component.html>`_ 
        Representative of joint of the robot, which is controlled by 
        `ROS 2 JointState Msg <https://docs.ros2.org/galactic/api/sensor_msgs/msg/JointState.html>`_
        via `URRRobotROS2Interface <doxygen_generated/html/d6/d64/class_u_r_r_robot_r_o_s2_interface.html>`_ . 


- `ARRBaseRobotROSController <doxygen_generated/html/d1/d45/class_a_r_r_base_robot_r_o_s_controller.html>`_ 
    Actor controller class which has authority to start/stop ROS 2 Interfaces.
    Following the Pawn and AIController structure in Unreal Engine, 
    `Ref <https://docs.unrealengine.com/4.27/en-US/InteractiveExperiences/Framework/Pawn/>`_.


Examples and more
^^^^^^^^^^^^^^^^^
.. toctree::
    :maxdepth: 1
    
    robots/mobile_robot
    robots/robot_arm

Todo
^^^^^^^^^^^^^^^^^
- Support spawning robot from URDF
- Non Robot actor with ROS 2 Interfaces

GameMode
-----------------

The GameMode start all the simulation components, mainly the Simulation State and its associated ROS 2 Node.

- `ASimulationState <doxygen_generated/html/d2/dde/class_a_simulation_state.html>`_ 
    
    Simulation component, which has implementation of ROS 2 services and communicate with URRROS2SimulationstateClient.
    
    ASimulationState can not control all actors, but it only manages actors in 
    `Entities <doxygen_generated/html/d2/dde/class_a_simulation_state.html#a814e73a90bfbe5e01973e7b04552dfa1>`_  
    . Actors in the level at BeginPlay and actors spawned by ROS 2 services are added automatically.
    You need to manually call `ServerAddEntity() <doxygen_generated/html/d2/dde/class_a_simulation_state.html#a26555da061d66a6b31f5c42053708119>`_ 
    if you want to control actors that are not spawned from ASimulationState at runtime.
  
    ASimulationState can not spawn all UE actor classes, but only  
    `SpawnableEntityTypes <doxygen_generated/html/d2/dde/class_a_simulation_state.html#a236405852be150d955a4736a323cb514>`_ 
    If you want to control actors that are not spawned from ASimulationState at runtime, 
    you need to manually call `AddSpawnableEntityTypes() <doxygen_generated/html/d2/dde/class_a_simulation_state.html#a97b63ff1f474c8323b489abc4cfd4504>`_ 
    You can also change spawnable entities in the level Blueprint, by overwriting the game mode, and so on.

- `URRROS2SimulationstateClient <doxygen_generated/html/d7/d6a/class_u_r_r_r_o_s2_simulation_state_client.html>`_ 
    
    Simulation component, which provides ROS 2 services interface to manipulate actors.
    ASimulationState and URRROS2SimulationstateClient are separated to be used in client-server.

- `URRROS2ClockPublisher <doxygen_generated/html/d5/dc2/class_u_r_r_r_o_s2_clock_publisher.html>`_ 
  
    ROS 2 publisher class publishes `/clock` topic at every tick in the simulation. 
  
    In the example `turtlebot3-UE <https://github.com/rapyuta-robotics/turtlebot3-UE>`_ project,
    It is set to use
    `URRLimitRTFFixedSizeCustomTimeStep <doxygen_generated/html/d0/d4d/class_u_r_r_limit_r_t_f_fixed_size_custom_time_step.html>`_
    which enables running simulation with fixed timestep while limiting the RTF(Real Time Factor).

    The turtlebot3-UE project is set to update `DefaultEngine.ini`` with environment variable from 
    `DefaultEngineBase.ini <https://github.com/rapyuta-robotics/turtlebot3-UE/blob/devel/Config/DefaultEngineBase.ini>`_
    . The frame rate and max RTF are set from environment variable `FIXED_FRAME_RATE` and `TARGET_RTF`.

.. list-table:: SimulationState ROS 2 Service
   :header-rows: 1

   * - Service name
     - Service type
     - About
   * - /GetEntityState
     - `GetEntityState.srv <https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/GetEntityState.srv>`_
     - return the actor actor state
   * - /SetEntityState
     - `SetEntityState.srv <https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SetEntityState.srv>`_
     - et the actor state.
   * - /GetEntityState
     - `GetEntityState.srv <https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/GetEntityState.srv>`_
     - return the actor state
   * - /SpawnEntity
     - `SpawnEntity.srv <https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnEntity.srv>`_
     - spawn actor
   * - /SpawnEntities
     - `SpawnEntities.srv <https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnEntities.srv>`_
     - spawn many actors from a list
   * - /DeleteEntity
     - `DeleteEntity.srv <https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/DeleteEntity.srv>`_
     - return the actor actor state
   * - /Attach
     - `Attach.srv <https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/Attach.srv>`_
     - attach two actors which are not connected or detach two actors which are connected.

Todo
^^^^^^^^^^^^^^^^^
- SpawnWorld service