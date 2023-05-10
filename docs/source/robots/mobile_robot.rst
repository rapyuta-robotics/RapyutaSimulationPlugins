Mobile Robots
=============

Example Robots
--------------

.. contents:: Index
   :depth: 4

Turtlebot3 
^^^^^^^^^^^^^^

There are 4 turtlebot actors, which have same ROS 2 Interfaces.

.. list-table:: ROS 2 Interface
   :header-rows: 1

   * - Topic name
     - Topic type
     - About
   * - cmd_vel
     - `geometry_msgs/msg/Twist Msg <https://docs.ros2.org/galactic/api/geometry_msgs/msg/Twist.html>`_
     - Robot velocity reference
   * - odom
     - `nav_msgs/msgs/Odometry Msg <https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html>`_
     - Odometry from robot pose
   * - scan
     - `sesor_msgs/msg/LaserScan Msg <https://docs.ros2.org/galactic/api/sensor_msgs/msg/LaserScan.html>`_
     - Laser scan msg


Turtlebot3 Burger
""""""""""""""""""

Turtlebot3 Burger Actors are examples of robots implemented in C++. 
All components, joints, etc are defined in C++.

Even if it is implemented in C++, we defined Blueprint child class to change parameters and assign meshes.

\* Since all components are defined in C++, it is marked `Edit in C++` in left components bar.
\* Hardcode mesh path in C++ cause error when the project is packaged.

- `ATurtlebotBurgerVehicle <../doxygen_generated/html/de/d76/class_a_turtlebot_burger.html>`_
    Example kinematic robot implementation in C++. 
    
    This is child class of ARRBaseRobot and is added 
    `SkeletalMesh <https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/Components/USkeletalMeshComponent/>`_ 
    and `URR2DLidarComponent  <../doxygen_generated/html/d4/d87/class_u_r_r2_d_lidar_component.html>`_.

    - `BP_TurtlebotBurgerVehicle <https://github.com/rapyuta-robotics/RapyutaSimulationPlugins/blob/devel/Content/Robots/Turtlebot3/Kinematics/BP_TurtlebotBurgerVehicle.uasset>`_ : BP child class of ATurtlebotBurgerVehicle
        
    .. image:: ../images/turtlebot3_burger_vehicle.png

- `ATurtlebotBurger <../doxygen_generated/html/de/d76/class_a_turtlebot_burger.html>`_
    Example physics robot implementation in C++. 

    This is child class of ARRBaseRobot and is added 
    `UStaticMeshComponent <https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/Components/UStaticMeshComponent/>`_, 
    `UPhysicsConstraintComponent <https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/>`_, 
    `UDifferentialDriveComponent <../doxygen_generated/html/db/df5/class_u_differential_drive_component.html>`_, 
    and `URR2DLidarComponent  <../doxygen_generated/html/d4/d87/class_u_r_r2_d_lidar_component.html>`_.

    - `BP_TurtlebotBurger <https://github.com/rapyuta-robotics/RapyutaSimulationPlugins/blob/devel/Content/Robots/Turtlebot3/Physics/BP_TurtlebotBurger.uasset>`_: BP child class of ATurtlebotBurger
            
    .. image:: ../images/turtlebot3_burger.png

- `URRTurtlebotROS2Interface <../doxygen_generated/html/d6/d7d/class_u_r_r_turtlebot_r_o_s2_interface.html>`_
    Example child class of URRROS2Interfaces.


Turtlebot3 Waffle
""""""""""""""""""

Turtlebot3 Waffle Actors are examples of robots implemented in BP.
Both are child class of ARRBaseRobot and all additional components, joints, etc are defined in BP.

- `BP_TurtlebotWaffleVehicle <https://github.com/rapyuta-robotics/RapyutaSimulationPlugins/blob/devel/Content/Robots/Turtlebot3/Kinematics/BP_TurtlebotWaffleVehicle.uasset>`_
    Example kinematic robot implementation in BP. 
    
    This is child class of ARRBaseRobot and is added 
    `SkeletalMesh <https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/Components/USkeletalMeshComponent/>`_ 
    and `URR2DLidarComponent  <../doxygen_generated/html/d4/d87/class_u_r_r2_d_lidar_component.html>`_
    similar as ATurtlebotBurgerVehicle in C++.

    .. image:: ../images/turtlebot3_waffle_vehicle.png

- `BP_TurtlebotWaffle <https://github.com/rapyuta-robotics/RapyutaSimulationPlugins/blob/devel/Content/Robots/Turtlebot3/Physics/BP_TurtlebotWaffle.uasset>`_
    Example physics robot implementation in BP. 

    This is child class of ARRBaseRobot and is added 
    `UStaticMeshComponent <https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/Components/UStaticMeshComponent/>`_, 
    `UPhysicsConstraintComponent <https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/>`_, 
    `UDifferentialDriveComponent <../doxygen_generated/html/db/df5/class_u_differential_drive_component.html>`_, 
    and `URR2DLidarComponent  <../doxygen_generated/html/d4/d87/class_u_r_r2_d_lidar_component.html>`_ similar as ATurtlebotBurger in C++.

    .. image:: ../images/turtlebot3_waffle.png


    To configure MoveComponent in BP, 
    `VehicleMoveComponentClass <../doxygen_generated/html/df/d13/class_a_r_r_base_robot.html#aa69278b89215d02dd07da74b6feb83f3>`_
    is set as None, then, VehicleMoveComponent is not configured in C++ function.
    Instead of configuration in C++, MoveComponent are set in 
    `Construction script in BP <https://docs.unrealengine.com/5.1/en-US/construction-script-in-unreal-engine/>`_

    .. image:: ../images/turtlebot3_waffle_drive_comp.png

- `BP_TurtlebotROS2Interface <https://github.com/rapyuta-robotics/RapyutaSimulationPlugins/blob/devel/Content/Robots/Turtlebot3/BP_TurtlebotROS2Interface.uasset>`_
    Example child class of URRROS2Interfaces.



Custom Robot Creation TIPS
--------------------------

To create custom mobile robot

1. Create child class of ARRBaseRobot
    1. Overwrite default parameters.
    2. Configure meshes and physics constraints in BP.
    3. If you want to dynamically spawn robots and pass random parameters, overwrite `InitPropertiesFromJSON() <../doxygen_generated/html/df/d13/class_a_r_r_base_robot.html#a214c5936450e3b17dffaad40e944bea6>`_ 
2. Create child class of URobotVehicleMovementComponent if you want custom movement behaviour.
    1. Configure movecomponent in your Robot Class similar as BP_TurtlebotWaffle.
3. Create child class of URRROS2Interfaces
    1. Overwrite default parameters such as topic name.
    2. Add necessary ROS Interfaces. Please also refer `rclUE tutorials <https://rclue.readthedocs.io/en/latest/examples.html#topic-service-action-examples>`_.
4. Create ROS 2 Service client of  `/SpawnEntity <https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnEntity.srv>`_ and pass necessary parameters outside of UE if you want to dynamically spawn robots from outside of UE

