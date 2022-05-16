Overview
=========

.. image:: images/robot_and_gamemode.png

Above figure shows typical way of creating Robot and GameMode with RapyutaSimulationPlugins and how expose ROS2 interfaces.
Please check each components overview in :doc:`components` and :doc:`api`.

\* This provide typical way of creating robot. You should be able to create Actor which has ROS interfaces with arbitrary architecture. 

`ARRRobotVehicleROSController <doxygen_generated/html/d6/d83/class_a_r_r_robot_vehicle_r_o_s_controller.html>`_ has `AROS2Node <https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dcb/class_a_r_o_s2_node.html>`_ and controls publisher and subscribers in the Robot. 
URRRobotVehicleROSController subscribes command topic such as /cmd_vel and move robot by setting velocity to `URobotVehicleMovementComponent <doxygen_generated/html/d7/d01/class_u_robot_vehicle_movement_component.html>`_ in `ARobotVehicle <doxygen_generated/html/d7/d80/class_a_robot_vehicle.html>`_ 
ARobotVehicle has `URRROS2BaseSensorComponent <doxygen_generated/html/d0/d58/class_u_r_r_r_o_s2_base_sensor_component.html>`_ which has `URRROS2BaseSensorPublisher <doxygen_generated/html/d5/d69/class_u_r_r_r_o_s2_base_sensor_publisher.html>`_.

\* Following the Pawn and AIController structure in Unreal Engine, `Ref <https://docs.unrealengine.com/4.27/en-US/InteractiveExperiences/Framework/Pawn/>`_.

`ARRGameMode <doxygen_generated/html/d1/dbb/class_a_r_r_game_mode.html>`_ provides ROS2 service interface to manipulate actors and `URRROS2ClockPublisher <doxygen_generated/html/d5/dc2/class_u_r_r_r_o_s2_clock_publisher.html>`_ to publish /clock.
`ASimulationState <doxygen_generated/html/d2/dde/class_a_simulation_state.html>`_ has Entities and SpawnableEntities as member variable and only manipulate actors in those. 

\* You need to use specific method to add actors to the variables if you want to manipulate from ROS2 services.
Actors in the level at beginplay and spawned actor by ROS2 services are added automatically.
You can change spawnable entities in level BP, by overwriting game mode and etc.


.. list-table:: SimulationState ROS2 Service 
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
     - `Attach.srv <https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/Attach.srv>`_
     - return the actor actor state
   * - /SpawnEntity
     - `SpawnEntity.srv <https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnEntity.srv>`_
     - spawn actor
   * - /SpawnWorld
     - `SpawnWorld.srv <https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnWorld.srv>`_
     - spawn world
   * - /DeleteEntity
     - `DeleteEntity.srv <https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/DeleteEntity.srv>`_
     - return the actor actor state
   * - /Attach
     - `Attach.srv <https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/Attach.srv>`_
     - attach two actors which are not connected or detach two actors which are connected.
