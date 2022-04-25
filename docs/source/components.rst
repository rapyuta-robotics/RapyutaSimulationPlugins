Components and main C++ classes
===============================

Core
----

| Core has base classes which are directly or indirectly used by other components in RapyutaSimulationPlugins Plugins.
| Core also has util classes as well. 

**main C++ classes**

- RRActorCommon
- RRBaseActor
- RRROS2GameMode

Robots
------

| Robots has robot vehicle class and controller class.
| Custom robot class should be created as child class of *RobotVehicle* class. 
| *RRRobotVehicleROSController* create ROS2Node and control all drive and sensor components in the *RobotVehicle* class.

**main C++ classes**

- RobotVehicle
- RRRobotVehicleROSController

Drives
------

| Drives provides feature to control robot from ROS2Node. 
| Custom drive class should be created as child class of *RobotVehicleMovementComponent*

**main C++ classes**

- RobotVehicleMovementComponent
- DifferentialDriveComponent

Sensors
-------

| Sensor classes which can be used as stand alone actor or component of Robot.
| *<sensor name>Component* class is designed to used as part of robot.
| *<sensor name>Actor* class is designed to used as stand alone actor and ROS node.

**main C++ classes**

- RRROS2BaseSensorComponent
- RRROS2CameraComponent
- RR2DLidarComponent
- RR3DLidarComponent

Tools
-----

| Offline and runtime tools. 
| SimulationState provide basic ROS2 service interface to interact with UE4 from ROS2 such as Spawn Robot.

**main C++ classes**

- SimulationState
- ROS2Spawnable
- OccupancyMapGenerator
