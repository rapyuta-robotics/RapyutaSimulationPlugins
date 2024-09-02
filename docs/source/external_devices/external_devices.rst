External Interface Requirements
===============================

Software Interfaces
--------------------

Base Class
~~~~~~~~~~

Overview
++++++++

External Devices are implemented by BP to be easily edited by non-engineers as well. 
External Device is a child class `BP_ExternalDeviceBase`, which is a child class of `RRBaseRobot` to have `ROS2Node` and `ROS2Interface`.

Base Functions
++++++++++++++

- **ParamParser** and **ParamParserImple**: Parse JSON parameters. Additional ROS spawn parameter parsing should be implemented here.
  - When it is spawned from ROS, ParamParser is called from `BPInitParamPaser`.
  - When it is spawned from UE, e.g., placed from editor, ParamParser is called from `Initialize` function. It is expected that parameters are overwritten by the editor as normal UE Actors.
- **InitializeChildActor**: Some external devices have external devices as child actors, e.g., vertical conveyor has elevator and conveyors as child actors. Initialization of child actors such as passing parameters, setting relative pose, etc., is done here.
- **Construction Script** calls:
  1. `InitializeChildActor`
  2. `Initialize`
  3. `PostInitialize`

Parameters
++++++++++

.. list-table::
   :header-rows: 1

   * - Parameter Name
     - Type = Default
     - Note
   * - /debug
     - bool = false
     - Mainly used to print debug log or not
   * - /mode
     - int = 0
     - Most external devices have modes, e.g., manual or auto
   * - /disable_physics
     - bool = true
     - Disable physics of target object during operation, e.g., conveyor disables physics of payload during movement
   * - /size
     - dict = {x:1, y:1, z:1}
     - Scale of external device. Not all devices are confirmed with arbitrary sizes

Other BP Exposed Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Parameter Name
     - Type = Default
     - Note
   * - ParseParamFromJSON
     - bool = true
     - Parse parameter from JSON or not
   * - DebugParamParser
     - bool = false
     - Use TestJsonInput below as JSON parameter input. Mainly used for debugging JSON Param Parser
   * - TestJsonInput
     - string = ''
     - Test JSON input for debugging

Conveyor
~~~~~~~~

Overview
++++++++

The conveyor has collision meshes with the `OverlapAll` setting. The conveyor moves objects with a given velocity that overlaps with the collision meshes.

There are `BP_Conveyor` and `BP_SplineConveyor` classes, which are child classes of `BP_ConveyorBase` class. The main functionality is implemented in `BP_ConveyorCollisionMeshAddon`.

`BP_Conveyor` is a simple straight conveyor, and `BP_SplineConveyor` is a conveyor along a spline curve.

Conveyors can be controlled by velocity input and come with two sensors that detect objects at edges.

Parameters
++++++++++

.. list-table::
   :header-rows: 1

   * - Parameter Name
     - Type = Default
     - Note
   * - /mode
     - int32 = 0
     - 0: Move payload until it goes out of the area
       1: Move payload until it hits the entrance sensor
   * - /sensor1_transform
     - transform = { position: {x:97.5, y:0, z:0}, rotation: {x:0, y:0, z:0}, size: {x:0.05, y:1, z:0.5} }
     - Relative transform of sensor1
   * - /sensor2_transform
     - transform = { position: {x:-97.5, y:0, z:0}, rotation: {x:0, y:0, z:0}, size: {x:0.05, y:1, z:0.5} }
     - Relative transform of sensor2
   * - /vel
     - float = 1 [m/s]
     - Conveyor velocity
   * - /sensor_length
     - float = 0.1 [m]
     - Length of sensor area
   * - /points
     - dict = { position: None, rotation: None, arrive_tangent: None, leave_tangent: None, scale: None }
     - Spline points of curves. Conveyor meshes are created along the spline curve.

ROS 2 API
~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Topic Name
     - Message Type
     - Note
   * - /set_vel
     - `example_interfaces/msg/Float32`
     - [m/s] Move conveyor. Can be + or -
   * - /set_mode
     - `example_interfaces/msg/Int32`
     - 0: Move payload until it goes out of the area
       1: Move payload until it hits the entrance sensor
   * - /entrance
     - `example_interfaces/msg/Int32MultiArray`
     - Size=2. 0 means no object at sensor location, 1 means something is there.

*Note: Topic names are temporary and can be changed from parameters.*

Elevator
~~~~~~~~

Overview
++++++++

Elevators have a container that moves between floors. The container can have doors or not. The container has a conveyor inside to automatically pull in/push out the payload.

*Todo:*

- Support multiple types of doors
- Multiple door locations for each floor
- Test Spawning from ROS 2

ROS 2 API
~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Parameter Name
     - Type = Default
     - Note
   * - /mode
     - int8 = 1
     - 0: Manual. Able to control the elevator and door by velocity input.
       1: Normal. Control the elevator and door by `/move_to` and `/open_door`.
       2: Auto Move Return. The elevator moves to another floor specified in `/auto_target_floors` when the payload enters and returns to the original floor when objects go outside the elevator.
   * - /floor_height
     - float = 5 [m]
     - Distance between floors
   * - /floors
     - int8[2] = [0,1]
     - Top and bottom floor numbers
   * - /doors
     - bool[true, false, false]
     - Which door opens: front or back on each floor. 0: front, 1: back
   * - /door
     - bool = true
     - Spawn doors or not
   * - /door_vel
     - float = 0.3 [m/s]
     - Door moving speed
   * - /initial_floor
     - int32 = 0
     - Initial floor
   * - /auto_target_floors
     - int8[2] = [0,1]
     - Target floors for auto mode
   * - /vel
     - float = 3 [m/s]
     - Elevator moving speed

.. list-table::
   :header-rows: 1

   * - Topic Name
     - Message Type
     - Note
   * - /set_vel
     - `example_interfaces/msg/Float32`
     - [m/s] Mode==0: Move the container. Can be + or -. Mode > 0: Set velocity used for `/move_to` and auto movement.
   * - /set_front_door_vel
     - `example_interfaces/msg/Float32`
     - [m/s] Mode==0: Move the door. Can be + (open), - (close). Mode > 0: Set velocity used for `/move_to` and auto movement.
   * - /set_back_door_vel
     - `example_interfaces/msg/Float32`
     - [m/s] Mode==0: Move the door. Can be + (open), - (close). Mode > 0: Set velocity used for `/move_to` and auto movement.
   * - /open_door
     - `example_interfaces/msg/Bool`
     - Open/Close the door.
   * - /move_to
     - `example_interfaces/msg/Int32`
     - Close door, move to the given floor, and open the door.
   * - /set_mode
     - `example_interfaces/msg/Int32`
     - 0: Manual. Able to control the elevator and door by velocity input. 1: Normal. Control the elevator and door by `/move_to` and `/open_door`.

*Note: Topic names are temporary and can be changed from parameters.*

.. note::
   - [ROS 2 Documentation for Int32](https://docs.ros2.org/foxy/api/example_interfaces/msg/Int32.html)
   - [ROS 2 Documentation for Float32](https://docs.ros2.org/foxy/api/example_interfaces/msg/Float32.html)
   - [ROS 2 Documentation for Bool](https://docs.ros2.org/foxy/api/example_interfaces/msg/Bool.html)

Vertical Conveyor
~~~~~~~~~~~~~~~~~

Overview
++++++++

Vertical Conveyor is a combination of conveyors (entrances) and an elevator.

*Todo:*

- Different parameter settings for each entrance
- Test spawning from ROS 2

ROS 2 API
~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Parameter Name
     - Type = Default
     - Note
   * - /floor_height
     - float = 10 [m]
     - Distance between floors
   * - /entrances
     - int32[4] = [false, true, false, true]
     - 0 means no entrance, 1 means entrance. Defines whether each entrance is active or not.
   * - /target_entrance
     - int8[2] = [0,1]
     - Target entrance, [in, out].
   * - /elevator
     - string = ''
     - JSON spawn parameter passed to the child elevator actor.
   * - /entrance
     - string = ''
     - JSON spawn parameter passed to the child entrance conveyor actors.

BP Parameters
~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Parameter Name
     - Type = Default
     - Note
   * - EntranceActorClass
     - Actor Class = BP_Conveyor
     - Entrance Conveyor Class

Vertical Conveyor ROS 2 API
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Topic Name
     - Message Type
     - Note
   * - /set_target_entrance
     - int32[2]
     - Target entrance. [in, out].

*Note: Topic names are temporary and can be changed from parameters.*
