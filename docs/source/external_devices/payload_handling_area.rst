Payload Handling Area
=====================

Payload Handling Areas are zones where actors are spawned, deleted, teleported, or altered. These areas are mainly used for:

- Mimicking non-interested payload movements by spawning, deleting, and teleporting payloads in simulation.
- Mimicking industrial machines by changing actors to another type in the simulation.

Payload Handler Base Class
--------------------------

The `BP_PayloadHandlerBase` class serves as the base class for all payload handling area classes. It is a child class of `BP_ExternalDeviceBase`.

Basic Functions
^^^^^^^^^^^^^^^^

- `/trigger` action is overwritten by child classes.

- **Manual and Auto Mode**:

  - *Manual*: The `/trigger` action is invoked manually by Unreal Engine (UE) or ROS 2.
  - *Auto*: The `/trigger` action is invoked periodically after a certain duration once a specific condition is met.

- **Target Actor Filtering**: The target actor is filtered by `TargetTag`.

Payload Handler Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1

   * - Param Name

        [UE name if it is not pascal case of ROS one]
     - Type (Default)
     - Note
   * -  **ROS JSON SPAWN PARAMETER**
     -
     -
   * - /mode
     - int32 (0)
     - 0: Manual
       1: Auto
   * - /duration
     - float (1.0)
     - Time interval to trigger action in auto mode.
   * - /tag

        [TargetTag]
     - string ('Payload')
     - Filters target actors by this tag.

ROS 2 API for Payload Handler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1

   * - Topic Name
     - Msg Type
     - Note
   * -  **SUBSCRIBE**
     -
     -
   * - /set_mode
     - `example_interfaces/msg/Int32 <https://docs.ros2.org/foxy/api/example_interfaces/msg/Int32.html>`_
     - Sets the mode: 0 for manual, 1 for auto.
   * - /set_duration
     - `example_interfaces/msg/Float32 <https://docs.ros2.org/foxy/api/example_interfaces/msg/Float32.html>`_
     - Sets the duration for auto-triggering actions.
   * - /trigger
     - `example_interfaces/msg/Empty <https://docs.ros2.org/foxy/api/example_interfaces/msg/Empty.html>`_
     - Manually triggers an action.

Sink Area
---------

`BP_SinkArea` is a child class of `BP_PayloadHandlerBase`.

- `/trigger` action
    deletes the actor.

- **Condition for Auto Mode**:
    The action is triggered when an actor overlaps with the area.


.. video:: ../_static/videos/sink_area.mp4
    :width: 750
    :height: 450

*Video: Sink Area*


Source Area
-----------

`BP_SourceArea` is a child class of `BP_PayloadHandlerBase`. The source area contains matrix spots where actors are spawned and attached to the area.

- `/trigger` action:
   spawns and attaches an actor to the spots.

- **Condition for Auto Mode**:
    The action is triggered when the actor in the spot is detached from the area.

Depending on the `/source_mode`, the `/trigger` action will either spawn one actor at a time or all actors when all spots are empty.

.. video:: ../_static/videos/source_area.mp4
    :width: 750
    :height: 450

*Video: Source Area*

Parameters for Source Area
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. list-table::
   :header-rows: 1

   * - Param Name
     - Type (Default)
     - Note
   * -  **ROS JSON SPAWN PARAMETER**
     -
     -
   * - /source_mode
     - int32 (0)
     -
        1. Spawns one actor at a time.
        2. Spawns all actors together when all spots are empty.
   * - /num
     - vector (x:1, y:1, z:1)
     - Dimensions of the source spots.
   * - /clearance
     - vector (x:1, y:1, z:1)
     - Clearance between spots.
   * - /spawn_actor_classes
     - string[]
     - List of actor classes spawned by the area.

ROS 2 API for Source Area
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1

   * - Topic Name
     - Msg Type
     - Note
   * -  **SUBSCRIBE**
     -
     -
   * - /set_spawn_mode
     - `example_interfaces/msg/Int32 <https://docs.ros2.org/foxy/api/example_interfaces/msg/Int32.html>`_
     - Sets the spawn mode:

       0. Spawns one actor at a time,
       1. Spawns all actors together.

Teleport Area and Manager
-------------------------

`BP_TeleportArea` is a child class of `BP_SinkArea`, and `BP_TeleportAreaManager` is a child class of `BP_PayloadHandlerBase`. `BP_TeleportAreaManager` holds a list of `BP_TeleportArea` instances, and manually triggering its action will trigger all teleport areas in the list.

.. video:: ../_static/videos/teleport_area.mp4
    :width: 750
    :height: 450

*Video: Teleport Area*

Teleport Area Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1

   * - Param Name
     - Type (Default)
     - Note
   * -  **ROS JSON SPAWN PARAMETER**
     -
     -
   * - /teleport_mode
     - int32 (0)
     - 0: Entrance
       1: Exit
   * - /target_exit
     - string ('')
     - Specifies the target exit, which must be the actor name of another TeleportArea instance.
   * - /num
     - vector (x:1, y:1, z:1)
     - Number of teleportation spots.
   * - /clearance
     - vector (x:1, y:1, z:1)
     - Clearance between teleport spots.

ROS 2 API for Teleport Area
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1

   * - Topic Name
     - Msg Type
     - Note
   * -  **SUBSCRIBE**
     -
     -
   * - /set_teleport_mode
     - `example_interfaces/msg/Int32 <https://docs.ros2.org/foxy/api/example_interfaces/msg/Int32.html>`_
     - 0: Entrance
       1: Exit
   * - /set_target_exit
     - `example_interfaces/msg/String <https://docs.ros2.org/foxy/api/example_interfaces/msg/String.html>`_
     - Sets the target exit actor name.

Teleport Area Manager Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1

   * - Param Name

        [UE name if it is not pascal case of ROS one]
     - Type (Default)
     - Note
   * -  **ROS JSON SPAWN PARAMETER**
     -
     -
   * - /areas

        [TeleportAreas]
     - json
        .. code-block:: json

          [
            {
              "name": "teleport_area1",
              "transform": …,
              "teleport_mode": …,
              …
            },
            {
              "name": "teleport_area2",
              "transform": …,
              "teleport_mode": …,
              …
            }
          ]
     - List of TeleportAreas, each defined by name, transform, and parameters as defined in Teleport Area Parameters.
   * - /random
     - bool (false)
     - Selects the entrance and exit randomly.

Change Area
-----------

`BP_ChangeArea` is a child class of `BP_TeleportArea`.
It changes the target actor into a different class during teleportation.

.. video:: ../_static/videos/change_area.mp4
    :width: 750
    :height: 450

*Video: Change Area*

Change Area Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1

   * - Param Name
     - Type (Default)
     - Note
   * -  **ROS JSON SPAWN PARAMETER**
     -
     -
   * - /input_actor_classes
     - string[]
     - List of input actor classes.
   * - /output_actor_class
     - string ('')
     - Output actor class after teleportation and change.
