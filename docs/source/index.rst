Welcome to RapyutaSimulationPlugins's documentation!
===================================

**RapyutaSimulationPlugins** includes robot common assets to build 
`ROS2 <https://docs.ros.org/en/foxy/index.html#>`_ simulator with 
`Unreal Engine <https://www.unrealengine.com/en-US/?lang=en-US>`_.

RapyutaSimulationPlugins provide Actor and ActorComponents 
which provide feature to control Actor from ROS2, output simulated sensor to ROS2 and etc.

RapyutaSimulationPlugins depends on `rclUE <https://rclue.readthedocs.io/en/devel/index.html>`_ 
to create ROS2 Node, Publisher/Subscriber, Serve server/client, and Action server/clients.

Pleaase check `rclUE <https://rclue.readthedocs.io/en/devel/index.html>`_  to check overview of software architecture.

A demo of our tool can be found in our port of the `turtlebot examples of UE4 <https://github.com/rapyuta-robotics/turtlebot3-UE>`_.

Repositories
------------
- Core features to bridge ROS2 and UE4: `<https://github.com/rapyuta-robotics/rclUE>`_
- This repository:  `<https://github.com/rapyuta-robotics/RapyutaSimulationPlugins>`_
- Turtlebot3 Simulation Example with UE4: `<https://github.com/rapyuta-robotics/turtlebot3-UE>`_ 

Contents
--------

.. toctree::
   
   components
   api