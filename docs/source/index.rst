Welcome to RapyutaSimulationPlugins's documentation!
===================================

**RapyutaSimulationPlugins** includes robot common assets to build 
`ROS2 <https://docs.ros.org/en/foxy/index.html#>`_ simulator with 
`Unreal Engine <https://www.unrealengine.com/en-US/?lang=en-US>`_.

RapyutaSimulationPlugins provide UE components 
which provide feature to control Actor from ROS2, output simulated sensor to ROS2 and etc.

This plugins support to configure distributed simulation by leveraging 
the `UE's multiplayer feature <https://docs.unrealengine.com/4.27/en-US/InteractiveExperiences/Networking/>`_.
Please check :doc:`distributed simulation` for details.

This plugin belongs to a group of rclUE softwares which contains multiple repositories. 
Please check `rclUE <https://rclue.readthedocs.io/en/devel/index.html>`_  to understand overview of rclUE related repositories.

A demo of our tool can be found in the `turtlebot examples of UE4 <https://github.com/rapyuta-robotics/turtlebot3-UE>`_.

Repositories
------------
- Core features to bridge ROS2 and UE: `<https://github.com/rapyuta-robotics/rclUE>`_
- This repository:  `<https://github.com/rapyuta-robotics/RapyutaSimulationPlugins>`_
- Turtlebot3 Simulation Example with UE: `<https://github.com/rapyuta-robotics/turtlebot3-UE>`_ 

Contents
--------

.. toctree::
   
   overview
   components
   distributed_simulation
   ue_api
