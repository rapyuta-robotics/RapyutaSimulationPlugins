Getting Started
================

This page will introduce you how to setup rclUE enabled UE project and run example.

Please check `rclUE documentation <https://rclue.readthedocs.io/en/latest/examples.html>`_ 
for ROS 2 basic tutorial with rclUE.

Setup and run UE Project
------------------------
1. Download the lastest UE5 for Linux by following `Unreal Engine for Linux <https://www.unrealengine.com/en-US/linux>`_.
2. Clone and build

.. code-block:: shell

    git clone `turtlebot3-UE <https://github.com/rapyuta-robotics/turtlebot3-UE/>`_
    cd turtlebot3-UE
    ./update_project_files.sh
    make turtlebot3Editor

\* `devel` branch is supported to run with Ubuntu 20.04 with ROS 2 foxy.

\* `jammy` branch is supported to run with Ubuntu 22.04 with ROS 2 humble.

3. Run

.. code-block:: shell

    ./run_editor.sh

4. Open `turtlebot3-UE/Content/Maps/turtlebot3_benchmark.umap` and Play.

Since the prooject is set to use
`ROS2 with Discovery Server <https://docs.ros.org/en/foxy/Tutorials/Advanced/Discovery-Server/Discovery-Server.html>`_
to communicate with ROS2 Node in UE, you needs to execute `source turtlebot3_UE/fastdds_setup.sh`.

\* In `jammy` branch, you can execute `./run_editor false` to run project without Discovery Server.


Control from ROS2 2
------------------------

turtlebot3_benchmark.umap has default turtlebot waffle robot in the level. You can replace that with other turtlebot3.

- You can control robot from teleop
    .. code-block:: shell

         ros2 run teleop_twist_keyboard teleop_twist_keyboard 

- You can control robot with nav2
    .. code-block:: shell

        ros2 launch nav2_bringup tb3_simulation_launch.py use_simulator:=False map:=<path to turtlebot3-UE>/Content/Turtlebot3_benchmark.yaml