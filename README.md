# RapyutaSimulationPlugins

Robot common assets/utils used with rclUE

## Online documentation

https://RapyutaSimulationPlugins.readthedocs.io/en/devel/

## Depedency repository

- [rclUE](https://github.com/rapyuta-robotics/rclUE) : Core functionalities to integrate ROS2 and UE4.

## Example repository

- [turtlebot3-UE](https://github.com/rapyuta-robotics/turtlebot3-UE) : Package to simulate turtlebot3 with UE4.

## Example BP robots
- [SampleArm](https://github.com/rapyuta-robotics/RapyutaSimulationPlugins/tree/devel/Content/Robots/SampleArm): Simple kinematics-based 4-link arm enabled with ROS controller.
- [Turtlebot3](https://github.com/rapyuta-robotics/RapyutaSimulationPlugins/tree/devel/Content/Robots/Turtlebot3): Kinematics & Physics-based turtlebot3 Burger & Waffle enabled with ROS controller.
- [Skeletal Turtlebot3](https://github.com/rapyuta-robotics/RapyutaSimulationPlugins/tree/devel/Content/SkeletalRobots/Turtlebot3): Physics-enabled skeletal mesh component-based turtlebot3 robots of types:
    - _BallCasterSphereWheeled_ : With ball caster sphere wheel
    - _ConvexWheeled_ : Wheels have convex-hull collision
    - _SphereWheeled_ : Wheels have sphere collision
    - _StaticMeshConstrained_ : Built from separate static mesh components connected to one another by physics constraints
    - _FullLockConstrained_ : All physics constraints are locked
    - _WheeledVehicle_ : Utilize SimpleWheeledVehicleMovement
- [SkeletalTurtlebot3Examples](https://github.com/rapyuta-robotics/RapyutaSimulationPlugins/tree/devel/Content/SkeletalRobots/Turtlebot3/SkeletalTurtlebot3Examples.umap) level: Have all example skeletal robots being put to automatically move forward upon Play.

# Install pre-commit
Please install pre-commit before commiting your changes.
Follow this instruction https://pre-commit.com/

then run

```bash
pre-commit install
```

# Documentation

## Online doc are available in readthedoc

https://RapyutaSimulationPlugins.readthedocs.io/en/devel/

## Tools

Documentation is built with three tools

- [doxygen](http://www.doxygen.org)
- [sphinx](http://www.sphinx-doc.org)
- [breathe](https://breathe.readthedocs.io)

## Locally build

1. Install tools in #tools section.
2. Build
   ```
   cd docs
   make --always-make html
   ```
3. Open following in your browser.
   - Sphinx at `file:///<path to cloned repo>/docs/source/_build/html/index.html`
   - Original doxygen output at `file:///<path to cloned repo>/docs/source/_build/html/doxygen_generated/html/index.html`

# Maintainer

yu.okamoto@rapyuta-robotics.com

ducanh.than@rapyuta-robotics.com
