# RapyutaSimulationPlugins

Robot common assets used with rclUE

## Online documentation

https://RapyutaSimulationPlugins.readthedocs.io/en/devel/

## Depedency repository

- [rclUE](https://github.com/rapyuta-robotics/rclUE) : Core functionalities to integrate ROS2 and UE4.

## Example repository

- [turtlebot3-UE](https://github.com/rapyuta-robotics/turtlebot3-UE) : Package to simulate turtlebot3 with UE4.

## Maintainer

yu.okamoto@rapyuta-robotics.com

ducanh.than@rapyuta-robotics.com

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
