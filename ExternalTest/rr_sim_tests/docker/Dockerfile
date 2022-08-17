FROM ros:foxy as builder

# Other settings
SHELL ["/bin/bash", "-c"]
RUN apt update 

#ROS2 
## setup
ENV ROS2_WS=/home/ros2_ws
RUN mkdir -p ${ROS2_WS}/src/

## clone
WORKDIR ${ROS2_WS}/src
RUN git clone https://github.com/rapyuta-robotics/UE_msgs.git "ue_msgs"
COPY . rr_sim_tests/

## build
WORKDIR ${ROS2_WS}
RUN source /opt/ros/foxy/setup.bash && colcon build --symlink-install 


FROM ros:foxy

SHELL ["/bin/bash", "-c"]

ENV ROS2_WS=/home/ros2_ws
ENV ROBOT_NAME='amr1'

COPY --from=builder ${ROS2_WS} ${ROS2_WS}

RUN apt update && apt install -y python3-pip ros-foxy-tf-transformations && pip3 install transforms3d

RUN cp ${ROS2_WS}/src/rr_sim_tests/docker/fastdds_setup.sh /
RUN cp ${ROS2_WS}/src/rr_sim_tests/docker/fastdds_config.xml /

CMD /bin/bash -c ". ${ROS2_WS}/install/setup.bash && source fastdds_setup.sh && ros2 run rr_sim_tests test_random_spawn --ros-args -p robot_name:=${ROBOT_NAME}"

