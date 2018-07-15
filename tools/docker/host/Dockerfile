FROM ros:kinetic

RUN apt-get update && apt-get install -y \
    ros-kinetic-mavros \
    ros-kinetic-cv-bridge \
    ros-kinetic-image-transport \
    ros-kinetic-tf \
    ros-kinetic-sophus* \
    ros-kinetic-rqt-gui* \
    ros-kinetic-xacro \
    ros-kinetic-gazebo* \
    ros-kinetic-video-stream-opencv \
    ros-kinetic-pid \
    clang-format \
    pylint

RUN /opt/ros/kinetic/lib/mavros/install_geographiclib_datasets.sh

ADD install-cuda.sh /install-cuda.sh 
RUN /install-cuda.sh

VOLUME catkin_ws
WORKDIR /catkin_ws
CMD ["catkin_make"]
