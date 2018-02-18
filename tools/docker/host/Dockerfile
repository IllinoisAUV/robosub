FROM ros:kinetic

RUN apt-get update
RUN apt-get -qq install -y \
    ros-kinetic-mavros \
    ros-kinetic-cv-bridge \
    ros-kinetic-image-transport \
    clang-format \
    pylint

ADD install-cuda.sh /install-cuda.sh 
RUN /install-cuda.sh

VOLUME catkin_ws
WORKDIR /catkin_ws
CMD ["catkin_make"]
