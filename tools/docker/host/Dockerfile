FROM ros:kinetic

RUN apt-get update
RUN apt-get install -y ros-kinetic-mavros ros-kinetic-cv-bridge ros-kinetic-image-transport
RUN apt-get install -y clang-format
RUN apt-get install -y pylint

ADD install-cuda.sh /install-cuda.sh 
RUN /install-cuda.sh

VOLUME catkin_ws
WORKDIR /catkin_ws
CMD ["catkin_make"]
