FROM ros:kinetic

RUN apt-get update && apt-get install -y ros-kinetic-mavros ros-kinetic-mavros-msgs ros-kinetic-cv-bridge ros-kinetic-image-transport

RUN mkdir -p /catkin_ws

VOLUME "/catkin_ws"
WORKDIR "/catkin_ws"
CMD ["catkin_make"]
