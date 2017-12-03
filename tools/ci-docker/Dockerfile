FROM ros:kinetic

RUN apt-get update
RUN apt-get install -y openssl ros-kinetic-cv-bridge


VOLUME "/catkin_ws"
WORKDIR "/catkin_ws"
CMD ["catkin_make"]
