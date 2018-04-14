FROM ubuntu:16.04


RUN apt-get update && apt-get install -y \
    git \
    sudo


RUN git clone https://github.com/ardupilot/ardupilot
RUN cd ardupilot && git checkout -b ArduSub-stable

RUN apt-get install -y lsb-core

RUN yes | env USER=root ardupilot/Tools/scripts/install-prereqs-ubuntu.sh

RUN cd ardupilot && ./waf configure && ./waf -j4
RUN echo "export PATH=/ardupilot/Tools/autotest/:$PATH" >> ~/.bashrc

VOLUME ["/ardupilot/ArduSub/mav.parm"]
CMD ["bash", "-ic", "cd /ardupilot/ArduSub && [ -f mav.parm ] && ../Tools/autotest/sim_vehicle.py -j$(nproc) || >&2 echo Please provide /ardupilot/ArduSub/mav.parm"]
