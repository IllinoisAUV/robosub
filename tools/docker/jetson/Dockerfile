FROM ubuntu:16.04

ENV SYSROOT=/sysroot
RUN mkdir -p $SYSROOT


RUN dpkg --add-architecture arm64
ADD arm64-sources.list /etc/apt/sources.list.d/arm64.list
RUN sed -i 's/deb /deb [arch=amd64] /g' /etc/apt/sources.list && sed -i 's/deb-src /deb-src [arch=amd64] /g' /etc/apt/sources.list

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN apt-get update

RUN apt-get -qq install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
        wget \
        cmake \
        ros-kinetic-ros-base \
        apt-rdepends \
        python \
        python-apt

ENV TRIPLET=aarch64-linux-gnu
ENV CROSS_COMPILE=${TRIPLET}-

ADD get_debs.py /get_debs.py

ADD install-package-sysroot.sh /install-package-sysroot.sh

RUN /install-package-sysroot.sh ros-kinetic-desktop-full
RUN /install-package-sysroot.sh ros-kinetic-mavros

# Fix some hard coded include directories
RUN find $SYSROOT/opt/ros/kinetic/share/ -name "*Config.cmake" | xargs sed -i 's/set(include \${idir})/set(include \$ENV{SYSROOT}\${idir})/g'
RUN find $SYSROOT/opt/ros/kinetic/share/ -name "*Config.cmake" | xargs sed -i 's/IS_DIRECTORY \${idir}/IS_DIRECTORY \$ENV{SYSROOT}\${idir}/g'

ADD fix-links.sh /fix-links.sh
RUN /fix-links.sh

RUN find $SYSROOT/opt/ros/kinetic/share/ -name "*Config.cmake" | xargs sed -i 's/foreach(path \/opt\/ros\/kinetic\/lib;\/opt\/ros\/kinetic\/lib)/foreach(path $ENV{SYSROOT}\/opt\/ros\/kinetic\/lib;$ENV{SYSROOT}\/opt\/ros\/kinetic\/lib)/g'

RUN find $SYSROOT/opt/ros/kinetic/share/ -name "*Config.cmake" | xargs sed -i '/elseif(IS_ABSOLUTE \${library})/{ N; s/elseif(IS_ABSOLUTE \${library})\n\(.*\)list(APPEND \(.*\)_LIBRARIES ${library})/elseif(IS_ABSOLUTE \${library})\n\1list(APPEND \2_LIBRARIES $ENV{SYSROOT}${library})/ }'


RUN cd $SYSROOT/usr/lib/aarch64-linux-gnu && ln -s mesa/libGL.so.1 .

ADD ros-entrypoint.sh /ros-entrypoint.sh


ADD install-cuda.sh /install-cuda.sh
RUN /install-cuda.sh

ADD toolchain.cmake /toolchain.cmake

RUN /install-package-sysroot.sh python-catkin-pkg python-catkin-pkg-modules

VOLUME /catkin_ws
WORKDIR /catkin_ws
ENTRYPOINT ["/ros-entrypoint.sh"]
CMD ["catkin_make", "-DCMAKE_TOOLCHAIN_FILE=/toolchain.cmake", "-DCMAKE_BUILD_TYPE=Release"]
