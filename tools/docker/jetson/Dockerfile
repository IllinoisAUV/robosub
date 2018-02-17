FROM ubuntu:16.04

# SYSROOT is where the root filesystem that mimics the jetson will live
ENV SYSROOT=/sysroot
RUN mkdir -p $SYSROOT


# We add arm64 as an architecture on the container to be able to download arm64 versions of packages
RUN dpkg --add-architecture arm64
ADD arm64-sources.list /etc/apt/sources.list.d/arm64.list
# Makes apt-get update not fail with errors on things, since the URL's for arm64 packages are not shared with amd64 packages
RUN sed -i 's/deb /deb [arch=amd64] /g' /etc/apt/sources.list && sed -i 's/deb-src /deb-src [arch=amd64] /g' /etc/apt/sources.list

# Make apt aware of ros packages
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Install required packages
RUN apt-get update && apt-get -qq install -y \
        gcc-aarch64-linux-gnu \
        g++-aarch64-linux-gnu \
        wget \
        cmake \
        ros-kinetic-ros-base \
        apt-rdepends \
        python \
        python-apt

# Triplet specifies the type of compiler to use
ENV TRIPLET=aarch64-linux-gnu

ADD get_debs.py /get_debs.py
ADD install-package-sysroot.sh /install-package-sysroot.sh

# Add install necessary packages
RUN /install-package-sysroot.sh ros-kinetic-desktop-full
RUN /install-package-sysroot.sh ros-kinetic-mavros


# Fix absolute symlinks in the sysroot to point to the correct files inside the sysroot
ADD fix-links.sh /fix-links.sh
RUN /fix-links.sh

# Fix some hard coded variables in cmake files
RUN find $SYSROOT/opt/ros/kinetic/share/ -name "*Config.cmake" | xargs sed -i 's/set(include \${idir})/set(include \$ENV{SYSROOT}\${idir})/g'
RUN find $SYSROOT/opt/ros/kinetic/share/ -name "*Config.cmake" | xargs sed -i 's/IS_DIRECTORY \${idir}/IS_DIRECTORY \$ENV{SYSROOT}\${idir}/g'
RUN find $SYSROOT/opt/ros/kinetic/share/ -name "*Config.cmake" | xargs sed -i 's/foreach(path \/opt\/ros\/kinetic\/lib;\/opt\/ros\/kinetic\/lib)/foreach(path $ENV{SYSROOT}\/opt\/ros\/kinetic\/lib;$ENV{SYSROOT}\/opt\/ros\/kinetic\/lib)/g'
RUN find $SYSROOT/opt/ros/kinetic/share/ -name "*Config.cmake" | xargs sed -i '/elseif(IS_ABSOLUTE \${library})/{ N; s/elseif(IS_ABSOLUTE \${library})\n\(.*\)list(APPEND \(.*\)_LIBRARIES ${library})/elseif(IS_ABSOLUTE \${library})\n\1list(APPEND \2_LIBRARIES $ENV{SYSROOT}${library})/ }'


# Link a failed symlink
RUN cd $SYSROOT/usr/lib/aarch64-linux-gnu && ln -s mesa/libGL.so.1 .

ADD install-cuda.sh /install-cuda.sh
RUN /install-cuda.sh

# Add a cmake toolchain file specifying our sysroot
ADD toolchain.cmake /toolchain.cmake

# Install the catkin build tools to the sysroot
RUN /install-package-sysroot.sh python-catkin-pkg python-catkin-pkg-modules

# Run commands through an entrypoint that sources the ros setup.bash file
ADD ros-entrypoint.sh /ros-entrypoint.sh

VOLUME /catkin_ws
WORKDIR /catkin_ws
ENTRYPOINT ["/ros-entrypoint.sh"]
CMD ["catkin_make", "-DCMAKE_TOOLCHAIN_FILE=/toolchain.cmake", "-DCMAKE_BUILD_TYPE=Release"]
