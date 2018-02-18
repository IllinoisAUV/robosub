#!/bin/bash

set -e

wget --quiet -O /tmp/cuda-repo-ubuntu1604_8.0.61-1_amd64.deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-repo-ubuntu1604_8.0.61-1_amd64.deb
dpkg -i /tmp/cuda-repo-ubuntu1604_8.0.61-1_amd64.deb


set +e
apt-get -qq update
set -e



# Install cuda 9.1
apt-get -qq install -y cuda-cross-aarch64-8-0


ln -sf /usr/local/cuda-8.0/targets/aarch64-linux/lib /usr/local/cuda-8.0/lib64
ln -sf /usr/local/cuda-8.0/targets/aarch64-linux/include /usr/local/cuda-8.0/include
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> /root/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64' >> /root/.bashrc

cd /
rm -rf /tmp/*
