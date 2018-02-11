#!/bin/bash

set -e

wget -O /tmp/cuda-repo-ubuntu1604_8.0.61-1_amd64.deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-repo-ubuntu1604_8.0.61-1_amd64.deb
dpkg -i /tmp/cuda-repo-ubuntu1604_8.0.61-1_amd64.deb


set +e
apt-get update
set -e

# Install cuda 9.1
apt install -y cuda-toolkit-8-0


echo 'export PATH=/usr/local/cuda/bin:$PATH' >> /root/.bashrc

cd /
rm -rf /tmp/*
