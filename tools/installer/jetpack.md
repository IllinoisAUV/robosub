# Jetson Command Line Flashing

Reference material: https://devtalk.nvidia.com/default/topic/982779/jetson-tk1/how-to-perform-a-correct-fresh-ubuntu-install-on-jetson-tk1/post/5040584/#5040584

We can skip the jetpack GUI and flash the Jetson OS image using the flash.sh script provided by the Jetson drivers package from the [Download Center](https://developer.nvidia.com/embedded/downloads).



First, download and unpack the drivers package


```sh
wget --quiet https://developer.nvidia.com/embedded/dlc/l4t-jetson-tx2-driver-package-28-1 -O drivers-package-tx2.tar.gz
tar -xf drivers-package-tx2.tar.gz
# Now the Linux_for_Tegra directory should exist

# Rename the folder to Linux_for_Tegra_tx2 for the Orbitty BSP
mv Linux_for_Tegra Linux_for_Tegra_tx2
```

Next, download and unpack the sample rootfs to the `Linux_for_Tegra_tx2/rootfs` folder. Note the tar command requires sudo to get the permissions on the folder correct
```sh
wget --quiet https://developer.nvidia.com/embedded/dlc/l4t-sample-root-filesystem-28-1 -O rootfs-tx2.tar.bz2
sudo tar -xf rootfs-tx2.tar.bz2 -C Linux_for_Tegra_tx2/rootfs
```

Now, apply the proprietary nvidia binaries from the `Linux_for_Tegra_tx2` folder
```sh
sudo apply_binaries.sh
```

We can now download and unpack the Orbitty BSP in the `Linux_for_Tegra_tx2` folder
```sh
wget --quiet http://www.connecttech.com/ftp/Drivers/CTI-L4T-V111.tgz -O CTI-L4T-V111.tgz
tar -xf CTI-L4T-V111.tgz
cd CTI-L4T && sudo ./install.sh
```

Now place the Jetson in Recovery Mode, as specified in the `README.md` and, in the `Linux_for_Tegra_tx2` folder, run
```sh
sudo ./flash.sh orbitty mmcblk0p1
```

Now the jetson will have a default Ubuntu 16.04 image installed on it. It will not have CUDA or any of the other important Nvidia packages, but those can be installed separately.
