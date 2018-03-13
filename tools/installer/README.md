# Jetson Installer
Note: Requires ansible>=2.4. Run `install-ansible.sh` on ubuntu to get the latest version

# Steps
1. Put the Jetson in USB Recovery mode
  * Power off the device completely.
  * Press the power button
  * Hold the recovery button
  * Press and release the reset button and hold the recovery button for 2 seconds
2. The Jetson should now be ssh-able on 10.0.0.2 with username and password being ubuntu
3. Run `ansible-playbook -i inventory install.yml` or `full-install.sh`


# Logging in afterwards
The `full_install.sh` script puts the id_jetson.pub key into the authorized_keys file of the
ubuntu user on the jetson. In order to make ssh use this key to log into the
jetson, add the following to `~/.ssh/config`.

```
Host 10.0.0.2
  User ubuntu
  StrictHostKeyChecking no
  IdentityFile /path/to/id_rsa_jetson # CHANGE THIS PATH
```


# Things that are being done
* Passwordless sudo
* An SSH Key to authorized_keys
* Disable NetworkManager and always use 8.8.8.8 for DNS
* Install ROS and needed ros packages
* Install ZED SDK
* Create a catkin_ws in /home/ubuntu/catkin_ws
* Install udev rules for the bottom camera
* Install tiscamera (bottom camera) control software
* Install CUDA
