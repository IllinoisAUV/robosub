# Jetson Installer
Note: Requires `ansible`

## Steps
1. Do a Jetpack install on the Jetson
2. Setup the jetson with a static ip address adding the file `eth0` to `/etc/network/interfaces.d/`
3. Run `ansible-playbook -i inventory install.yml`
