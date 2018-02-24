# Jetson Installer
Note: Requires `ansible`

## Steps
1. Do a Jetpack install on the Jetson - currently using jetpack 3.1
2. Setup the jetson with a static ip address adding the file `eth0` to `/etc/network/interfaces.d/`
3. Run `ansible-playbook -i inventory install.yml`


# Logging in afterwards
The installer puts the id_jetson.pub key into the authorized_keys file of the 
ubuntu user on the jetson. In order to make ssh use this key to log into the 
jetson, add the following to `~/.ssh/config`.

```
Host 10.0.0.2
  User ubuntu
  StrictHostKeyChecking no
  IdentityFile /path/to/id_rsa_jetson
```
