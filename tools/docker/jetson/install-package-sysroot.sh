#!/bin/bash
set -e

touch /tmp/installed-packages

if [[ -z "$@" ]]; then
    echo "Please specify packages to install"
    exit 1
fi

/get_debs.py $@ > /tmp/packages
# comm -13 <(sort file1) <(sort file2) returns all of the lines in file2 not in file1
# Download all packages that aren't already in the packages list
bash -c "cd /tmp/ && comm -13 <(sort /tmp/installed-packages) <(sort tmp/packages) | xargs apt-get -qq download"

# Install downloaded deb files then remove the old files
ls /tmp/*.deb | xargs --max-procs $(nproc) -I pkg -n1 dpkg-deb -x pkg $SYSROOT > /dev/null && rm /tmp/*.deb

# Update the list of installed packages
cat /tmp/installed-packages /tmp/packages | uniq > /tmp/installed-packages
