#!/bin/bash
set -e

touch /tmp/packages-all

if [[ -z "$@" ]]; then
    echo "Please specify packages to install"
    exit 1
fi
/get_debs.py $@ > /tmp/packages
# comm -13 <(sort file1) <(sort file2) returns all of the lines in file2 not in file1
bash -c "cd /tmp/ && comm -13 <(sort /tmp/packages-all) <(sort /tmp/packages) | xargs apt-get -qq download"
ls /tmp/*.deb | xargs --max-procs $(nproc) -I pkg -n1 dpkg-deb -x pkg $SYSROOT > /dev/null && rm /tmp/*.deb
cat /tmp/packages-all /tmp/packages | uniq > /tmp/packages-all
