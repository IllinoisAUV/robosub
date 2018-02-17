#!/usr/bin/env python

# This file determines all of the dependencies of a package and outputs the to 
# stdout with for installation. Running ./get-debs.py <pkg1> <pkg2> ... will 
# output a list of packages that should be install
from __future__ import print_function

import subprocess
import sys

import apt

if len(sys.argv) < 2:
    print("Please specify a package to install", file=sys.stderr)
    sys.exit(1)

packages = str(subprocess.check_output(["apt-rdepends"] + sys.argv[1:]))

packages = [line for line in packages.split("\n") if "Depends:" not in line]


cache = apt.Cache()

final = []
for package in packages:
    if package.strip() == '':
        continue
    if cache.is_virtual_package(package):
        providers = cache.get_providing_packages(package)
        if len(providers) == 1:
            provider = providers[0]
            if(provider.architecture() not in ['arm64', 'all']):
                if cache.has_key(provider.name + ':arm64'):
                    final.append(cache[provider.name + ':arm64'])
                elif cache.has_key(provider.name + ':all'):
                    final.append(cache[provider.name + ':all'])
                else:
                    print(package + " not found for arm64 architecture. Ignoring", file=sys.stderr)
        else:
            arm_providers = [provider for provider in providers
                             if provider.architecture() == 'arm64']
            if len(arm_providers) == 1:
                final.append(arm_providers[0])
            elif len(arm_providers) > 1:
                # Multiple providers in the arm architecture. Just choose the
                # first one and hope for the best
                final.append(arm_providers[0])
            else:
                # Just pick the first one in the list and hope for the best
                final.append(providers[0])

        continue
    if cache.has_key(package + ":arm64"):
        pkg = cache[package+':arm64']
        final.append(pkg)
    elif cache.has_key(package + ":all"):
        pkg = cache[package]
        final.append(pkg)
    else:
        # Ignore packages that aren't found. Usually this is because no versions
        # of the package is published, meaning we can safely ignore it
        print(package + " not found. Ignoring", file=sys.stderr)
print('\n'.join([pkg.name for pkg in final]))
