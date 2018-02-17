#!/bin/bash
set -e

# Redo all absolute links to be under the sysroot
find $SYSROOT -type l -print | while read l; do 
    case $(readlink $l) in (/*) ln -sf $SYSROOT$(readlink $l) $l
    esac
done
