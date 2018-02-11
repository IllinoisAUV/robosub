#!/bin/bash
set -e

find $SYSROOT -type l -print | while read l; do 
    case $(readlink $l) in (/*) ln -sf $SYSROOT$(readlink $l) $l
    esac
done
