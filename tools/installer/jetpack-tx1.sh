#!/bin/bash

# To flash for orbitty, specify orbitty as the command line argument

VARS=""
case $1 in
    dev);;
    orbitty)
        VARS="orbitty=1"
        ;;
    *)
        echo "Please specify dev or orbitty as command line argument"
        exit 1
        ;;
esac

# Wrapper for the jetpack playbook
ansible-playbook --ask-become-pass --extra-vars "$VARS" jetpack-tx1.yml
