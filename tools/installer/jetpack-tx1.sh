#!/bin/bash

# Wrapper for the jetpack playbook
ansible-playbook --ask-become-pass --extra-vars "$1" jetpack-tx1.yml
