#!/bin/bash
set -xe

# Takes the list of files to check as an argument, and checks if the files
# require changes. If they do, this script fails
clang-format -output-replacements-xml $@ | grep "<replacement " > /dev/null

# Previous command does opposite of desired.
if [ $? -eq 0 ]
then
    echo "Clang-format checks did not pass. Please run catkin_make --make-args clangformat"
    exit 1
fi
