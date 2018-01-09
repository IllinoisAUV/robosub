#!/bin/bash
set -xe

# Takes the list of files to check as an argument, and checks if the files
# require changes. If they do, this script fails
clang-format -output-replacements-xml $@ | grep "<replacement " > /dev/null && exit 1 || exit 0
