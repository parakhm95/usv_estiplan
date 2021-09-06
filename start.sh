#!/usr/bin/env bash



# Absolute path to this script.

SCRIPT=$(readlink -f $0)



# Containing directory

SCRIPTPATH=`dirname $SCRIPT`

cd "$SCRIPTPATH"/config



# start tmuxinator

tmuxinator start -p session.yaml
