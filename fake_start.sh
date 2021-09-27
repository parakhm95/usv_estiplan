#!/usr/bin/env bash



# Absolute path to this script.

SCRIPT=$(readlink -f $0)



# Containing directory

SCRIPTPATH=`dirname $SCRIPT`

cd "$SCRIPTPATH"/config/tmux



# start tmuxinator

tmuxinator start -p fake_session.yaml
