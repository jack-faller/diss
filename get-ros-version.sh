#!/bin/sh
grep ROS_DISTRO'.*=' "$(dirname "$0")"/docker/Dockerfile | sed 's/.*=// ; 1q'
