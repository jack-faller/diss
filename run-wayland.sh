#!/bin/sh
CLANG_PORT=1222
set -evo pipefail
	   # --expose $CLANG_PORT \
	   # --network=host --privileged \
docker run --name dis --volume "$(pwd):/proj" --rm --interactive --tty --detach \
       --env XDG_RUNTIME_DIR=/tmp \
	   --volume /tmp/.X11-unix:/tmp/.X11-unix \
       --env WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
	   --volume "$XDG_RUNTIME_DIb/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY" \
	   --env CLANG_PORT=$CLANG_PORT \
       --env DISPLAY="$DISPLAY" \
       --env QT_QPA_PLATFORM="$QT_QPA_PLATFORM" \
       --env QT_QPA_PLATFORM_THEME="$QT_QPA_PLATFORM_THEME" \
	   --user $(id -u):$(id -g) \
	   ros_$(./get-ros-version.sh) bash
echo 3
docker exec --workdir /proj --detach \
	   dis socat tcp-listen:$CLANG_PORT,reuseaddr,fork exec:clangd
