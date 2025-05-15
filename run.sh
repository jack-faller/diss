#!/bin/sh
CLANG_PORT=1222
set -evo pipefail
	   # --expose $CLANG_PORT \
	   # --network=host --privileged \
docker run --name dis --volume "$(pwd):/proj" --interactive --tty --detach \
	   --volume /tmp/.X11-unix:/tmp/.X11-unix \
	   --env CLANG_PORT=$CLANG_PORT \
       --env DISPLAY="$DISPLAY" \
	   --env QT_X11_NO_MITSHM=1 \
	   ros_$(./get-ros-version.sh)
docker exec --workdir /proj --detach \
	   dis socat tcp-listen:$CLANG_PORT,reuseaddr,fork exec:clangd
