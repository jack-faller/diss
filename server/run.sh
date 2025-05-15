#!/bin/sh
set -evo pipefail
ssh root@jackfaller.xyz rm -rf /disserv
scp -r $(dirname "$0") root@jackfaller.xyz:/disserv
ssh root@jackfaller.xyz pkill node || true
ssh root@jackfaller.xyz node /disserv/app.js /disserv/dissert.html
