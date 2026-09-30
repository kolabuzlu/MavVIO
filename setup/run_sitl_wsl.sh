#!/bin/bash
# Starts the Ubuntu-built ArduPlane simulator for tools/sitl.py (build "wsl"): runs it in the folder given
# as the first argument, with its output in console.log there and no terminal input. The remaining
# arguments go to the simulator. Its exit code is added to console.log when it ends.
# Two WSL quirks this works around: started directly by wsl.exe (or with "exec" here), the simulator dies
# as soon as a client connects; and "broken pipe" signals are ignored, because WSL's port relay makes a
# short test connection before the real one.
cd "$1" || exit 1
shift
trap '' PIPE
"$HOME/ardupilot/build/sitl/bin/arduplane" "$@" > console.log 2>&1 < /dev/null
echo "=== simulator ended with exit code $?" >> console.log
