#!/bin/bash
set -e
echo "Starting Xvfb on DISPLAY=:99..."
Xvfb :99 -screen 0 1280x720x24 -ac &
XVFB_PID=$!
sleep 2
for i in {1..10}; do
  if xdpyinfo -display :99 >/dev/null 2>&1; then
    echo "Xvfb ready"
    break
  fi
  sleep 1
done
exec "$@"
trap "kill $XVFB_PID" EXIT
