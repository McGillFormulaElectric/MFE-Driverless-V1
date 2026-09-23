#!/bin/bash
set -e

# Start VNC server on DISPLAY=:99
echo "Starting VNC server on DISPLAY=:99..."
mkdir -p /root/.vnc
echo "password" | vncpasswd -f > /root/.vnc/passwd
chmod 600 /root/.vnc/passwd

vncserver :99 -geometry 1280x720 -depth 24 &
VNC_PID=$!
sleep 3

# Wait for VNC to be ready
for i in {1..10}; do
  if netstat -tuln 2>/dev/null | grep -q 5999; then
    echo "VNC ready on :99 (port 5999)"
    break
  fi
  sleep 1
done

echo "Executing: $@"
exec "$@"

# Cleanup on exit
trap "kill $VNC_PID" EXIT
