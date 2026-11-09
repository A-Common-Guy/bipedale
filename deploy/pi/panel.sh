#!/usr/bin/env bash
# Main process of the robot container: keeps the control panel (sandro_gui) on
# port 8080 running from the moment the Pi boots. The panel starts the robot's
# launch in its own session, so restarting the panel never stops the robot.
#
# On `docker stop` the launch gets one SIGINT first, as Ctrl-C, so the
# controller manager can idle the drives before the container goes away; this
# waits for it to exit within docker's stop timeout (deploy/pi/pi: 30 s).
cd /workspaces/sandro || exit 1
mkdir -p gui_runs
log=gui_runs/panel.log
max_log_bytes=5000000
launch_pattern='ros2 launch sandro_bringup'
child=0
housekeeper=0

# The panel appends (O_APPEND), so truncating in place keeps it writing at the
# new end of the same file.
housekeeping() {
  while sleep 60; do
    if [ -f "$log" ] && [ "$(stat -c %s "$log")" -gt "$max_log_bytes" ]; then
      cp "$log" "$log.1" && : >"$log"
    fi
    # ROS_LOG_DIR is a size-capped tmpfs: drop what no process wrote for a day.
    if [ -n "${ROS_LOG_DIR:-}" ] && [ -d "$ROS_LOG_DIR" ]; then
      find "$ROS_LOG_DIR" -mindepth 1 -type f -mmin +1440 -delete 2>/dev/null
      find "$ROS_LOG_DIR" -mindepth 1 -type d -empty -delete 2>/dev/null
    fi
  done
}

stop() {
  if pkill -INT -f "$launch_pattern" 2>/dev/null; then
    for _ in $(seq 50); do
      pgrep -f "$launch_pattern" >/dev/null || break
      sleep 0.5
    done
    pgrep -f "$launch_pattern" >/dev/null &&
      echo "$(date '+%F %T') launch still running after 25 s at stop" >>"$log"
  fi
  [ "$child" -ne 0 ] && kill "$child" 2>/dev/null
  [ "$housekeeper" -ne 0 ] && kill "$housekeeper" 2>/dev/null
  exit 0
}
trap stop TERM INT

housekeeping &
housekeeper=$!

while true; do
  if [ -f install/sandro_gui/share/sandro_gui/package.xml ]; then
    # shellcheck disable=SC1091
    source install/setup.bash
    ros2 run sandro_gui sandro_gui >>"$log" 2>&1 &
    child=$!
    wait "$child"
    status=$?
    echo "$(date '+%F %T') panel exited ($status), restarting" >>"$log"
    child=0
    sleep 2
  else
    sleep 5  # not built yet: deploy/pi/pi build
  fi
done
