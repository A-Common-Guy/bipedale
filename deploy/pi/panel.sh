#!/usr/bin/env bash
# Main process of the robot container: keeps the control panel (sandro_gui) on
# port 8080 running from the moment the Pi boots. The panel starts the robot's
# launch in its own session, so restarting the panel never stops the robot.
#
# On `docker stop` the launch gets one SIGINT first, as Ctrl-C, so the
# controller manager can idle the drives before the container goes away.
cd /workspaces/sandro || exit 1
mkdir -p gui_runs
log=gui_runs/panel.log
child=0

stop() {
  pkill -INT -f 'ros2 launch sandro_bringup' 2>/dev/null && sleep 5
  [ "$child" -ne 0 ] && kill "$child" 2>/dev/null
  exit 0
}
trap stop TERM INT

while true; do
  if [ -f install/sandro_gui/share/sandro_gui/package.xml ]; then
    if [ -f "$log" ] && [ "$(stat -c %s "$log")" -gt 5000000 ]; then
      mv "$log" "$log.1"
    fi
    # shellcheck disable=SC1091
    source install/setup.bash
    ros2 run sandro_gui sandro_gui >>"$log" 2>&1 &
    child=$!
    wait "$child"
    echo "$(date '+%F %T') panel exited ($?), restarting" >>"$log"
    child=0
    sleep 2
  else
    sleep 5  # not built yet: deploy/pi/pi build
  fi
done
