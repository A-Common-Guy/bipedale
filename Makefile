.PHONY: build sim sim-cascade real test shell hardware-shell xauth pi-build pi-launch pi-shell pi-can

XAUTH ?= /tmp/.sandro.xauth

build:
	# Every service is behind a profile, so a bare `docker compose build` selects
	# nothing. They all share one image, so building any single service is enough.
	docker compose --profile shell build shell

# GUI windows (MuJoCo, RViz) need an X cookie the container can read. The
# family byte is wildcarded because the container hostname differs from the
# host one. Without a display this is skipped; use headless:=true then.
xauth:
	@[ -d $(XAUTH) ] && rmdir $(XAUTH) || true
	@touch $(XAUTH)
	@if [ -n "$$DISPLAY" ] && command -v xauth >/dev/null 2>&1; then \
	  xauth nlist $$DISPLAY | sed -e 's/^..../ffff/' | xauth -f $(XAUTH) nmerge -; \
	else \
	  printf '%s\n' 'No DISPLAY or xauth found; run launches with headless:=true rviz:=false.'; \
	fi
	@chmod 644 $(XAUTH)

sim: xauth
	docker compose --profile sim run --rm sim

# Same simulation with the PID cascade instead of the whole-body controller.
sim-cascade: xauth
	SANDRO_LAUNCH_ARGS="controller:=driving_balancer $(SANDRO_LAUNCH_ARGS)" \
	  docker compose --profile sim run --rm sim

real:
	@printf '%s\n' 'REAL MODE DOES NOT ARM DRIVES. Verify lifted robot, E-stop, CAN mapping, signs, zeros, and limits.'
	docker compose --profile real run --rm real

test:
	docker compose --profile test run --rm test

shell: xauth
	docker compose --profile shell run --rm shell

hardware-shell: xauth
	@printf '%s\n' 'Shell WITH the IMU attached. Launching the balancer here arms the drives.'
	docker compose --profile hardware run --rm hardware-shell

# Raspberry Pi deployment (see deploy/pi/README.md)
pi-build:
	deploy/pi/pi build

pi-launch:
	deploy/pi/pi launch $(ARGS)

pi-shell:
	deploy/pi/pi shell

pi-can:
	deploy/pi/pi can
