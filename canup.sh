#!/usr/bin/env bash
# usage: ./canup.sh [bitrate] [iface]   — default: 1000000 can0
# Must match the drives' can.config.baud_rate: factory default is 500000,
# commissioned chains run 1000000 (e.g. ./canup.sh 500000 for a stock drive).
set -e
BITRATE=${1:-1000000}
IFACE=${2:-can0}
# sample-point 0.875: gs_usb defaults to 0.747 at 1 Mbps, which mismatches the
# drive's ~87.5% timing and silently drops host->drive frames (SDO timeouts,
# ignored save). restart-ms 100: recover automatically from bus-off.
# txqueuelen 1000: the kernel default of 10 overflows into ENOBUFS the moment
# the bus stops ACKing (wrong bitrate / drives off); a deep queue keeps sends
# working while the error is diagnosed.
sudo ip link set "$IFACE" down 2>/dev/null || true
sudo ip link set "$IFACE" type can bitrate "$BITRATE" sample-point 0.875
sudo ip link set "$IFACE" txqueuelen 1000
sudo ip link set "$IFACE" up
ip -details link show "$IFACE"
