#!/usr/bin/env bash
#
# VCanSim - create and bring up a virtual CAN interface.
#
# Usage:
#   sudo ./scripts/setup_vcan.sh [interface]
#
# Defaults to vcan0.  Requires iproute2 and a kernel with the vcan module.

set -euo pipefail

IFACE="${1:-vcan0}"

if [[ "${EUID}" -ne 0 ]]; then
    echo "error: root privileges are required" >&2
    echo "       run: sudo $0 ${IFACE}" >&2
    exit 1
fi

if ! [[ "${IFACE}" =~ ^[a-zA-Z0-9_.-]{1,15}$ ]]; then
    echo "error: invalid interface name: ${IFACE}" >&2
    exit 1
fi

if ! command -v ip >/dev/null 2>&1; then
    echo "error: 'ip' not found; install iproute2" >&2
    exit 1
fi

# The vcan driver may be built in, so a modprobe failure is not fatal.
if command -v modprobe >/dev/null 2>&1; then
    modprobe vcan 2>/dev/null || true
fi

if ip link show "${IFACE}" >/dev/null 2>&1; then
    echo "Reusing existing interface ${IFACE}"
    ip link set dev "${IFACE}" down
else
    echo "Creating virtual CAN interface ${IFACE}"
    if ! ip link add dev "${IFACE}" type vcan; then
        echo "error: could not create ${IFACE}; is the vcan module available?" >&2
        echo "       try: sudo modprobe vcan" >&2
        exit 1
    fi
fi

ip link set dev "${IFACE}" up
ip link set dev "${IFACE}" txqueuelen 1000

echo
ip -details link show "${IFACE}"
echo

if command -v candump >/dev/null 2>&1; then
    echo "Ready. Inspect live traffic with:"
    echo "  candump ${IFACE}"
else
    echo "Ready. Install can-utils to inspect traffic:"
    echo "  sudo apt install can-utils"
    echo "  candump ${IFACE}"
fi
