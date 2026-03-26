#!/usr/bin/env bash

set -euo pipefail

INTERFACE="${CAN_INTERFACE:-can0}"
BITRATE="${CAN_BITRATE:-500000}"
TX_QUEUE_LEN="${CAN_TX_QUEUE_LEN:-1000}"
RESTART_MS="${CAN_RESTART_MS:-100}"
MODE="${CAN_SETUP_MODE:-auto}"
TTY_DEVICE="${1:-}"

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run this script as root, for example:"
    echo "  sudo $0 [/dev/ttyACM0]"
    exit 1
fi

if ! command -v slcand >/dev/null 2>&1 || ! command -v ip >/dev/null 2>&1; then
    echo "Missing required tools. Install can-utils first:"
    echo "  sudo apt-get install can-utils"
    exit 1
fi

slcand_speed_flag() {
    case "$1" in
        10000) echo "-s0" ;;
        20000) echo "-s1" ;;
        50000) echo "-s2" ;;
        100000) echo "-s3" ;;
        125000) echo "-s4" ;;
        250000) echo "-s5" ;;
        500000) echo "-s6" ;;
        750000) echo "-s7" ;;
        1000000) echo "-s8" ;;
        *)
            echo "Unsupported CAN bitrate: $1" >&2
            exit 1
            ;;
    esac
}

interface_exists() {
    ip link show "${INTERFACE}" >/dev/null 2>&1
}

configure_socketcan_interface() {
    ip link set "${INTERFACE}" down 2>/dev/null || true
    ip link set "${INTERFACE}" type can bitrate "${BITRATE}" restart-ms "${RESTART_MS}" berr-reporting on
    ip link set "${INTERFACE}" txqueuelen "${TX_QUEUE_LEN}"
    ip link set "${INTERFACE}" up

    echo "${INTERFACE} is ready:"
    ip -details link show "${INTERFACE}"
}

configure_slcan_interface() {
    ip link set "${INTERFACE}" down 2>/dev/null || true
    ip link set "${INTERFACE}" txqueuelen "${TX_QUEUE_LEN}" 2>/dev/null || true
    ip link set "${INTERFACE}" up

    echo "${INTERFACE} is ready (slcan mode):"
    ip -details link show "${INTERFACE}"
}

stop_slcand() {
    ip link set "${INTERFACE}" down 2>/dev/null || true
    pkill -f "slcand.*${INTERFACE}" 2>/dev/null || true
    sleep 0.2
}

stop_slcand

if [[ "${MODE}" != "slcan" ]] && interface_exists; then
    echo "Found native ${INTERFACE}; configuring direct socketcan mode"
    configure_socketcan_interface
    exit 0
fi

if [[ "${MODE}" == "native" ]]; then
    echo "Native ${INTERFACE} not found. Put the adapter in candleLight mode or use CAN_SETUP_MODE=slcan."
    exit 1
fi

detect_tty_device() {
    local by_id

    by_id="$(find /dev/serial/by-id -maxdepth 1 -type l 2>/dev/null | grep -i 'canable' | head -n 1 || true)"
    if [[ -n "${by_id}" ]]; then
        echo "${by_id}"
        return
    fi

    ls /dev/ttyACM* 2>/dev/null | head -n 1 || true
}

if [[ -z "${TTY_DEVICE}" ]]; then
    TTY_DEVICE="$(detect_tty_device)"
fi

if [[ -z "${TTY_DEVICE}" ]]; then
    echo "No CANable/ttyACM device found. Pass the serial device explicitly."
    exit 1
fi

if [[ ! -e "${TTY_DEVICE}" ]]; then
    echo "Serial device does not exist: ${TTY_DEVICE}"
    exit 1
fi

SLCAND_SPEED="$(slcand_speed_flag "${BITRATE}")"

echo "Configuring ${INTERFACE} from ${TTY_DEVICE} at ${BITRATE} bps via slcand"
slcand -o -c "${SLCAND_SPEED}" "${TTY_DEVICE}" "${INTERFACE}"
sleep 0.5

configure_slcan_interface
