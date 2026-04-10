#!/bin/bash

set -euo pipefail

echo ""
echo "Install udev rule for RoboMaster C board serial port."
echo "It will create a stable alias /dev/ttyNAV (recommended)."
echo ""

UDEV_RULES_FILE="/etc/udev/rules.d/99-RoboMaster_C_Board.rules"

# Optional: pass serial as first arg.
TARGET_SERIAL="${1:-}"

if [ -z "$TARGET_SERIAL" ]; then
	for dev in /dev/ttyACM*; do
		[ -e "$dev" ] || continue
		props="$(udevadm info -q property -n "$dev" || true)"
		vid="$(echo "$props" | grep -m1 '^ID_VENDOR_ID=' | cut -d= -f2 || true)"
		pid="$(echo "$props" | grep -m1 '^ID_MODEL_ID=' | cut -d= -f2 || true)"
		serial="$(echo "$props" | grep -m1 '^ID_SERIAL_SHORT=' | cut -d= -f2 || true)"

		# Supported common USB-UART chips used by this board.
		if { [ "$vid" = "1a86" ] && [ "$pid" = "55d3" ]; } || { [ "$vid" = "10c4" ] && [ "$pid" = "ea60" ]; }; then
			if [ -n "$serial" ]; then
				TARGET_SERIAL="$serial"
				break
			fi
		fi
	done
fi

if [ -z "$TARGET_SERIAL" ]; then
	echo "[ERROR] Cannot auto-detect target board serial."
	echo "Please plug in the board first, then run this script again."
	echo "Or run: $0 <ID_SERIAL_SHORT>"
	exit 1
fi

UDEV_RULE="SUBSYSTEM==\"tty\", KERNEL==\"ttyACM*\", ATTRS{serial}==\"${TARGET_SERIAL}\", SYMLINK+=\"ttyNAV\", GROUP=\"dialout\", MODE=\"0660\""

echo "Setting udev rules..."
echo "$UDEV_RULE" | sudo tee "$UDEV_RULES_FILE" > /dev/null

echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo -e "\e[32mUdev rules have been set and reloaded successfully.\e[0m"
echo "Bound serial: $TARGET_SERIAL -> /dev/ttyNAV"

CURRENT_USER=$(whoami)

echo ""
echo "Adding user $CURRENT_USER to dialout group..."
sudo usermod -aG dialout "$CURRENT_USER"

echo -e "\e[32mUser '$CURRENT_USER' has been added to the dialout group successfully\e[0m"
echo -e "\e[33mPlease unplug/replug the board and use /dev/ttyNAV in your params.\e[0m"
