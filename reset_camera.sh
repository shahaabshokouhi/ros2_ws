#!/bin/bash
# Software replug for a wedged RealSense ("Frames didn't arrive within 5
# seconds" / "Device or resource busy"). Equivalent to unplug/replug:
# performs a USB-level port reset on the first Intel RealSense found.

DEV=$(lsusb | grep -i "8086:0b3a\|Intel.*RealSense" | head -1 | awk '{printf "/dev/bus/usb/%s/%s", $2, substr($4,1,3)}')
if [ -z "$DEV" ] || [ ! -e "$DEV" ]; then
    echo "No RealSense device found on USB."
    exit 1
fi

echo "Resetting $DEV ..."
python3 - "$DEV" <<'EOF'
import fcntl, sys
USBDEVFS_RESET = 21780
with open(sys.argv[1], 'w') as f:
    fcntl.ioctl(f, USBDEVFS_RESET, 0)
print("USB reset OK — wait ~4 seconds before starting SLAM.")
EOF
