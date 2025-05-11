#!/bin/bash

echo "Identifying the dimmer..."
ARDUINO_PATH=/dev/ttyACM0 #$(dmesg | grep -i "tty" | grep -o "ttyAC[A-Za-z0-9]*" | head -n 1)
if [ -z "$ARDUINO_PATH" ]; then
    echo "No Arduino found. Please connect the dimmer and try again."
    exit 1
fi
echo "Arduino found at $ARDUINO_PATH. Creating symlink..."

echo "Getting vendor and product ID..."
VENDOR_ID=$(lsusb | grep -i "Arduino" | awk '{print $6}' | cut -d':' -f1)
PRODUCT_ID=$(lsusb | grep -i "Arduino" | awk '{print $6}' | cut -d':' -f2)
if [ -z "$VENDOR_ID" ] || [ -z "$PRODUCT_ID" ]; then
    echo "Could not find vendor or product ID. Please check the connection."
    exit 1
fi
echo "Vendor ID: $VENDOR_ID"
echo "Product ID: $PRODUCT_ID"
echo "Creating udev rule..."
UDEV_RULE="/etc/udev/rules.d/99-arduino.rules"

echo "ATTRS{idVendor}==\"$VENDOR_ID\", ATTRS{idProduct}==\"$PRODUCT_ID\", SYMLINK+=\"light_dimmer\"" | sudo tee $UDEV_RULE > /dev/null

echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger
