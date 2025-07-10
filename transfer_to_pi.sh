#!/bin/bash

# WaveShare Robotaxi - Transfer to Pi Script
# This script helps transfer the ugv_rpi folder to your Raspberry Pi

echo "WaveShare Robotaxi - Transfer to Pi"
echo "=================================="

# Check if ugv_rpi directory exists
if [ ! -d "ugv_rpi" ]; then
    echo "❌ Error: ugv_rpi directory not found!"
    echo "Make sure you're in the WaveShare directory."
    exit 1
fi

# Get Pi IP address
echo "Enter your Raspberry Pi's IP address:"
read PI_IP

if [ -z "$PI_IP" ]; then
    echo "❌ Error: IP address is required!"
    exit 1
fi

# Test connection to Pi
echo "Testing connection to Pi at $PI_IP..."
if ! ping -c 1 -W 3 "$PI_IP" > /dev/null 2>&1; then
    echo "❌ Error: Cannot reach Pi at $PI_IP"
    echo "Make sure:"
    echo "1. Your Pi is powered on and connected to the same network"
    echo "2. SSH is enabled on the Pi"
    echo "3. The IP address is correct"
    exit 1
fi

echo "✅ Pi is reachable!"

# Transfer the ugv_rpi folder
echo "Transferring ugv_rpi folder to Pi..."
echo "This may take a few minutes..."

if scp -r ugv_rpi pi@"$PI_IP":~/; then
    echo "✅ Transfer successful!"
    echo ""
    echo "Next steps on your Pi:"
    echo "1. SSH into your Pi: ssh pi@$PI_IP"
    echo "2. Navigate to ugv_rpi: cd ugv_rpi"
    echo "3. Run setup: sudo chmod +x setup.sh && sudo ./setup.sh"
    echo "4. Configure auto-start: sudo chmod +x autorun.sh && ./autorun.sh"
    echo "5. Reboot: sudo reboot"
    echo "6. Access web interface: http://$PI_IP:5000"
else
    echo "❌ Transfer failed!"
    echo "Make sure:"
    echo "1. SSH is enabled on the Pi"
    echo "2. You have the correct password (default: raspberry)"
    echo "3. There's enough space on the Pi"
fi 