#!/bin/bash

# WaveShare Robotaxi Status Check Script

echo "=== WaveShare Robotaxi Status ==="
echo

# Check if service is enabled and running
echo "Service Status:"
if systemctl is-enabled robotaxi.service >/dev/null 2>&1; then
    echo "✓ Service is enabled (will start on boot)"
else
    echo "✗ Service is NOT enabled (won't start on boot)"
fi

if systemctl is-active robotaxi.service >/dev/null 2>&1; then
    echo "✓ Service is running"
else
    echo "✗ Service is NOT running"
fi

echo

# Show recent logs
echo "Recent Logs (last 20 lines):"
echo "================================"
if [ -d "/home/pi/WaveShare/logs" ]; then
    LATEST_LOG=$(ls -t /home/pi/WaveShare/logs/robotaxi_*.log 2>/dev/null | head -1)
    if [ -n "$LATEST_LOG" ]; then
        tail -20 "$LATEST_LOG"
    else
        echo "No log files found"
    fi
else
    echo "Log directory not found"
fi

echo

# Show service logs
echo "System Service Logs (last 10 lines):"
echo "===================================="
journalctl -u robotaxi.service -n 10 --no-pager

echo

# Show system resources
echo "System Resources:"
echo "================="
echo "CPU Usage: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%"
echo "Memory Usage: $(free | grep Mem | awk '{printf("%.1f%%", $3/$2 * 100.0)}')"
echo "Disk Usage: $(df -h / | tail -1 | awk '{print $5}')"

echo

# Show network status
echo "Network Status:"
echo "==============="
if ping -c 1 raspberrypi.local >/dev/null 2>&1; then
    echo "✓ Pi is reachable via network"
    echo "IP Address: $(hostname -I | awk '{print $1}')"
else
    echo "✗ Pi is not reachable via network"
fi 