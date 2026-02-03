#!/bin/bash
# E-Wolf Quick Update Script
# First give permissions with chmod +x update.sh
# Then execute the script dentro de Brain ./update.sh

set -e # Exit immediately if a command fails

echo "Fetching latest code from GitHub..."
git pull origin main

echo "Cleaning up existing Python processes..."
# Ensure we use the correct path to your kill script
if [ -f "./services/brain-autostart/kill-brain.sh" ]; then
    ./services/brain-autostart/kill-brain.sh
fi

echo "Restarting system services..."
sudo systemctl restart angular-dashboard.service
sudo systemctl restart brain-monitor.service

echo "Verifying System Status..."
# Check if the monitor is active and show the last 5 logs
sudo systemctl status brain-monitor.service --no-pager -n 5

# Show active network connection
nmcli -t -f DEVICE,CONNECTION device | grep "^wlan0" || echo "Check Wi-Fi connection!"

echo "--- ALL SET ---"
