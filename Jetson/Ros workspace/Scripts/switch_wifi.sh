#!/bin/bash

echo "Switch Wi-Fi script ran at $(date)" >> /home/tera/ros2_ws2/Scripts/wifi_run.log

# Define the SSID of the network you're connected to
target_ssid="Clevon"

# Scan all available Wi-Fi networks and filter for those with the target SSID
strongest_ap=$(nmcli -f SSID,BSSID,SIGNAL dev wifi | grep "$target_ssid" | sort -r -k3 | head -n 1)

if [ -n "$strongest_ap" ]; then
    # Extract the BSSID and signal strength of the strongest access point
    best_bssid=$(echo $strongest_ap | awk '{print $2}')
    strongest_signal=$(echo $strongest_ap | awk '{print $3}')

    # Connect to the strongest BSSID (access point) of the SSID
    nmcli dev wifi connect "$target_ssid" bssid "$best_bssid"
    echo "Connected to the strongest AP: $best_bssid with signal strength: $strongest_signal"
else
    echo "No access points found for the SSID: $target_ssid"
fi


