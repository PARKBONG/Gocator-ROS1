#!/bin/bash
# This script automatically assign IP on network interface.

IFACE="enx00e04c680015"
IP="192.168.1.100/24"
SENSOR_IP="192.168.1.10"

echo " ======================================================================"
echo "Setting up network interface $IFACE with IP $IP for Gocator sensor at $SENSOR_IP"

# Function to show network interface info
show_interface_info() {
    echo "Current network interface status:"
    ifconfig $IFACE 2>/dev/null || echo "Interface $IFACE not found"
    echo ""
}

# Function to remove existing IP addresses from interface
remove_existing_ips() {
    echo "Removing existing IP addresses from $IFACE..."
    # Get all IP addresses on the interface and remove them
    ip addr show $IFACE | grep 'inet ' | awk '{print $2}' | while read ip; do
        if [ ! -z "$ip" ]; then
            echo "Removing IP: $ip"
            sudo ip addr del $ip dev $IFACE 2>/dev/null || true
        fi
    done
    echo "Existing IPs removed."
    echo ""
}

# Function to add new IP
add_new_ip() {
    echo "Adding new IP address: $IP to interface: $IFACE"
    sudo ip addr add $IP dev $IFACE
    if [ $? -eq 0 ]; then
        echo "IP address successfully added."
    else
        echo "Failed to add IP address."
    fi
    echo ""
}

# Function to test connectivity
test_connectivity() {
    SENSOR_IP=$SENSOR_IP
    echo "Testing connectivity to sensor at $SENSOR_IP..."
    ping -c 3 $SENSOR_IP
    echo ""
}

# Main execution
echo "=== Gocator Network Setup Script ==="
echo ""

# Show current status
show_interface_info

# Remove existing IPs
remove_existing_ips

# Add new IP
add_new_ip

# Show updated status
echo "Updated network interface status:"
show_interface_info

# Test connectivity
test_connectivity

echo " ======================================================================"
