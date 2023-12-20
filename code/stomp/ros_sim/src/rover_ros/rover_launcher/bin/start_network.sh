#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  


CCAST_CONFIG_FILE="/etc/CCAST_CONFIG"
CCAST_AVAHI_SERVICE_CONFIG="/etc/avahi/services/ccast.service"

set_hostname() {
  source "/etc/CCAST_CONFIG" 2> /dev/null
  # Set the hostname from the MAC address
  auto_hostname_device="${CCAST_WIFI_INTERFACE:-eth0}"
  mac="$(ip link show $auto_hostname_device | grep ether | awk '{print $2}' | tr -d ':')"
  [ -z "$mac" ] && exit 1
  hostname="tx2-${mac:6:6}"
  echo "$hostname" > /etc/hostname
  sed -i "/$hostname/d" "/etc/hosts" 
  grep "$hostname" /etc/hosts > /dev/null 2>&1 || echo "127.0.1.1 $hostname" >> /etc/hosts
  hostnamectl set-hostname $hostname
}

launch_auto_hostname() {
  (while true; do set_hostname && break; sleep 1; done) &
}


launch_avahi_service() {
  source "/etc/CCAST_CONFIG"
    local device_info=""
    # There's (apparently) a 255 character-limit on the text line in the configuration file...
    while IFS='' read -r line; do
	device_info="$device_info $(echo "$line" | sed -e "s/#.*//")"
    done < "$CCAST_CONFIG_FILE"
    device_info="$(echo "$device_info" | cut -c1-255)"
    
    cat <<EOF > "$CCAST_AVAHI_SERVICE_CONFIG"
<?xml version="1.0" standalone='no'?><!--*-nxml-*-->
<!DOCTYPE service-group SYSTEM "avahi-service.dtd">

<service-group>
  <name replace-wildcards="yes">%h</name>
  <service>
    <type>_ssh._tcp</type>
    <port>22</port>
    <txt-record>model=Xserve</txt-record>
  </service>
  <service>
    <type>_device-info._udp</type>
    <port>1234</port>
    <txt-record>CCAST_VEHICLE_TYPE="$CCAST_VEHICLE_TYPE" </txt-record>
    <txt-record>CCAST_ROVER_HAS_ARDUCAM="$CCAST_ROVER_HAS_ARDUCAM"</txt-record>
    <txt-record>CCAST_ROVER_CPU="$CCAST_ROVER_CPU" </txt-record>
    <txt-record>CCAST_WIFI_MODE="$CCAST_WIFI_MODE" </txt-record>
    <txt-record>CCAST_WIFI_INTERFACE="$CCAST_WIFI_INTERFACE"</txt-record>
    <txt-record>CCAST_WIFI_CHANNEL="$CCAST_WIFI_CHANNEL"</txt-record>
    <txt-record>CCAST_ADHOC_KEY="$CCAST_ADHOC_KEY"</txt-record>
    <txt-record>CCAST_ADHOC_ESSID="$CCAST_ADHOC_ESSID"</txt-record>
    <txt-record>CCAST_ADHOC_BSSID="$CCAST_ADHOC_BSSID"</txt-record>
    <txt-record>CCAST_INFRASTRUCTURE_INET=$CCAST_INFRASTRUCTURE_INET" </txt-record>
    <txt-record>CCAST_INFRASTRUCTURE_SSID="$CCAST_INFRASTRUCTURE_SSID"</txt-record>
    <txt-record>CCAST_INFRASTRUCTURE_PSK="$CCAST_INFRASTRUCTURE_PSK"</txt-record>
  </service>
EOF

    if [ "x${CCAST_VEHICLE_TYPE,,}" = "xrover" ] &&
	   [ "x${CCAST_ROVER_CPU,,}" = "xpi2" ]; then
	cat <<EOF >>"$CCAST_AVAHI_SERVICE_CONFIG"
  <service>
    <type>_protelis._udp</type>
    <port>$PROTELIS_PORT</port>
  </service>
EOF
    fi

    cat <<EOF >>"$CCAST_AVAHI_SERVICE_CONFIG"
</service-group>
EOF

    # We'll manually manage the service - make it load our new configuration
    systemctl restart avahi-daemon
}

launch_auto_hostname

sleep 2

launch_avahi_service
