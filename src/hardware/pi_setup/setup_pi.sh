#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/bin/bash

# This script should be run on a raspberry pi for first-time setup after running transfer_mace_to_pi.sh

printf "\nInstalling Python requirements\n"
pip install --no-index --find-links ~/mace/src/hardware/pi_setup/pi_dependencies -r ~/mace/src/hardware/pi_setup/pi_requirements.txt

printf "\nInstalling other dependencies\n"
sudo dpkg -i ~/mace/src/hardware/pi_setup/pi_dependencies/*.deb


sudo dpkg -i ~/mace/src/hardware/pi_setup/pi_dependencies/*.deb; printf "\nCopying service files to /etc/systemd/system\n"; sudo cp -a ~/mace/config/systemd/* /etc/systemd/system; sudo mkdir /opt/mace; sudo cp ~/mace/scripts/run_init_device.sh /opt/mace; printf "\nCopying mqtt config file to /etc/mosquitto/mosquitto.conf\n"; sudo cp ~/mace/config/mosquitto/payload_mosquitto_2023.conf /etc/mosquitto/mosquitto.conf



read -p "\nConfigure as Payload(p) or Target(t)? " payloadOrTarget

if [[ $payloadOrTarget = p* ]] || [[ $payloadOrTarget = P* ]]; then
    printf "Configuring pi as Payload"
    sudo systemctl enable mace-payload.service
    
elif [[ $payloadOrTarget = t* ]] || [[ $payloadOrTarget = T* ]]; then
    printf "Configuring pi as Target"
    sudo systemctl enable mace-target.service
else
    printf "[ERROR] Please enter a valid input (p/t)."
fi

# Set permanent BLE port
printf "\nSetting permanent BLE port\n"
~/mace/scripts/configure_ble.sh

# Ensure no extraneous directories are made on reboot
~/mace/src/hardware/pi_setup/clear_default_directories.sh

printf "\nSetup complete. Please reboot pi with <sudo reboot>\n"
