#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  


# Get the IP address of the pi
read -p "[INPUT] Please enter the IP address of the Raspberry Pi that you wish to update: " rpi_addr 

# Use rsync to sync the Pi's MACE directory contents
rsync -avzh ../ pi@$rpi_addr:~/mace/ --exclude-from=exclude.txt
# scp ../src/hardware/pi_dependencies/setup_pi.sh pi@$rpi_addr:~/

while true; do

    read -p "[INPUT] Would you like to update another device? (y/n): " update_another_device

    if [ $update_another_device == "y" ]; then
        read -p "[INPUT] Please enter the IP address of the Raspberry Pi that you wish to update: " rpi_addr 
        rsync -avzh ../ pi@$rpi_addr:~/mace/ --exclude-from=exclude.txt
        # scp ../src/hardware/pi_dependencies/setup_pi.sh pi@$rpi_addr:~/
    elif [ $update_another_device == "n" ]; then
        break
    else
        echo "[ERROR] Please enter a valid input (y/n)."
    fi

done


echo "[INFO] Exiting."