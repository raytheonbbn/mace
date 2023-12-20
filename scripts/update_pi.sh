#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  


## A script that uses a command line arg for the IP address and the default pi password to update with latest mace files

# # Use rsync to sync the Pi's MACE directory contents
# # Change the argument following "-p" tp change the password
sshpass -p 'mace2021' rsync -avzh -e ssh ../ pi@$1:~/mace/ --exclude-from=exclude.txt

