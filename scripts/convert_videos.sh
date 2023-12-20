#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

#!/bin/bash

# This script converts all files in DIR to .webm format

DIR=../src/analytics_webserver/static/videos/

for MP4 in "$DIR"*
    do 
        # VIDEONAME is the name of hte video minus the last 4 characters (for removing ".mp4")
        VIDEONAME=$(echo ${MP4:0:-4})
        echo "Converting ""$MP4"" to "$VIDEONAME".webm"
        # Open a new termial tab and convert a video in the directory to .webm (the variables are passed into the new tab via $1, $2 etc)
        gnome-terminal --tab -- bash -c 'ffmpeg -i "$1" $2".webm"; exec bash' bash "$MP4" "$VIDEONAME"

    done