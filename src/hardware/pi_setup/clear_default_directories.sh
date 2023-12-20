#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  


# This script deletes default diretories (Desktop, Documents, etc) and prevents them from being created again

rmdir ~/Desktop/ ~/Documents/ ~/Downloads/ ~/Music/ ~/Pictures/ ~/Public/ ~/Templates/ ~/Videos/ ~/Bookshelf/ > /dev/null 2>&1
 
rm ~/.config/user-dirs.dirs
printf 'XDG_DESKTOP_DIR="$HOME"\nXDG_DOWNLOAD_DIR="$HOME"\nXDG_TEMPLATES_DIR="$HOME"\nXDG_PUBLICSHARE_DIR="$HOME"\nXDG_DOCUMENTS_DIR="$HOME"\nXDG_MUSIC_DIR="$HOME"\nXDG_PICTURES_DIR="$HOME"\nXDG_VIDEOS_DIR="$HOME"' > ~/.config/user-dirs.dirs    


