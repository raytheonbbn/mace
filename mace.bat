::  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

::  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



@ECHO off
setlocal enabledelayedexpansion

:: The script is for starting MACE on windows. 
:: It starts the analytics server and STOMP after getting user input 
:: After running, provide desired numbers of each sim entity (Quads, Rovers, Targets)
:: To run in white force mode, run with a "-w" flag, e.g. "mace.bat -w"

:: Prompt user for number of sim entities, default to 0
:get_num_sim_objects
    set /P NUM_QUADS=How many sim QUADS:
    set /P NUM_ROVERS=How many sim ROVERS: 
    set /P NUM_TARGETS=How many sim TARGETS:
    :: If no user input, default to zero
    IF [%NUM_QUADS%]==[]  set NUM_QUADS=0
    IF [%NUM_ROVERS%]==[]  set NUM_ROVERS=0
    IF [%NUM_TARGETS%]==[]  set NUM_TARGETS=0

:: Remake docker-compose.yaml file with user selected number of sim entities
:set_num_sim_objects
    set InputFile=docker-compose.yaml
    set OutputFile=temp.txt 
    set "_strInsert=    command: ["%NUM_QUADS%", "%NUM_ROVERS%", "%NUM_TARGETS%"]"
    >"%OutputFile%" (
        for /f "delims= tokens=*" %%a in (%InputFile%) do (
            set line=%%a
            if "!line:~0,12!"=="    command:" (
                echo %_strInsert%
            ) else (
                echo !line!
            )
        )
    )
    del %InputFile%
    rename %OutputFile% %InputFile%

:set_forcetype
    :: If argument "-w" is present, set forceType to white, else blue
    if [%1]==[-w] (
        set "_strInsert=forceType=white"
    ) else ( 
        set "_strInsert=forceType=blue"
    )
    :: Remake localDevice.properties line by line and set type
    set InputFile=localDevice.properties
    set OutputFile=temp.txt
    cd code\stomp
    >"%OutputFile%" (
        for /f "delims= tokens=*" %%a in (%InputFile%) do (
            set line=%%a
            if "!line:~0,10!"=="forceType=" (
                echo %_strInsert%
            ) else (
                echo %%a
            )
        )
    )
    :: Delete the original file, and replace it with the one we just made
    del %InputFile%
    rename %OutputFile% %InputFile%
    :: Because batch is dumb, we need to go over the file again and add in the "!" (see the ^^ escape before ! in the mqttPassword)
    >"%OutputFile%" (
        for /f "delims= tokens=*" %%a in (%InputFile%) do (
            set line=%%a
            if "!line:~0,12!"=="mqttPassword" (
                echo mqttPassword=maceRef^^!
            ) else (
                echo %%a
            )
        )
    )
    del %InputFile%
    rename %OutputFile% %InputFile%
    :: Return to top level dir
    cd ..\..

:start_mace_and_stomp 
    echo Starting MACE and STOMP
    :: Start up Analytics Server
    START docker compose up

    :: Start STOMP
    cd code\stomp
    START start.bat

    :: Return to top level dir
    cd ..\..
