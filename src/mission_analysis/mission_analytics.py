#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import glob
import datetime
import math
import pandas
import geopy.distance

import numpy as np




# Computes if a position is valid
def valid_position(position):
    # Position
    if position[0] < -90 or position[0] > 90:
        return False
    if position[1] < -90 or position[1] > 90:
        return False
    if position[2] < 0 or position[2] > 1000:
        return False
    return True

# Computes the total distance travelled dwelling time and time travelled
def payload_movement_statistics(payload_df):

    # Get a list of all payloads
    payload_uids = payload_df['uid'].unique()

    # Initilize the distance and current location dictionary
    distances = {}
    dwellingtime = {}
    movingtime = {}
    current_location = {}
    current_timestamp = {}
    
    for key in payload_uids:
        distances[key] = []
        dwellingtime[key] = []
        movingtime[key] = []

        current_location[key] = (15000, 15000, 15000)
        current_timestamp[key] = None


    # Go through the data and compute how far each of the payloads travelled
    for idx, row in payload_df.iterrows():
        # Get the information we are interest in
        uid = row["uid"]
        pos = (row["latitude"], row["longitude"], row["altitude"])
        timestamp = row["timestamp"]
        input_format = "%Y.%m.%d.%H.%M.%S"
        timestamp = datetime.datetime.strptime(timestamp, input_format)

        # Check if there is already a valid position (which is the previous position)
        if valid_position(current_location[uid]):
            # Get the positions
            p1 = (current_location[uid][0], current_location[uid][1])
            p2 = (pos[0], pos[1])
            # Compute the distance ignoring altitude
            horizontal_dist = geopy.distance.distance(p1, p2).meters
            # Compute the change in height
            delta_alitude = abs(current_location[uid][2] - pos[2])
            # Compute distance for the hypotenuse
            true_dist = math.sqrt(math.pow(horizontal_dist, 2) + math.pow(delta_alitude, 2))
            # Save the distance 
            distances[uid].append(true_dist)

            # We should also have a valid timestep by now
            dt = timestamp - current_timestamp[uid]

            # Check if we were moving at this time or dwelling
            if true_dist <= 0.001:
                dwellingtime[uid].append(dt.total_seconds())
            else:
                movingtime[uid].append(dt.total_seconds())


        # Update the last known position of the robot
        if valid_position(pos):
            # Update the position
            current_location[uid] = pos
            # Update the timestep
            current_timestamp[uid] = timestamp

    for key in distances:
        # Add up all the distances before returning
        distances[key] = np.sum(distances[key])
        dwellingtime[key] = np.sum(dwellingtime[key])
        movingtime[key] = np.sum(movingtime[key])

    return distances, dwellingtime, movingtime

# Load the data from the log file
def load_data(log_folder):
    # Load the target file
    target_df = pandas.read_csv("{}/target_input_state.csv".format(log_folder))
    # Load the payload file
    payload_df = pandas.read_csv("{}/payload_output_state.csv".format(log_folder))

    # Return the target file
    return target_df, payload_df

# Get the current log file
def get_current_log():
    log_files = glob.glob("../../logs/*")
    log_files = sorted(log_files)
    current_log = log_files[-1]
    return current_log

# Define the target statistics
def target_statistics(target_df):
    # Get the unique target uids
    target_uids = target_df['uid'].unique()

    # Init the variables
    starttime = None
    timetocapture = None
    timetodiscover = None
    undiscoverlist = [key for key in target_uids]
    discoverlist = []
    capturelist = []

    # Go through the data
    for idx, row in target_df.iterrows():
        uid = row["uid"]
        captured = row["captured"]
        timestamp = row["timestamp"]
        input_format = "%Y.%m.%d.%H.%M.%S"
        timestamp = datetime.datetime.strptime(timestamp, input_format)

        # Get the starttime
        if starttime is None:
            pos = (row["latitude"], row["longitude"], row["altitude"])
            if valid_position(pos):
                starttime = timestamp

        # Change the state of from undiscovered to captured
        if (captured == True) and (uid not in capturelist):
            undiscoverlist.remove(uid)
            capturelist.append(uid)

        # Save the time to discover
        if (len(undiscoverlist) == 0) and (timetodiscover is None):
            timetodiscover = (timestamp - starttime).total_seconds()

        # Save the time to discover
        if (len(undiscoverlist) == 0) and (len(discoverlist) == 0) and (timetocapture is None):
            timetocapture = (timestamp - starttime).total_seconds()

    # Compute the mission time
    mission_time = (timestamp - starttime).total_seconds()


    return timetocapture, timetodiscover, mission_time, len(undiscoverlist), len(discoverlist), len(capturelist)

# Define the target statistics
def configuration_status(target_df, payload_df):
    # Get the unique target uids
    target_uids = target_df['uid'].unique()
    total_targets = len(target_uids)

    # Get a list of all payloads
    payload_uids = payload_df['uid'].unique()
    total_payloads = len(payload_uids)

    airAssets = 0
    groundAssets = 0
    for key in payload_uids:
        if "Quad" in key:
            airAssets += 1
        if "Rover" in key:
            groundAssets += 1


    return total_targets, total_payloads, airAssets, groundAssets


def main():

    # Find the correct log file
    log_file = get_current_log()

    # Load the target file and print it out
    target_df, payload_df = load_data(log_file)

    # Compute the total distance travelled, dwelling time and moving time
    distances, dwellingtime, movingtime = payload_movement_statistics(payload_df)

    # Compute the time to capture, time to discover, undiscover, discover, and capture count
    timetocapture, timetodiscover, mission_time, undiscovercount, discovercount, capturecount = target_statistics(target_df) 

    # Get the total targets and payloads
    total_targets, total_payloads, airAssets, groundAssets = configuration_status(target_df, payload_df)

    # Print the payload statitics
    print("Mission statistics:")
    print("Total Mission time: {}".format(datetime.timedelta(seconds=mission_time)))
    print("Total payloads: {}".format(total_payloads))
    print("Total targets: {}".format(total_targets))
    print("Total air assets: {}".format(airAssets))
    print("Total ground assets: {}".format(groundAssets))
    print("")
    print("Payload statistics:")
    for key in distances:
        print("{} -- Distance Travelled: {}".format(key, distances[key]))
        print("{} -- Dwelling time: {}".format(key, datetime.timedelta(seconds=dwellingtime[key])))
        print("{} -- Moving time: {}".format(key, datetime.timedelta(seconds=movingtime[key])))
    # Compute the total distance, moving time and dewlling time
    total_distance = sum(distances[key] for key in distances)
    total_dwellingtime = sum(dwellingtime[key] for key in dwellingtime)
    total_movingtime = sum(movingtime[key] for key in movingtime)
    print("")
    print("Total distance: {}".format(total_distance))
    print("Total dwelling time: {}".format(datetime.timedelta(seconds=total_dwellingtime)))
    print("Total moving time: {}".format(datetime.timedelta(seconds=total_movingtime)))
    print("")
    print("Target Statistics")
    print("Targets undiscovered: {}".format(undiscovercount))
    print("Targets discovercount: {}".format(discovercount))
    print("Targets capturecount: {}".format(capturecount))
    print("Time to discover all: {}".format(datetime.timedelta(seconds=timetodiscover)))
    print("Time to capture all: {}".format(datetime.timedelta(seconds=timetocapture)))


if __name__ == '__main__':
    main()
