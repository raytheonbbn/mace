#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import math
import datetime
import geopy.distance

import numpy as np

from typing import List
from typing import Tuple
from util import valid_position
from util import get_unique_tags
from util import get_all_networks_captured
from pandas.core.frame import DataFrame

# Used to convert the datetime from a string to a datetime object
INPUT_FORMAT = "%Y.%m.%d.%H.%M.%S"

def configuration_status(target_dfs_list: List[DataFrame], payload_dfs_list: List[DataFrame]) -> Tuple[List[int], List[int], List[int], List[int]]:
    """
    Get the configuration data: total number of targets, total number of payloads, number of air assets, and number of ground assets
    """

    # Check if we got not data
    if (len(target_dfs_list) <= 0) or (len(payload_dfs_list) <= 0):
        return [0], [0], [0], [0]

    # Declare all the values we want to return
    total_targets           = []
    total_payloads          = []
    total_airAssets         = []
    total_groundAssets      = []

    # For each of the target and payload dataframes
    for target_df, payload_df in zip(target_dfs_list, payload_dfs_list):

        # Get the unique target uids
        current_target_uids = target_df['uid'].unique()
        current_total_targets = len(current_target_uids)

        # Get a list of all payloads
        current_payload_uids = payload_df['uid'].unique()
        current_total_payloads = len(current_payload_uids)

        current_airAssets = 0
        current_groundAssets = 0
        for key in current_payload_uids:
            if "Quad" in key:
                current_airAssets += 1
            if "Rover" in key:
                current_groundAssets += 1

        # Save this information
        total_targets.append(current_total_targets)
        total_payloads.append(current_total_payloads)
        total_airAssets.append(current_airAssets)
        total_groundAssets.append(current_groundAssets)

    # Return the data
    return total_targets, total_payloads, total_airAssets, total_groundAssets

def get_mission_time(payload_dfs_list: List[DataFrame]) -> List[float]:
    """
    Get the mission time as and return as a list of times
    """

    # Check if we got not data
    if len(payload_dfs_list) <= 0:
        return [0]

    # Used to save the total time for each run
    total_time_per_run = []

    # Load the log data
    for payload_df in payload_dfs_list:

        # Determine when we started recording information
        for idx, row in payload_df.iterrows(): 
            # Wait until we start getting valid data
            cur_position = (row["latitude"], row["longitude"], row["altitude"])
            # When we find a valid position, break
            if valid_position(cur_position) == True:
                break

        # Get the start and end time stamps
        start_timestamp = payload_df.iloc[idx]["timestamp"]
        end_timestamp   = payload_df.iloc[-1]["timestamp"]

        # Get the start and end time as datetime objects
        start_timestamp = datetime.datetime.strptime(start_timestamp, INPUT_FORMAT)
        end_timestamp = datetime.datetime.strptime(end_timestamp, INPUT_FORMAT)

        # Compute and save the total mission time
        total_time = end_timestamp - start_timestamp
        total_time_per_run.append(total_time.total_seconds())

    # Return this
    return total_time_per_run

def get_mission_statistics(target_dfs_list: List[DataFrame]) -> Tuple[List[float], List[float], List[int], List[int], List[int], List[int]] :
    """
    Compute the time to capture, time to discover, and the total number of undiscovered, discovered and captured targets and return as a list of values
    """

    # Check if we got not data
    if len(target_dfs_list) <= 0:
        return [0], [0], [0], [0], [0], [0], [0]

    # Declare all the values we want to return
    times_to_capture_all            = []
    times_to_discover_all           = []
    total_undiscovered_targets      = []
    total_discovered_targets        = []
    total_captured_targeted         = []
    total_suppressed_targets        = []
    total_targets                   = []

    # Load the log data
    for target_df in target_dfs_list:

        # Used to compute the start time 
        cur_mission_start_time = None

        # Declare the current mission time to discover and capture
        cur_mission_time_to_discover    = None
        cur_mission_time_to_capture     = None
        
        # Declare this runs target statistics
        cur_mission_undiscovered_targets    = set()
        cur_mission_discovered_targets      = set()
        cur_mission_captured_targets        = set()
        cur_mission_suppressed_targets      = set()

        # Set all targets to undiscovered to start
        target_uids = target_df['uid'].unique()
        for key in target_uids:
            cur_mission_undiscovered_targets.add(key)

        # Save the total number of targets for this run
        cur_mission_total_targets = len(cur_mission_undiscovered_targets)

        # Iterate through the dataframe
        for idx, row in target_df.iterrows():

            # Extract the information from that row of the dataframe
            uid             = row["uid"]
            if row["type"] == "LINK":
                captured = get_all_networks_captured(row["networks_captured"])
            else:
                captured        = row["captured"]
            discovered      = row["discovered"]
            timestamp       = row["timestamp"]
            suppression     = row["suppression"]

            # Wait until we start getting valid data
            cur_position = (row["latitude"], row["longitude"], row["altitude"])
            if valid_position(cur_position) == False:
                continue

            # Convert the timestamp to a datetime
            timestamp = datetime.datetime.strptime(timestamp, INPUT_FORMAT)

            # Get the start_time
            if cur_mission_start_time is None:
                cur_mission_start_time = timestamp

            # Change the state of from undiscovered to discovered
            if (discovered == True) and (uid in cur_mission_undiscovered_targets):
                cur_mission_undiscovered_targets.remove(uid)
                cur_mission_discovered_targets.add(uid)

            # Change the state of from discovered to captured
            if (captured == True) and (uid in cur_mission_discovered_targets):
                cur_mission_discovered_targets.remove(uid)
                cur_mission_captured_targets.add(uid)

            # Check if captured targets are actually just suppressed
            if (suppression == True) and (uid in cur_mission_captured_targets):
                cur_mission_captured_targets.remove(uid)
                cur_mission_suppressed_targets.add(uid)
            
            # Remove any suppressed targets that are no longer suppresssed
            if (suppression == True) and (captured == False) and (uid in cur_mission_suppressed_targets):
                cur_mission_suppressed_targets.remove(uid)

            # Check for all discovered
            if (len(cur_mission_undiscovered_targets) == 0) and (cur_mission_time_to_discover is None):
                cur_mission_time_to_discover = (timestamp - cur_mission_start_time).total_seconds()

            # Check for all captured
            if (len(cur_mission_undiscovered_targets) == 0) and (len(cur_mission_discovered_targets) == 0) and (cur_mission_time_to_capture is None):
                cur_mission_time_to_capture = (timestamp - cur_mission_start_time).total_seconds()

                # We can break as we have all the information we need
                break

        # Save the computed metric
        times_to_capture_all.append(cur_mission_time_to_capture)
        times_to_discover_all.append(cur_mission_time_to_discover)
        total_undiscovered_targets.append(len(cur_mission_undiscovered_targets))
        total_discovered_targets.append(len(cur_mission_discovered_targets))
        total_captured_targeted.append(len(cur_mission_captured_targets))
        total_suppressed_targets.append(len(cur_mission_suppressed_targets))
        total_targets.append(cur_mission_total_targets)

    # Return 0 if this time has not yet been computed
    if times_to_capture_all[0] is None:
        times_to_capture_all = [0.0]
    if times_to_discover_all[0] is None:
        times_to_discover_all = [0.0]

    return times_to_capture_all, times_to_discover_all, total_undiscovered_targets, total_discovered_targets, total_captured_targeted, total_suppressed_targets, total_targets

def get_movement_statistics(payload_dfs_list: List[DataFrame]) -> Tuple[List[float], List[float], List[float]] :
    """
    Compute the movement time, dwelling time, and distance traveled
    """

    # Check if we got not data
    if len(payload_dfs_list) <= 0:
        return [0], [0], [0]

    # Declare all the values we want to return
    total_movement_times            = []
    total_dwelling_times            = []
    total_distance_traveled         = []

    # Load the log data
    for payload_df in payload_dfs_list:

        # Used to compute the start time 
        cur_mission_start_time = None

        # Declare the current mission movement, dwelling and distance traveled
        cur_mission_movement_time       = {}
        cur_mission_dwelling_time       = {}
        cur_mission_distance_traveled   = {}
        cur_mission_prev_location       = {}
        cur_mission_prev_timestamp      = {}

        # For each of the payloads, initialize each of the robots fields
        payload_uids = payload_df['uid'].unique()
        for key in payload_uids:
            cur_mission_movement_time[key]      = []
            cur_mission_dwelling_time[key]      = []
            cur_mission_distance_traveled[key]  = []
            cur_mission_prev_location[key]      = (15000, 15000, 15000)
            cur_mission_prev_timestamp[key]     = None
        

        # Iterate through the dataframe
        for idx, row in payload_df.iterrows():

            # Extract the information from that row of the dataframe
            current_uid         = row["uid"]
            current_position    = (row["latitude"], row["longitude"], row["altitude"])
            current_timestamp   = row["timestamp"]

            # Wait until we start getting valid data
            if valid_position(current_position) == False:
                continue

            # Convert the timestamp to a datetime
            current_timestamp = datetime.datetime.strptime(current_timestamp, INPUT_FORMAT)

            # If this is our first valid reading handle it differently
            if cur_mission_prev_timestamp[current_uid] is None:
                cur_mission_prev_location[current_uid]  = current_position
                cur_mission_prev_timestamp[current_uid] = current_timestamp
                continue

            # Get the last recorded position and the position from the new row
            p1 = (cur_mission_prev_location[current_uid][0], cur_mission_prev_location[current_uid][1])
            p2 = (current_position[0], current_position[1])
            # Compute the distance ignoring altitude
            horizontal_dist = geopy.distance.distance(p1, p2).meters
            # Compute the change in height
            delta_altitude = abs(cur_mission_prev_location[current_uid][2] - current_position[2])
            # Compute distance for the hypotenuse
            true_dist = math.sqrt(math.pow(horizontal_dist, 2) + math.pow(delta_altitude, 2))
            # Save the distance 
            cur_mission_distance_traveled[current_uid].append(true_dist)
            
            # We should also have a valid timestep by now
            dt = current_timestamp - cur_mission_prev_timestamp[current_uid]

            # Check if we were moving at this time or dwelling
            if true_dist <= 0.001:
                cur_mission_dwelling_time[current_uid].append(dt.total_seconds())
            else:
                cur_mission_movement_time[current_uid].append(dt.total_seconds())

            # Update the previous information
            cur_mission_prev_location[current_uid]  = current_position
            cur_mission_prev_timestamp[current_uid] = current_timestamp

        # Compute the total per uid
        for key in cur_mission_distance_traveled:
            # Add up all the distances before returning
            cur_mission_distance_traveled[key]  = np.sum(cur_mission_distance_traveled[key])
            cur_mission_dwelling_time[key]      = np.sum(cur_mission_dwelling_time[key])
            cur_mission_movement_time[key]      = np.sum(cur_mission_movement_time[key])

        # Get the total number of rovers
        total_payloads = len(cur_mission_dwelling_time.keys())

        # Compute the mission totals
        total_distance = sum(cur_mission_distance_traveled[key] for key in cur_mission_distance_traveled)
        total_dwelling_time = sum(cur_mission_dwelling_time[key] for key in cur_mission_dwelling_time) / total_payloads
        total_moving_time = sum(cur_mission_movement_time[key] for key in cur_mission_movement_time) / total_payloads

        # Save the totals
        total_movement_times.append(total_moving_time)
        total_dwelling_times.append(total_dwelling_time)
        total_distance_traveled.append(total_distance)

    # Return the data
    return total_movement_times, total_dwelling_times, total_distance_traveled

def get_individual_target_statistics(target_dfs_list: List[DataFrame]) -> Tuple[List[str], List[List[float]], List[List[float]], List[List[float]], List[List[float]]] :
    """
    Returns the target names, and their min and max capture and discover times.
    """

    # Check if we got not data
    if len(target_dfs_list) <= 0:
        return ["na"], [[0, 0]], [[0, 0]], [[0, 0]], [[0, 0]]

    # Declare all the values we want to return
    target_names                    = []
    target_time_to_discover_range   = {}
    target_time_to_capture_range    = {}

    # Load the log data
    for target_df in target_dfs_list:

        # Used to compute the start time 
        cur_mission_start_time = None

        # Used to compute the start time 
        cur_mission_target_discover_times = {}
        cur_mission_target_capture_times = {}

        # Get the list of target names
        target_uids = target_df['uid'].unique()
        for key in target_uids:
            cur_mission_target_discover_times[key] = None
            cur_mission_target_capture_times[key] = None

        # Loop through the target dataframe and extract each of the capture and discovery times
        for idx, row in target_df.iterrows():

            # Load the information we need
            uid             = row["uid"]
            if row["type"] == "LINK":
                captured = get_all_networks_captured(row["networks_captured"])
            else:
                captured        = row["captured"]
            discovered      = row["discovered"]
            timestamp       = row["timestamp"]

            # If we are recording junk ignore
            if np.isnan(captured) or np.isnan(discovered):
                continue

            # Wait until we start getting valid data
            cur_position = (row["latitude"], row["longitude"], row["altitude"])
            if valid_position(cur_position) == False:
                continue

            # Convert the timestamp to a datetime
            timestamp = datetime.datetime.strptime(timestamp, INPUT_FORMAT)

            # Get the start_time
            if cur_mission_start_time is None:
                cur_mission_start_time = timestamp

            # Save the discovered time
            if discovered and cur_mission_target_discover_times[uid] is None:
                discover_time = timestamp - cur_mission_start_time
                cur_mission_target_discover_times[uid] = discover_time.total_seconds()

            # Save the capture time
            if captured and cur_mission_target_capture_times[uid] is None:
                capture_time = timestamp - cur_mission_start_time
                cur_mission_target_capture_times[uid] = capture_time.total_seconds()

        # Save the information for that run
        for uid in target_uids:
            # Check if that target name exists
            if uid not in target_names:
                target_names.append(uid)
                target_time_to_discover_range[uid] = []
                target_time_to_capture_range[uid] = []

            # Add the information to the respective target file
            target_time_to_discover_range[uid].append(cur_mission_target_discover_times[uid])
            target_time_to_capture_range[uid].append(cur_mission_target_capture_times[uid])

    # Return the min and max values
    target_time_to_discover_range_return = []
    target_time_to_capture_range_return = []  

    # Return the average values
    average_discover_time_return = []
    average_capture_time_return = []

    for uid in target_names:

        # Convert the capture and discovery times to numpy arrays
        # This converts all None values to NaN
        # None's occur when there was no capture or discovery
        dis = np.array(target_time_to_discover_range[uid], dtype=np.float64)
        cap = np.array(target_time_to_capture_range[uid], dtype=np.float64)

        # Check for all nan values
        if np.isnan(dis).all():
            target_time_to_discover_range_return.append([None, None])
            average_discover_time_return.append(None)
        else: 
            # Save the min and max
            target_time_to_discover_range_return.append([np.nanmin(dis), np.nanmax(dis)])
            average_discover_time_return.append(np.mean(dis))

        if np.isnan(cap).all():
            target_time_to_capture_range_return.append([None, None])
            average_capture_time_return.append(None)
        else:
            # Save the min and max
            target_time_to_capture_range_return.append([np.nanmin(cap), np.nanmax(cap)])
            average_capture_time_return.append(np.mean(cap))
    
    # Return the data
    return target_names, target_time_to_discover_range_return, target_time_to_capture_range_return, average_discover_time_return, average_capture_time_return

def get_tag_data(requested_files, all_times_to_capture, all_times_to_discover, all_mission_times, all_total_distance_traveled, all_total_dwelling_times, all_total_movement_times):

    if len(requested_files) == 0:
        return ["na"], [[0,0,0,0,0,0]]

    # Get the unique tags
    unique_tags = get_unique_tags(requested_files)

    # Create the tag data
    tag_analysis = {}
    for key in unique_tags:
        tag_analysis[key] = []

    # Get the min and max value
    min_array = np.full(5, np.inf)
    max_array = np.full(5, -np.inf)

    # Save the tag data
    for i in range(len(requested_files)):
        log_name = requested_files[i]
        cap_time = all_times_to_capture[i]
        dis_time = all_times_to_discover[i]
        mis_time = all_mission_times[i]
        trvl_dis = all_total_distance_traveled[i]
        dwl_time = all_total_dwelling_times[i]
        mvt_time = all_total_movement_times[i]

        # Get the data 
        # data = [mis_time, mvt_time, dwl_time, trvl_dis, dis_time, cap_time]
        data = [mis_time, mvt_time, trvl_dis, dis_time, cap_time]

        # Keep tack of the min and max
        min_array = np.min(np.stack([np.array(data),min_array]), axis=0)
        max_array = np.max(np.stack([np.array(data),max_array]), axis=0)

        # Get the tag for this file
        tag = get_unique_tags([log_name])
        
        # If there is no tag
        if tag is None:
            continue

        assert(len(tag) == 1)
        tag = tag[0]

        # Save the tag data
        tag_analysis[tag].append(data)
    
    # Compute the average for each tag
    for tag in tag_analysis:
        data = np.array(tag_analysis[tag])
        if len(np.shape(data)) > 1:
            average_data = list(np.mean(data, axis=0))
        else:
            average_data = list(data)
        tag_analysis[tag] = average_data

    tag_labels = [tag for tag in tag_analysis]
    tag_data = [tag_analysis[tag] for tag in tag_analysis]

    # Normalize the tag data
    converted_tag_data = np.array(tag_data)
    converted_tag_data =  np.round((converted_tag_data - min_array) / (max_array - min_array),4)

    # Invert so that smaller is worse
    converted_tag_data = 1 - converted_tag_data

    # Convert to a list
    tag_data = [list(data) for data in converted_tag_data]

    print(tag_data)

    return tag_labels, tag_data