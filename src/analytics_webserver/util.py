#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import glob
import pandas

from typing import List
from typing import Tuple

from pandas.core.frame import DataFrame

def valid_position(position: Tuple[float, float, float]) -> bool:
    """
    Computes if a position is valid longitude latitude altitude position
    """
    # Position
    if position[0] < -90 or position[0] > 90:
        return False
    if position[1] < -90 or position[1] > 90:
        return False
    if position[2] < 0 or position[2] > 1000:
        return False
    return True

def load_data(log_folders: List[str]) -> Tuple[DataFrame, DataFrame]:
    """
    Load both the target and payload state information from the log files
    """
    # Create a list of target and payload datafiles
    target_dfs = []
    payload_dfs = []

    for log in log_folders:
        # Load the target file
        target_df = pandas.read_csv("{}/target_wf_output_state.csv".format(log))
        target_dfs.append(target_df)
        # Load the payload file
        payload_df = pandas.read_csv("{}/payload_output_state.csv".format(log))
        payload_dfs.append(payload_df)

        # If either file is empty, return None
        if (target_df.size <= 0) or (payload_df.size <= 0):
            return None

    # Return the target file
    return target_dfs, payload_dfs

def get_current_log(log_folder: str="../../logs/") -> str:
    """
    Get the most recent log file from all log files
    """
    log_files = get_all_logs(log_folder=log_folder)

    # Check if there are log files
    if log_files is not None:

        # Remove all log files with a tag, as these can not be the latest
        log_files_clean = []
        for log in log_files:
            log_name = log[log.rfind("/")+1:]
            if "_" not in log_name:
                log_files_clean.append(log)

        # If there are no clean log files return None
        if len(log_files_clean) <= 0:
            return None

        # Sort the clean log files
        log_files_clean = sorted(log_files_clean)

        # Return the latest
        current_log = log_files_clean[-1]

        return current_log
    return None

def get_all_logs(log_folder: str="../../logs/") -> str:
    """
    Get all log files in the log folder
    """
    log_files = glob.glob(log_folder + "*")
    if len(log_files) >= 1:
        return log_files
    return None

def get_colors() -> dict:
    colors = []
    colors.append("rgb(255, 99, 132)")
    colors.append("rgb(255, 159, 64)")
    colors.append("rgb(54, 162, 235)")
    colors.append("rgb(153, 102, 255)")
    colors.append("rgb(201, 203, 207)")
    colors.append("rgb(255, 205, 86)")
    colors.append("rgb(75, 192, 192)")

    colors_alpha = []
    for color in colors:
        new_color = color.replace("rgb(", "rgba(")
        new_color = new_color.replace(")", ", 0.05)")
        colors_alpha.append(new_color)

    color_dict = {}
    color_dict["rgb"] = colors
    color_dict["rgba"] = colors_alpha

    return color_dict

def get_unique_tags(file_names: List[str]) -> List[str]:
    result = []
    # Go through each file
    for name in file_names:

        # Get the tag
        tag = name[0:name.rfind("_")]

        # Check that there is a tag
        if len(tag) == len(name):
            continue
        
        # Save if this tag has never been seen before
        if tag not in result:
            result.append(tag)

    # If there are no tags
    if len(result) == 0:
        return None

    return result

def get_all_networks_captured(data: str) -> bool:
    all_networks_captured = True
    split_data = data.split(',')
    for d in split_data:
        d = d[d.rfind('=')+1:]
        if d.upper() != "TRUE":
            all_networks_captured = False
            break
    return all_networks_captured