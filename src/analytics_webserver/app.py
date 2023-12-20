#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import os
import glob
import datetime
from flask import Flask, render_template, send_from_directory, request

import numpy as np

from util import load_data
from util import get_colors
from util import get_all_logs
from util import valid_position
from util import get_current_log

from mission_analytics import get_tag_data
from mission_analytics import get_mission_time
from mission_analytics import configuration_status
from mission_analytics import get_mission_statistics
from mission_analytics import get_movement_statistics
from mission_analytics import get_individual_target_statistics

app = Flask(__name__)

# Declare the refresh rate
REFRESH_RATE = 1
# Scatter plot point radius
POINT_RADIUS = 8

@app.route('/')
def index():
    """
    Computes the all statistics and returns the index template with all required statistics 
    """
    return render_template('index.html')     

@app.route('/about')
def about():
    """
    Shows the about page
    """
    return render_template('about.html')     

@app.route('/tutorial')
def tutorial():
    """
    Shows the tutorial page
    """
    return render_template('tutorial.html')     
    
@app.route('/latest_run')
def show_single_run():
    """
    Located the latest log file and displays all statistics on that current log file to the /latest_run webpage
    """

    # Find the correct log file
    log_file = get_current_log()

    # If the log file is none, wait for log files
    if log_file is None:
        return render_template('loading.html', refresh_rate=REFRESH_RATE)

    # Load the target file
    data = load_data([log_file])

    # Check that we have data
    if data is None:
        return render_template('loading.html', refresh_rate=REFRESH_RATE)

    # Get the data
    target_dfs  = data[0]
    payload_dfs = data[1]

    # Make sure we are getting the right data
    assert(len(target_dfs) == 1)
    assert(len(payload_dfs) == 1)

    # Get the moving time
    results = get_mission_time(payload_dfs)
    mission_time                = results[0]
    
    # Get the mission statistics
    results = get_mission_statistics(target_dfs)
    times_to_capture            = results[0][0]
    times_to_discover           = results[1][0]
    total_undiscovered_targets  = results[2][0]
    total_discovered_targets    = results[3][0]
    total_captured_targeted     = results[4][0]
    total_suppressed_targets    = results[5][0]
    total_targets               = results[6][0]

    # Compute the movement statistics
    results = get_movement_statistics(payload_dfs)
    total_movement_times        = results[0][0]
    total_dwelling_times        = results[1][0]
    total_distance_traveled     = results[2][0]

    # Get the total targets and payloads
    results = configuration_status(target_dfs, payload_dfs)
    total_targets               = results[0][0]
    total_payloads              = results[1][0]
    airAssets                   = results[2][0]
    groundAssets                = results[3][0]

    # Get the individual target statistics
    results = get_individual_target_statistics(target_dfs)
    target_labels                   = results[0]
    target_time_range_to_discover   = results[1]
    target_time_range_to_capture    = results[2]
    target_average_time_to_discover = results[3]
    target_average_time_to_capture  = results[4]

    target_time_to_discover = []
    target_time_to_capture = []
    for dis, cap in zip(target_time_range_to_discover, target_time_range_to_capture):
        # Make sure the min and max are the same
        assert(dis[0] == dis[1])
        assert(dis[0] == dis[1])
        # Append the data (NA, time, radius size)
        target_time_to_discover.append([0, dis[0], POINT_RADIUS])
        target_time_to_capture.append([0, cap[0], POINT_RADIUS])

    # Convert times to strings
    mission_time            = datetime.timedelta(seconds=np.round(mission_time))
    time_to_discover_all    = datetime.timedelta(seconds=np.round(times_to_discover))
    time_to_capture_all     = datetime.timedelta(seconds=np.round(times_to_capture))
    total_movement_times    = datetime.timedelta(seconds=np.round(total_movement_times))
    total_dwelling_times    = datetime.timedelta(seconds=np.round(total_dwelling_times))

    # Clean up the distance traveled
    total_distance_traveled = np.round(total_distance_traveled)

    # Generate the pie data
    pie_data            = {'labels'         : ['Undiscovered', 'Discovered', 'Captured', 'Suppressed'],
                           'data'           : [total_undiscovered_targets, total_discovered_targets, total_captured_targeted, total_suppressed_targets]}

    # Get the configuration data
    configuration_data   = {'TotalTargets'      : total_targets,
                            'TotalPayloads'     : total_payloads,
                            'TotalAirAssets'    : airAssets,
                            'TotalGroundAssets' : groundAssets}

    # Generate the Mission Statistics
    mission_statistics   = {'TimeToDiscoverAll' : time_to_discover_all,
                            'TimeToCaptureAll'  : time_to_capture_all,
                            'MovementTime'      : total_movement_times,
                            'DwellingTimer'     : total_dwelling_times,
                            "DistanceTraveled"  : total_distance_traveled}

    target_statistics    = {'TargetLabels'              : target_labels,
                            'TargetTimeToDiscover'      : target_time_to_discover,
                            'TargetTimeToCapture'       : target_time_to_capture}

    # Send it to the render_template
    return render_template(template_name_or_list        = 'single_run.html',
                           pie_data                     = pie_data,
                           configuration_data           = configuration_data,
                           current_time                 = mission_time,
                           mission_statistics           = mission_statistics,
                           target_statistics            = target_statistics,
                           refresh_rate                 = REFRESH_RATE,)     

@app.route('/multiple_runs', methods=['GET', 'POST'])
def show_multiple_runs():
    """
    Displays all logs files and then if it gets a POST message
    from the page with specific log files, it computes averaged statistics
    about the selected log files and displays it on the /multiple_runs webpage
    """
    
    # Get all the log files
    log_files = get_all_logs()

    # If the log file is none, wait for log files
    if log_files is None:
        return render_template('loading.html', refresh_rate=REFRESH_RATE)

    log_files = sorted(log_files)

    # Clean the log files for display
    log_files_display = [log[log.rfind("/")+1:] for log in log_files]

    # Get all the checkboxes which were clicked
    requested_files = []
    if request.method == 'POST':
        if len(request.form.getlist('selectall')) == 1:
            requested_files = log_files_display
        elif len(request.form.getlist('update')) == 1:
            requested_files = request.form.getlist('logfile_checkbox') 

    # Get the list of files we want to process
    requested_files_path = []
    for log in log_files:
        for requested_log in requested_files:
            if requested_log in log:
                requested_files_path.append(log)
    
    # Load the requested data
    data = load_data(requested_files_path)

    # Check that we have data
    if data is None:
        return render_template('loading.html', refresh_rate=REFRESH_RATE)

    # Compute which files should be checked and which shouldn't
    log_files_checked = []
    for log_file in log_files_display:
        if log_file in requested_files:
            log_files_checked.append(True)
        else:
            log_files_checked.append(False)

    # Get the data
    target_dfs  = data[0]
    payload_dfs = data[1]

    # Get all moving times
    results = get_mission_time(payload_dfs)
    all_mission_times               = results

    # Get all mission statistics
    results = get_mission_statistics(target_dfs)
    all_times_to_capture            = results[0]
    all_times_to_discover           = results[1]
    all_total_undiscovered_targets  = results[2]
    all_total_discovered_targets    = results[3]
    all_total_captured_targeted     = results[4]
    all_total_targets               = results[5]

    # Get the individual target statistics
    results = get_individual_target_statistics(target_dfs)
    target_labels                   = results[0]
    target_time_range_to_discover   = results[1]
    target_time_range_to_capture    = results[2]
    target_average_time_to_discover = results[3]
    target_average_time_to_capture  = results[4]
    
    # Compute all movement statistics
    results = get_movement_statistics(payload_dfs)
    all_total_movement_times        = results[0]
    all_total_dwelling_times        = results[1]
    all_total_distance_traveled     = results[2]

    # Compute the tag data
    results = get_tag_data(requested_files,
                           all_times_to_capture,
                           all_times_to_discover,
                           all_mission_times,
                           all_total_distance_traveled,
                           all_total_dwelling_times,
                           all_total_movement_times)
    tag_labels                      = results[0]
    normalized_tag_data             = results[1]

    # Get the set colors
    colors = get_colors()

    # Compute the averages where required
    avg_mission_time                = np.round(np.mean(np.array(all_mission_times)))
    avg_time_to_capture             = np.round(np.mean(np.array(all_times_to_capture)))
    avg_time_to_discover            = np.round(np.mean(np.array(all_times_to_discover)))
    avg_total_movement_times        = np.round(np.mean(np.array(all_total_movement_times)))
    avg_total_dwelling_times        = np.round(np.mean(np.array(all_total_dwelling_times)))
    avg_total_distance_traveled     = np.round(np.mean(np.array(all_total_distance_traveled)))

    # Compute the number of successful runs
    successful_runs = 0
    unsuccessful_runs = 0
    for cur_total_targets, cur_total_captured in zip(all_total_targets, all_total_captured_targeted):
        # Check there were targets
        if cur_total_targets == 0:
            break
        # If the total number of targets is equal to the total captured targets
        if cur_total_targets == cur_total_captured:
            successful_runs += 1
        else:
            unsuccessful_runs += 1

    # Compute the success percentage
    percentage_successful_runs = 0
    if successful_runs != 0:
        percentage_successful_runs = np.round((len(all_total_targets) / successful_runs) * 100)

    # Convert the average target capture and discovery times to have a radius size
    target_discovery_vs_capture_time = []
    for dis, cap in zip(target_average_time_to_discover, target_average_time_to_capture):
        # Append the data (discovery  time, capture, radius size)
        target_discovery_vs_capture_time.append([[dis, cap, POINT_RADIUS]])

    # Convert times to strings
    avg_mission_time            = datetime.timedelta(seconds=avg_mission_time)
    avg_time_to_capture_all     = datetime.timedelta(seconds=avg_time_to_capture)
    avg_time_to_discover_all    = datetime.timedelta(seconds=avg_time_to_discover)
    avg_total_movement_times    = datetime.timedelta(seconds=avg_total_movement_times)
    avg_total_dwelling_times    = datetime.timedelta(seconds=avg_total_dwelling_times)
    
    # Add all the data to dictionaries to be accessed in the html file
    mission_statistics  = {'avg_time_to_discover_all'    : avg_time_to_discover_all,
                           'avg_time_to_capture_all'     : avg_time_to_capture_all}

    mission_success     = {'labels'                      : ["Failed Missions", "Successful Missions"],
                           'data'                        : [unsuccessful_runs, successful_runs],
                           'percentage_successful_runs'  : percentage_successful_runs}

    movement_statistics = {'total_movement_times'        : avg_total_movement_times,
                           'total_dwelling_times'        : avg_total_dwelling_times,
                           'total_distance_traveled'     : avg_total_distance_traveled}

    complete_mission_times = {'labels'                   : requested_files,
                              'mission_completion_times' : all_mission_times,
                              'time_to_discover_all'     : all_times_to_discover,
                              'time_to_capture_all'      : all_times_to_capture}

    target_statistics    = {'TargetLabels'               : target_labels,
                            'TargetTimeToDiscoverRange'  : target_time_range_to_discover,
                            'TargetTimeToCaptureRange'   : target_time_range_to_capture,
                            'TargetDiscoveryVsCapture'   : target_discovery_vs_capture_time,}

    distance_traveled   = {'labels'                      : requested_files,
                           'distance_traveled'           : all_total_distance_traveled}

    tag_data            = {"labels"                      : tag_labels,
                           "data"                        : normalized_tag_data}

    # Send it to the render_template
    return render_template(template_name_or_list        = 'multiple_runs.html',
                           log_files                    = zip(log_files_display, log_files_checked),
                           average_mission_time         = avg_mission_time,
                           mission_statistics           = mission_statistics,
                           mission_success_statistics   = mission_success,
                           movement_statistics          = movement_statistics,
                           complete_mission_times       = complete_mission_times,
                           target_statistics            = target_statistics,
                           distance_traveled            = distance_traveled,
                           tag_data                     = tag_data,
                           zip                          = zip,
                           colors                       = colors)     

@app.route('/event_map')
def show_event_map():
    """
    Computes the event map, and returns it as an html page
    """
    # Get the current log file from all logs
    current_log = get_current_log()

    # Generate a new movement map based on the current log files
    command = "python3 ../mission_analysis/generate_event_map.py " + \
              "--output_file_dir reports/ " + \
              "--target_state_file {}/target_wf_output_state.csv ".format(current_log) + \
              "--payload_state_file {}/payload_output_state.csv".format(current_log)
    os.system(command)

    return send_from_directory('reports', 'event_map.html')

@app.route('/event_timeline')
def show_event_timeline():
    """
    Computes the event timeline, and returns it as an html page
    """
    # Get the current log file from all logs
    current_log = get_current_log()

    # Generate a new movement map based on the current log files
    command = "python3 ../mission_analysis/generate_event_timeline.py " + \
              "--output_file_dir reports/ " + \
              "--target_state_file {}/target_wf_output_state.csv ".format(current_log) + \
              "--payload_state_file {}/payload_output_state.csv ".format(current_log) + \
              "--target_config_file {}/target_output_configs.csv ".format(current_log) + \
              "--payload_config_file {}/payload_output_config.csv".format(current_log)
    os.system(command)

    return send_from_directory('reports', 'event_timeline.html')

@app.route('/mission_report')
def show_mission_report():
    """
    Computes the mission report, and returns it as an html page
    """

    # Get the current log file from all logs
    current_log = get_current_log()

    # Generate a new movement map based on the current log files
    command = "python3 ../mission_analysis/generate_mission_statistics.py " + \
            "--output_file_dir reports/ " + \
            "--target_state_file {}/target_wf_output_state.csv ".format(current_log)  + \
            "--payload_state_file {}/payload_output_state.csv".format(current_log)
    os.system(command)



    return send_from_directory('reports', 'mission_report.png')

@app.route('/movement_map')
def show_movement_map():
    """
    Computes the movement map, and returns it as an html page
    """

    # Get the current log file from all logs
    current_log = get_current_log()

    # Generate a new movement map based on the current log files
    command = "python3 ../mission_analysis/generate_movement_map.py " + \
              "--output_file_dir reports/ " + \
              "--target_state_file {}/target_wf_output_state.csv ".format(current_log)  + \
              "--payload_state_file {}/payload_output_state.csv".format(current_log)
    os.system(command)
    
    # Send them to the movement map
    return send_from_directory('reports', 'movement_map.html')

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5000, debug=True)
