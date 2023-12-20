#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import pandas as pd
import numpy as np
from pandas.errors import EmptyDataError
import argparse
from datetime import datetime
from datetime import timedelta
import os
import matplotlib.pyplot as plt
import warnings

warnings.filterwarnings("ignore")



class ReportGenerator:
    def __init__(self):
        pass


    def _parse_csv(self, filename):
        """
        Parse the csv file to a pandas dataframe
        """
        try:
            # Read the CSV
            df = pd.read_csv(filename)
            df.replace(r'^\s+$', np.nan, regex=True)
        except EmptyDataError:
            return pd.DataFrame(columns=['timestamp', 'uid'])

        # Convert the timestamp to pandas usable datetime
        df['timestamp'] = pd.to_datetime(df['timestamp'], format='%Y.%m.%d.%H.%M.%S')

        return df


    def _count_number_of_devices(self, df):
        """
        Count the total number of devices
        """
        return len(pd.unique(df['uid']))


    def _count_target_capture_instances(self, df):
        """
        Count the total number of targets that were captured
        """
        return len(df[df['captured'] == True])

    
    def _measure_mission_duration(self, target_df, payload_df):
        """
        Measure the total duration over which the mission occurred
        """
        target_start_time = target_df.iloc[0]['timestamp']
        payload_start_time = payload_df.iloc[0]['timestamp']

        start_time = min(target_start_time, payload_start_time)

        target_end_time = target_df.iloc[len(target_df) - 1]['timestamp']
        payload_end_time = payload_df.iloc[len(payload_df) - 1]['timestamp']

        end_time = max(target_end_time, payload_end_time)

        return end_time - start_time


    def _count_network_capture_instances(self, df):
        """
        Count the total number of networks that were captured
        """
        return len(df[df['network_captured'] == True])


    def _count_payload_in_range_events(self, df):
        """
        Count the total number of times payloads came in range 
        """        # Get all readings where the payloads were measured
        in_range_events = df[df['in_range'].notna()]

        # If there were none (e.g., in the case of IDLE targets), return the empty df
        if len(in_range_events) == 0:
            return 0

        # Determine where there were changes in the number of payloads present
        in_range_events['match'] = df['in_range'] == df['in_range'].shift()

        # Get only the rows where changes occurred
        in_range_events = in_range_events[in_range_events['match'] == False].drop(columns=['match'])

        return len(in_range_events[in_range_events['in_range'] == True])


    def _count_payload_left_detection_range_instances(self, df):
        """
        Count the total number of times payloads exited the detection range
        """
        # Get all readings where the payloads were measured
        in_range_events = df[df['in_range'].notna()]

        # If there were none (e.g., in the case of IDLE targets), return the empty df
        if len(in_range_events) == 0:
            return 0

        # Determine where there were changes in the number of payloads present
        in_range_events['match'] = df['in_range'] == df['in_range'].shift()

        # Get only the rows where changes occurred
        in_range_events = in_range_events[in_range_events['match'] == False].drop(columns=['match'])

        return len(in_range_events[in_range_events['in_range'] == False])


    def _calculate_entrance_to_exit_range_ratio(self, in_range_instances, left_range_instances):
        """
        Count the total number of times that payloads exited the detection range
        """
        return in_range_instances / left_range_instances


    def _measure_capture_time_stats(self, column, df):
        """
        Measure the average time to capture a target
        """
        # Get all readings where the payloads were measured
        target_states = df[df[column].notna()]

        # If there were none (e.g., in the case of IDLE targets), return the empty df
        if len(target_states) == 0:
            return 0, 0, (0, None), (0, None)

        # Determine which targets were captured
        targets = pd.unique(target_states['uid'])

        # Stats variables
        total_capture_time = timedelta(0)
        min_capture_time = timedelta(1000)
        min_capture_target = None
        max_capture_time = timedelta(0)
        max_capture_target = None

        # Measure the stats using each target
        for target in targets:
            # Get the current target's state information
            current_target_state = target_states[target_states['uid'] == target]

            # Skip the target if there is no valid state information
            if len(current_target_state) == 0:
                continue

            # Get the start time
            start_time = target_states.iloc[0]['timestamp']

            # Determine where there were changes in the target capture state
            current_target_state['match'] = current_target_state[column] == current_target_state[column].shift()

            # Get only the rows where changes occurred
            events = current_target_state[current_target_state['match'] == False].drop(columns=['match'])

            # Get only the rows where a target was captured
            capture_events = events[events[column] == True]

            # Skip if there were no valid capture events
            if len(capture_events) == 0:
                continue

            # The target was only captured oned
            elif len(capture_events) == 1:
                # Get the time difference between the start time and the capture time
                capture_time = capture_events.iloc[0]['timestamp'] - start_time
                
                # Update the relevant stats
                total_capture_time += capture_time

                if capture_time > max_capture_time:
                    max_capture_time = capture_time
                    max_capture_target = target
                
                if capture_time < min_capture_time:
                    min_capture_time = capture_time
                    min_capture_target = target
            # The target was captured multiple times
            else:
                # Get the first capture time
                initial_capture_time = capture_events.iloc[0]['timestamp'] - start_time

                # Update the stats
                total_capture_time += initial_capture_time

                if initial_capture_time > max_capture_time:
                    max_capture_time = initial_capture_time
                    max_capture_target = target
                
                if initial_capture_time < min_capture_time:
                    min_capture_time = initial_capture_time
                    min_capture_target = target

                # Calculate the duration and states for subsequent captures
                for i in range(1, len(capture_events)):
                    # Calculate the subsequent capture
                    subsequent_capture_time = capture_events.iloc[i]['timestamp'] - capture_events.iloc[i - 1]['timestamp']

                    # Update the stats
                    total_capture_time += subsequent_capture_time

                    if subsequent_capture_time > max_capture_time:
                        max_capture_time = subsequent_capture_time
                        max_capture_target = target
                    
                    if subsequent_capture_time < min_capture_time:
                        min_capture_time = subsequent_capture_time
                        min_capture_target = target
                    
        return total_capture_time, (total_capture_time / len(targets)), (max_capture_time, max_capture_target), (min_capture_time, min_capture_target)



    def generate_report(self, target_state_filename, payload_state_filename, output_filename):
        """
        TODO
        """
        # Get the CSV data as a dataframe
        target_state = self._parse_csv(target_state_filename)
        payload_state = self._parse_csv(payload_state_filename)

        total_targets = self._count_number_of_devices(target_state)
        total_payloads = self._count_number_of_devices(payload_state)

        mission_duration = self._measure_mission_duration(target_state, payload_state)
        
        total_times_target_captured = self._count_target_capture_instances(target_state)
        total_times_network_captured = self._count_network_capture_instances(target_state)

        in_range_events = self._count_payload_in_range_events(payload_state)
        exit_range_events = self._count_payload_left_detection_range_instances(payload_state)

        entrance_to_exit_ratio = self._calculate_entrance_to_exit_range_ratio(in_range_events, exit_range_events)

        total_target_capture_time, average_target_capture_time, (max_target_capture_time, max_target_capture_device), (min_target_capture_time, min_target_capture_device) = self._measure_capture_time_stats('captured', target_state)
        total_network_capture_time, average_network_capture_time, (max_network_capture_time, max_network_capture_device), (min_network_capture_time, min_network_capture_device) = self._measure_capture_time_stats('network_captured', target_state)

        #define figure and axes
        fig, ax = plt.subplots()

        # create values for table
        table_data=[
            ['Total Targets', total_targets],
            ['Total Payloads', total_payloads],
            ['Mission Duration', mission_duration],
            ['Total Target Capture Instances', total_times_target_captured],
            ['Total Network Capture Instances', total_times_network_captured],
            ['Payload In Range Instances', in_range_events],
            ['Payload Left Detection Range Instances', exit_range_events],
            ['Payload Detection Range Entrance to Exit Ratio', entrance_to_exit_ratio],
            ['Total Target Capture Time', total_target_capture_time],
            ['Average Target Capture Time', average_target_capture_time],
            ['Max Target Capture Time', max_target_capture_time],
            ['Target that Required the Max Capture Time', max_target_capture_device],
            ['Minimum Target Capture Time', min_target_capture_time],
            ['Target that Required the Minimum Capture Time', min_target_capture_device],
            ['Total Network Target Capture Time', total_network_capture_time],
            ['Average Network Capture Time', average_network_capture_time],
            ['Max Network Capture Time', max_network_capture_time],
            ['Target that Required the Max Capture Time', max_network_capture_device],
            ['Minimum Network Capture Time', min_network_capture_time],
            ['Network that Required the Minimum Capture Time', min_network_capture_device],
        ]

        column_label_color = [['#fcb8b8','w']] * len(table_data)

        #create table
        table = ax.table(cellText=table_data, loc='center', cellColours=column_label_color, cellLoc='left')

        #modify table
        fig.tight_layout()
        table.set_fontsize(14)
        ax.axis('off')
        ax.set_title('MACE Mission Report', fontweight='bold') 

        #display table
        plt.savefig(output_filename, dpi=150)



def main():
    # Create a new argument parser
    parser = argparse.ArgumentParser(description='System used to generate an analytics report from target state data and payload state data')

    # Add the desired arguments
    parser.add_argument('--target_state_file', type=str, help='The full path to the target state log file produced by the analytics server')
    parser.add_argument('--payload_state_file', type=str, help='The full path to the payload state log file produced by the analytics server')
    parser.add_argument('--output_file_dir', type=str, default=None, help='The full path to the directory that the generated event map should be saved to')
    parser.add_argument('--output_filename', type=str, default='mission_report', help='The filename that the generated event map should be called')

    # Parse the arguments
    args = parser.parse_args()

    # Create the filename for the generated movement map
    if args.output_file_dir is not None:
        output_dir = os.getcwd()

        output_dir = output_dir + "/" + args.output_file_dir + '/'
        
        if not os.path.exists(output_dir):
            os.mkdir(output_dir)

        output_filename = output_dir + args.output_filename + '.png'
    else:
        output_dir = os.getcwd() + '/results/'

        if not os.path.exists(output_dir):
            os.mkdir(output_dir)

        output_dir = output_dir + datetime.now().strftime('%Y-%m-%d-%H-%M-%S') + '/'

        if not os.path.exists(output_dir):
            os.mkdir(output_dir)

        output_filename = output_dir + args.output_filename + '.png'

    # Create a new map generator
    generator = ReportGenerator()

    # Generate the movement map
    generator.generate_report(args.target_state_file, args.payload_state_file, output_filename)


if __name__ == '__main__':
    main()
