#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import argparse
import os
from datetime import datetime
import pandas as pd
import numpy as np
from pandas.errors import EmptyDataError
from bokeh.plotting import figure, show, save
from bokeh.models import ColumnDataSource, CheckboxGroup, CustomJS, Div, Button
from bokeh.core.templates import FILE
from bokeh.colors import RGB
from bokeh.models.tools import HoverTool
from bokeh.layouts import row, column, widgetbox
import warnings

warnings.filterwarnings("ignore")


class EventParser:
    def __init__(self):
        pass


    def _parse_target_events(self, column, df):
        """
        Parse a column within a dataframe for significant events
        """
        # Get all readings where the payloads were measured
        potential_events_df = df[df[column].notna()]

        # If there were none (e.g., in the case of IDLE targets), return the empty df
        if len(potential_events_df) == 0:
            return potential_events_df

        # Determine where there were changes in the number of payloads present
        potential_events_df['match'] = df[column] == df[column].shift()

        # Get only the rows where changes occurred
        potential_events_df = potential_events_df[potential_events_df['match'] == False].drop(columns=['match'])

        return potential_events_df



class TargetEventParser(EventParser):
    def __init__(self):
        pass

    
    def parse_payload_presence_events(self, df):
        """
        Parse a dataframe for events indicating whether a payload entered
        or exited a target's detection range
        """
        events = self._parse_target_events('payloads', df)

        return events


    def parse_capture_events(self, df):
        """
        Parse a dataframe for events indicating that a target was captured
        """
        events = self._parse_target_events('captured', df)

        return events


    def parse_network_capture_events(self, df):
        """
        Parse a dataframe for events indicating that a network was captured
        """
        events = self._parse_target_events('network_captured', df)

        return events



class PayloadEventParser(EventParser):
    def __init__(self):
        pass


    def parse_in_range_events(self, df):
        """
        Parse a dataframe for events indicating that a payload came into or left
        the detection range of a target
        """
        events = self._parse_target_events('in_range', df)

        return events


    def parse_type_in_range_events(self, df):
        """
        Parse a dataframe for events indicating that the type of payload that a 
        payload is in range of changed
        """
        events = self._parse_target_events('type_in_range', df)

        return events



class EventTimelineGenerator:
    def __init__(self):
        # Presence hover data
        self.presence_hover_data = [
            ('Timestamp', '@timestamp{%Y-%m-%d %H:%M:%S.%3N}'),
            ('UID', '@uid'),
            ('Type', '@type'),
            ('Latitude', '@latitude'),
            ('Longitude', '@longitude'), 
            ('Altitude', '@altitude'),
            ('Payloads', '@payloads'),
        ]

        # Capture target hover data
        self.capture_hover_data = [
            ('Timestamp', '@timestamp{%Y-%m-%d %H:%M:%S.%3N}'),
            ('UID', '@uid'),
            ('Type', '@type'),
            ('Latitude', '@latitude'),
            ('Longitude', '@longitude'), 
            ('Altitude', '@altitude'),
            ('Captured', '@captured')
        ]

        # Network capture hover data
        self.network_capture_hover_data = [
            ('Timestamp', '@timestamp{%Y-%m-%d %H:%M:%S.%3N}'),
            ('UID', '@uid'),
            ('Type', '@type'),
            ('Latitude', '@latitude'),
            ('Longitude', '@longitude'), 
            ('Altitude', '@altitude'),
            ('Network Captured', '@network_captured')
        ]

        # Target configuration hover data
        self.target_config_hover_data = [
            ('Timestamp', '@timestamp{%Y-%m-%d %H:%M:%S.%3N}'),
            ('UID', '@uid'),
            ('Type', '@type'),
            ('Detection Range', '@detection_range'),
            ('Required Payloads', '@required_payloads'),
            ('Capture Countdown', '@capture_countdown'),
            ('Delete', '@delete'),
            ('Change UID', '@change_uid'),
            ('Old UID', '@old_uid'),
            ('Network', '@network'),
            ('Modify Capture State', '@modify_capture_state'),
            ('Captured', '@captured')
        ]

        # Payload hover data
        self.payload_hover_data = [
            ('Timestamp', '@timestamp{%Y-%m-%d %H:%M:%S.%3N}'),
            ('UID', '@uid'),
            ('Latitude', '@latitude'),
            ('Longitude', '@longitude'), 
            ('Altitude', '@altitude'),
            ('In Range', '@in_range'),
            ('Type In Range', '@type_in_range')
        ]

        # Payload configuration hover data
        self.payload_config_hover_data = [
            ('Timestamp', '@timestamp{%Y-%m-%d %H:%M:%S.%3N}'),
            ('UID', '@uid'),
            ('Intent', '@intent')
        ]


    def _parse_csv(self, filename):
        """
        Parse the csv file to a pandas dataframe
        """
        try:
            # Read the CSV
            df = pd.read_csv(filename)
            df.replace(r'^\s+$', np.nan, regex=True)
            print(df.to_string())
        except EmptyDataError:
            return pd.DataFrame(columns=['timestamp', 'uid'])

        # Convert the timestamp to pandas usable datetime
        df['timestamp'] = pd.to_datetime(df['timestamp'], format='%Y.%m.%d.%H.%M.%S')

        if 'latitude' in df:
            # Trim the dataframe down to the non-start points
            trim_df = df.loc[df['latitude'] != 15000]

            return trim_df

        return df

    
    def generate_event_plot(self, target_state_filename, payload_state_filename, target_config_filename, 
        payload_config_filename, output_filename):
        """
        Generate an event plot displaying significant events for targets and payloads
        """
        # Get all of the csv data as pandas dataframes
        target_state_df = self._parse_csv(target_state_filename)
        payload_state_df = self._parse_csv(payload_state_filename)
        target_config_df = self._parse_csv(target_config_filename)
        payload_config_df = self._parse_csv(payload_config_filename)

        # Create new parsers for the states
        target_parser = TargetEventParser()
        payload_parser = PayloadEventParser()

        # Get the UIDs of all of the targets and payloads that should be parsed and plotted
        state_targets = pd.unique(target_state_df['uid'])
        config_targets = pd.unique(target_config_df['uid'])
        state_payloads = pd.unique(payload_state_df['uid'])
        config_payloads = pd.unique(payload_config_df['uid'])

        targets = np.append(state_targets, config_targets)
        payloads = np.append(state_payloads, config_payloads)

        # Get the list of devices whose events will be plotted
        uids = np.array([])

        # Get the UIDs of all devices
        if len(state_targets) > 0:
            for uid in state_targets:
                if uid not in uids:
                    uids = np.append(uids, uid)
        if len(config_targets) > 0:
            for uid in config_targets:
                if uid not in uids:
                    uids = np.append(uids, uid)
        if len(state_payloads) > 0:
            for uid in state_payloads:
                if uid not in uids:
                    uids = np.append(uids, uid)
        if len(config_payloads) > 0:
            for uid in config_payloads:
                if uid not in uids:
                    uids = np.append(uids, uid)

        # Create a new figure
        plt = figure(plot_height=800, tools='pan,wheel_zoom,reset,save,help',
            x_axis_label='Timestamp', y_range=uids[::-1], x_axis_type='datetime', sizing_mode='stretch_width')
        print("Created new figure.")

        # Configure the Axis labels
        plt.xaxis.axis_label_text_font_size = '18px'
        plt.xaxis.axis_label_text_font_style = 'bold'
        plt.xaxis.major_label_text_font_size = '12px'
        plt.xaxis.major_label_text_font_style = 'bold'
        
        plt.yaxis.major_label_text_font_size = '12px'
        plt.yaxis.major_label_text_font_style = 'bold'

        # Set the title properties
        plt.title.text = 'MACE Event Plot'
        plt.title.align = 'center'
        plt.title.text_font_size = "25px"

        # Dictionary containing the renderers to show/hide
        renderers = {
            'renderers': []
        }

        # List of checkbox labels to enable display of
        labels = []

        for target in targets:
            # Get a dataframe for the current target
            current_target_df = target_state_df[target_state_df['uid'] == target]

            # Parse the current target's events
            presence_events = ColumnDataSource(target_parser.parse_payload_presence_events(current_target_df))
            capture_events = ColumnDataSource(target_parser.parse_capture_events(current_target_df))
            network_capture_events = ColumnDataSource(target_parser.parse_network_capture_events(current_target_df))
            target_config_events = ColumnDataSource(target_parser._parse_target_events('type', target_config_df[target_config_df['uid'] == target]))

            # If there are no events to plot, skip the device
            if len(presence_events.to_df()) == 0 and len(capture_events.to_df()) == 0 and \
                len(network_capture_events.to_df()) == 0 and len(target_config_events.to_df()) == 0:
                continue

            if target in labels:
                #Don't add target twice.
                continue

            # Draw the points
            presence_renderer = plt.circle('timestamp', 'uid', source=presence_events, size=10, fill_color=RGB(158, 0, 0), 
                line_color=RGB(158, 0, 0), legend_label='Payloads In Range Changed')
            plt.add_tools(HoverTool(renderers=[presence_renderer], tooltips=self.presence_hover_data, formatters={'@timestamp': 'datetime'}))
            presence_renderer.visible = False

            capture_renderer = plt.circle('timestamp', 'uid', source=capture_events, size=10, fill_color=RGB(247, 139, 139), 
                line_color=RGB(247, 139, 139), legend_label='Target Capture State Changed')
            plt.add_tools(HoverTool(renderers=[capture_renderer], tooltips=self.capture_hover_data, formatters={'@timestamp': 'datetime'}))
            capture_renderer.visible = False

            network_renderer = plt.circle('timestamp', 'uid', source=network_capture_events, size=10, fill_color=RGB(224, 224, 224), 
                line_color=RGB(224, 224, 224), legend_label='Network Capture State Changed')
            plt.add_tools(HoverTool(renderers=[network_renderer], tooltips=self.network_capture_hover_data, formatters={'@timestamp': 'datetime'}))
            network_renderer.visible = False

            target_config_renderer = plt.circle('timestamp', 'uid', source=target_config_events, size=10, fill_color=RGB(255, 0, 77), 
                line_color=RGB(255, 0, 77), legend_label='Target Configuration Change')
            plt.add_tools(HoverTool(renderers=[target_config_renderer], tooltips=self.target_config_hover_data, formatters={'@timestamp': 'datetime'}))
            target_config_renderer.visible = False

            # Add the label and renderers to the checkbox list
            labels.append(target)
            renderers['renderers'].append([
                presence_renderer, 
                capture_renderer, 
                network_renderer, 
                target_config_renderer
            ])


        # Display all payload events
        for payload in payloads:
            # Get the a dataframe for the current payload
            current_payload_df = payload_state_df[payload_state_df['uid'] == payload]

            # Parse the current payload's events
            in_range_events = ColumnDataSource(payload_parser.parse_in_range_events(current_payload_df))
            type_in_range_events = ColumnDataSource(payload_parser.parse_type_in_range_events(current_payload_df))
            payload_config_events = ColumnDataSource(payload_parser._parse_target_events('intent', payload_config_df[payload_config_df['uid'] == payload]))

            # If there are no events to plot, skip the device
            if len(in_range_events.to_df()) == 0 and len(type_in_range_events.to_df()) == 0 and len(payload_config_events.to_df()) == 0:
                continue
            if payload in labels:
                #Don't add payload twice.
                continue

            # Draw the points
            in_range_renderer = plt.circle('timestamp', 'uid', source=in_range_events, size=10, fill_color=RGB(153, 211, 242), 
                line_color=RGB(153, 211, 242), legend_label='Payload Entered/Exited Detection Range')
            plt.add_tools(HoverTool(renderers=[in_range_renderer], tooltips=self.payload_hover_data, formatters={'@timestamp': 'datetime'}))
            in_range_renderer.visible = False

            type_in_range_renderer = plt.circle('timestamp', 'uid', source=type_in_range_events, size=10, fill_color=RGB(153, 211, 242), 
                line_color=RGB(153, 211, 242), legend_label='Payload Entered/Exited Detection Range')
            plt.add_tools(HoverTool(renderers=[type_in_range_renderer], tooltips=self.payload_hover_data, formatters={'@timestamp': 'datetime'}))
            type_in_range_renderer.visible = False
            
            payload_config_renderer = plt.circle('timestamp', 'uid', source=payload_config_events, size=10, fill_color=RGB(14, 108, 156), 
                line_color=RGB(14, 108, 156), legend_label='Payload Configuration Change')
            plt.add_tools(HoverTool(renderers=[payload_config_renderer], tooltips=self.payload_config_hover_data, formatters={'@timestamp': 'datetime'}))
            payload_config_renderer.visible = False

            # Add the labels and renderers to the checkbox list
            labels.append(payload)
            renderers['renderers'].append([
                in_range_renderer,
                type_in_range_renderer,
                payload_config_renderer
            ])

        show_btn = Button(label='Show All')
        hide_btn = Button(label='Hide All')

        # Create a new checkbox to show/hide events
        checkbox = CheckboxGroup(labels=labels, width=250)
        renderers['checkbox'] = checkbox

        # Attach a callback that shows/hides events according to their checkbox
        show_btn.js_on_click(CustomJS(args=renderers, code="""
            let visible = [];
            for(let i = 0; i < renderers.length; i++){
                visible.push(i);
                for(let j = 0; j < renderers[i].length; j++){
                    renderers[i][j].visible = true;
                }
            }
            checkbox.active = visible;
        """))

        hide_btn.js_on_click(CustomJS(args=renderers, code="""
            checkbox.active = [];
            for(let i = 0; i < renderers.length; i++){
                for(let j = 0; j < renderers[i].length; j++){
                    renderers[i][j].visible = false;
                }
            }
        """))

        # Attach a callback that shows/hides events according to their checkbox
        checkbox.js_on_click(CustomJS(args=renderers, code="""
            for(let i = 0; i < renderers.length; i++){
                for(let j = 0; j < renderers[i].length; j++){
                    renderers[i][j].visible = checkbox.active.includes(i);
                }
            }
        """))

        # Template used to remove the hover icons
        template = FILE.environment.from_string("""
            {% extends "file.html" %}
            {% block postamble %}
            <style>
            .bk-toolbar-button.bk-tool-icon-hover {
                display: none;
            }
            </style>
            {% endblock %}
        """)

        # Create the device display selection column
        checkbox_title = Div(text='<h3 style="text-align: center">Display Device Events</h3>')
        select_events = column(checkbox_title, widgetbox(show_btn, width=220), widgetbox(hide_btn, width=220), checkbox)

        # Create a new layout with the checkbox and plot
        layout = row(select_events, plt)

        # Save the plot
        print("Saving plot.")
        save(layout, filename=output_filename, title='MACE Event Plot', template=template)



def main():
    # Create a new argument parser
    parser = argparse.ArgumentParser(description='System used to generate a movement map of the targets and payloads')

    # Add the desired arguments
    parser.add_argument('--target_state_file', type=str, help='The full path to the target state log file produced by the analytics server')
    parser.add_argument('--payload_state_file', type=str, help='The full path to the payload state log file produced by the analytics server')
    parser.add_argument('--target_config_file', type=str, help='The full path to the target configuration log file produced by the analytics server')
    parser.add_argument('--payload_config_file', type=str, help='The full path to the payload configuration log file produced by the analytics server')
    parser.add_argument('--output_file_dir', type=str, default=None, help='The full path to the directory that the generated event plot should be saved to')
    parser.add_argument('--output_filename', type=str, default='event_timeline', help='The filename that the generated event plot should be called')

    # Parse the arguments
    args = parser.parse_args()

    # Create the filename for the generated movement map
    if args.output_file_dir is not None:
        output_dir = os.getcwd()

        output_dir = output_dir + "/" + args.output_file_dir + '/'
        
        if not os.path.exists(output_dir):
            os.mkdir(output_dir)

        output_filename = output_dir + args.output_filename + '.html'
    else:
        output_dir = os.getcwd() + '/results/'

        if not os.path.exists(output_dir):
            os.mkdir(output_dir)

        output_dir = output_dir + datetime.now().strftime('%Y-%m-%d-%H-%M-%S') + '/'

        if not os.path.exists(output_dir):
            os.mkdir(output_dir)

        output_filename = output_dir + args.output_filename + '.html'

    # Create a new map generator
    generator = EventTimelineGenerator()

    # Generate the movement map
    generator.generate_event_plot(args.target_state_file, args.payload_state_file, args.target_config_file, args.payload_config_file, output_filename)


if __name__ == '__main__':
    main()
