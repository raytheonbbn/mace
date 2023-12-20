#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import argparse
import os
from datetime import datetime
import pandas as pd
import numpy as np
from pandas.errors import EmptyDataError
from bokeh.plotting import figure, save
from bokeh.models import ColumnDataSource, CheckboxGroup, CustomJS, Div, WMTSTileSource, Button
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



class EventMapGenerator:
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

        if 'latitude' in df:
            # Trim the dataframe down to the non-start points
            trim_df = df.loc[df['latitude'] != 15000]

            # Get the latitude and longitude columns as numpy arrays
            latitudes = trim_df['latitude'].to_numpy()
            longitudes = trim_df['longitude'].to_numpy()

            # Calculate the mercator coordinates from the latitudes and longitudes
            x, y = self._calculate_mercator_coordinates(latitudes, longitudes)

            # Append the mercator coordinates to the dataframe
            trim_df['x_mercator'] = x.tolist()
            trim_df['y_mercator'] = y.tolist()

            return trim_df

        return df

    
    def _calculate_mercator_coordinates(self, latitudes, longitudes):
        """
        Convert latitude and longitude coordinates to mercator and return as a new dataframe
        that may be merged to another
        """
        # Semi-major axis used for conversion
        r = 6378137.000

        # Convert longitudes to x-values
        x = r * np.radians(longitudes)

        # Calculate the scale to apply toward the latitudes
        scale = x/longitudes

        # Convert latitudes to y-values
        y = 180.0/np.pi * np.log(np.tan(np.pi/4.0 + latitudes * (np.pi/180.0)/2.0)) * scale

        return x, y

    
    def generate_event_map(self, target_state_filename, payload_state_filename, output_filename):
        """
        Generate an event map displaying significant events for targets and payloads
        """
        # Get all of the csv data as pandas dataframes
        target_state_df = self._parse_csv(target_state_filename)
        payload_state_df = self._parse_csv(payload_state_filename)

        # Create new parsers for the states
        target_parser = TargetEventParser()
        payload_parser = PayloadEventParser()

        # Get the UIDs of all of the targets and payloads that should be parsed and plotted
        targets = pd.unique(target_state_df['uid'])
        payloads = pd.unique(payload_state_df['uid'])

        # Create a new figure
        plt = figure(plot_height=800, x_axis_label='Longitude', tools='pan,wheel_zoom,reset,save,help', y_axis_label='Latitude',
            x_axis_type='mercator', y_axis_type='mercator', sizing_mode='stretch_width')

        # Configure the Axis labels
        plt.xaxis.axis_label_text_font_size = '18px'
        plt.xaxis.axis_label_text_font_style = 'bold'
        plt.xaxis.major_label_text_font_size = '12px'
        plt.xaxis.major_label_text_font_style = 'bold'
        
        plt.yaxis.axis_label_text_font_size = '18px'
        plt.yaxis.axis_label_text_font_style = 'bold'
        plt.yaxis.major_label_text_font_size = '12px'
        plt.yaxis.major_label_text_font_style = 'bold'

        # Set the title properties
        plt.title.text = 'MACE Event Map'
        plt.title.align = 'center'
        plt.title.text_font_size = "25px"

        # Add a new map provider
        mq_tile_source = WMTSTileSource(url='https://mt1.google.com/vt/lyrs=s&x={X}&y={Y}&z={Z}', attribution='Map data © 2021 Google')

        plt.add_tile(mq_tile_source)

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

            # If there are no events to plot, skip the device
            if len(presence_events.to_df()) == 0 and len(capture_events.to_df()) == 0 and \
                len(network_capture_events.to_df()) == 0:
                continue

            # Draw the points
            presence_renderer = plt.circle('x_mercator', 'y_mercator', source=presence_events, size=10, fill_color=RGB(158, 0, 0), 
                line_color=RGB(158, 0, 0), legend_label='Payloads In Range Changed')
            plt.add_tools(HoverTool(renderers=[presence_renderer], tooltips=self.presence_hover_data, formatters={'@timestamp': 'datetime'}))
            presence_renderer.visible = False

            capture_renderer = plt.circle('x_mercator', 'y_mercator', source=capture_events, size=10, fill_color=RGB(247, 139, 139), 
                line_color=RGB(247, 139, 139), legend_label='Target Capture State Changed')
            plt.add_tools(HoverTool(renderers=[capture_renderer], tooltips=self.capture_hover_data, formatters={'@timestamp': 'datetime'}))
            capture_renderer.visible = False

            network_renderer = plt.circle('x_mercator', 'y_mercator', source=network_capture_events, size=10, fill_color=RGB(224, 224, 224), 
                line_color=RGB(224, 224, 224), legend_label='Network Capture State Changed')
            plt.add_tools(HoverTool(renderers=[network_renderer], tooltips=self.network_capture_hover_data, formatters={'@timestamp': 'datetime'}))
            network_renderer.visible = False

            # Add the label and renderers to the checkbox list
            labels.append(target)
            renderers['renderers'].append([
                presence_renderer, 
                capture_renderer, 
                network_renderer, 
            ])

        # Display all payload events
        for payload in payloads:
            # Get the a dataframe for the current payload
            current_payload_df = payload_state_df[payload_state_df['uid'] == payload]

            # Parse the current payload's events
            in_range_events = ColumnDataSource(payload_parser.parse_in_range_events(current_payload_df))
            type_in_range_events = ColumnDataSource(payload_parser.parse_type_in_range_events(current_payload_df))

            # If there are no events to plot, skip the device
            if len(in_range_events.to_df()) == 0 and len(type_in_range_events.to_df()) == 0:
                continue

            # Draw the points
            in_range_renderer = plt.circle('x_mercator', 'y_mercator', source=in_range_events, size=10, fill_color=RGB(153, 211, 242), 
                line_color=RGB(153, 211, 242), legend_label='Payload Entered/Exited Detection Range')
            plt.add_tools(HoverTool(renderers=[in_range_renderer], tooltips=self.payload_hover_data, formatters={'@timestamp': 'datetime'}))
            in_range_renderer.visible = False

            type_in_range_renderer = plt.circle('x_mercator', 'y_mercator', source=type_in_range_events, size=10, fill_color=RGB(153, 211, 242), 
                line_color=RGB(153, 211, 242), legend_label='Payload Entered/Exited Detection Range')
            plt.add_tools(HoverTool(renderers=[type_in_range_renderer], tooltips=self.payload_hover_data, formatters={'@timestamp': 'datetime'}))
            type_in_range_renderer.visible = False

            # Add the labels and renderers to the checkbox list
            labels.append(payload)
            renderers['renderers'].append([
                in_range_renderer,
                type_in_range_renderer,
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
        save(layout, filename=output_filename, title='MACE Event Map', template=template)



def main():
    # Create a new argument parser
    parser = argparse.ArgumentParser(description='System used to generate a movement map of the targets and payloads')

    # Add the desired arguments
    parser.add_argument('--target_state_file', type=str, help='The full path to the target state log file produced by the analytics server')
    parser.add_argument('--payload_state_file', type=str, help='The full path to the payload state log file produced by the analytics server')
    parser.add_argument('--output_file_dir', type=str, default=None, help='The full path to the directory that the generated event map should be saved to')
    parser.add_argument('--output_filename', type=str, default='event_map', help='The filename that the generated event map should be called')

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
    generator = EventMapGenerator()

    # Generate the movement map
    generator.generate_event_map(args.target_state_file, args.payload_state_file, output_filename)


if __name__ == '__main__':
    main()
