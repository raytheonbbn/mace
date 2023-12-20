#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import argparse
import os
from datetime import datetime
from bokeh.plotting import figure, save
from bokeh.models import ColumnDataSource, CheckboxGroup, CustomJS, Div, WMTSTileSource, Button
from bokeh.core.templates import FILE
from bokeh.colors import RGB
from bokeh.models.tools import HoverTool
from bokeh.layouts import row, column, widgetbox
import pandas as pd
import numpy as np
import warnings

warnings.filterwarnings("ignore")


class MovementMapGenerator:
    def __init__(self):        
        
        # Specify the data that will display on point hover
        self.hover_data = [
            ('Timestamp', '@timestamp{%Y-%m-%d %H:%M:%S.%3N}'),
            ('UID', '@uid'),
            ('Latitude', '@latitude'),
            ('Longitude', '@longitude'), 
            ('Altitude', '@altitude')
        ]


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


    def _parse_csv(self, filename):
        """
        Parse the file for the device locations
        """
        # Read the CSV
        df = pd.read_csv(filename, usecols=['timestamp', 'latitude', 'longitude', 'altitude', 'uid'])
        df.replace(r'^\s+$', np.nan, regex=True)

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

        # Convert the timestamp to pandas usable datetime
        trim_df['timestamp'] = pd.to_datetime(df['timestamp'], format='%Y.%m.%d.%H.%M.%S')

        return trim_df

    
    def generate_movement_map(self, target_state_filename, payload_state_filename, output_filename):
        """
        Use the parsed locations to generate a movement map of the targets and/or payloads
        """
        target_locations = self._parse_csv(target_state_filename)
        payload_locations = self._parse_csv(payload_state_filename)

        # Get the UIDs of all of the targets and payloads that should be parsed and plotted
        targets = pd.unique(target_locations['uid'])
        payloads = pd.unique(payload_locations['uid'])

        # Create a new plot
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
        plt.title.text = 'MACE Movement Map'
        plt.title.align = 'center'
        plt.title.text_font_size = "25px"

        # Add a new map provider
        mq_tile_source = WMTSTileSource(url='https://mt1.google.com/vt/lyrs=s&x={X}&y={Y}&z={Z}', attribution='Map data ©2019 Google')

        plt.add_tile(mq_tile_source)

        # Dictionary that will be passed to the custom js
        renderers = {
            'renderers': []
        }

        # List of all labels that will be used for the checkbox
        labels = []

        for target in targets:
            current_target_locations_df = target_locations[target_locations['uid'] == target]

            # Skip this device if it has no valid locations
            if len(current_target_locations_df) == 0:
                continue

            # Convert the dataframe to a datasource
            current_target_locations = ColumnDataSource(current_target_locations_df)

            # Plot the target locations
            target_renderer = plt.line('x_mercator', 'y_mercator', source=current_target_locations, color=RGB(247, 139, 139), line_width=3, legend_label='Target Movement')
            plt.add_tools(HoverTool(renderers=[target_renderer], tooltips=self.hover_data, formatters={'@timestamp': 'datetime'}))
            target_renderer.visible = False

            # Add the label and renderers to the checkbox list
            labels.append(target)
            renderers['renderers'].append(target_renderer)

        for payload in payloads:
            current_payload_locations_df = payload_locations[payload_locations['uid'] == payload]

            # Skip this device if there are no valid locations
            if len(current_payload_locations_df) == 0:
                continue

            # Convert the dataframe to a datasource
            current_payload_locations = ColumnDataSource(current_payload_locations_df)

            # Plot the payload locations
            payload_renderer = plt.line('x_mercator', 'y_mercator', source=current_payload_locations, color=RGB(153, 211, 242), line_width=3, legend_label='Payload Movement')
            plt.add_tools(HoverTool(renderers=[payload_renderer], tooltips=self.hover_data, formatters={'@timestamp': 'datetime'}))
            payload_renderer.visible = False

            # Add the label and renderers to the checkbox list
            labels.append(payload)
            renderers['renderers'].append(payload_renderer)

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

        # Attach a callback that shows/hides movement according to their checkbox
        checkbox.js_on_click(CustomJS(args=renderers, code="""
            for(let i = 0; i < renderers.length; i++){
                renderers[i].visible = checkbox.active.includes(i);
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
        checkbox_title = Div(text='<h3 style="text-align: center">Display Device Movement</h3>')
        select_events = column(checkbox_title, widgetbox(show_btn, width=220), widgetbox(hide_btn, width=220), checkbox)

        # Create a new layout with the checkbox and plot
        layout = row(select_events, plt)

        # Save the plot
        save(layout, filename=output_filename, title='MACE Movement Map', template=template)


def main():
    # Create a new argument parser
    parser = argparse.ArgumentParser(description='System used to generate a movement map of the targets and payloads')

    # Add the desired arguments
    parser.add_argument('--payload_state_file', type=str, help='The full path to the payload state log file produced by the analytics server')
    parser.add_argument('--target_state_file', type=str, help='The full path to the target state log file produced by the analytics server')
    parser.add_argument('--output_file_dir', type=str, default=None, help='The full path to the directory that the generated movement map should be saved to')
    parser.add_argument('--output_filename', type=str, default='movement_map', help='The filename that the generated map should be called')

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
    generator = MovementMapGenerator()

    # Generate the movement map
    generator.generate_movement_map(args.target_state_file, args.payload_state_file, output_filename)


if __name__ == '__main__':
    main()
