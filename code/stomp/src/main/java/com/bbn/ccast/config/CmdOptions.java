//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.config;

import com.google.devtools.common.options.Option;
import com.google.devtools.common.options.OptionsBase;

import java.lang.reflect.Field;
import java.util.List;

public class CmdOptions extends OptionsBase {

    public static final String DEFAULT_STRING = "null";
    public static final String DEFAULT_INT = "null";
    public static final String DEFAULT_DOUBLE = "null";
    public static final String DEFAULT_BOOLEAN = "null";

    @Option(
            name = "help",
            abbrev = 'h',
            help = "Prints usage info.",
            defaultValue = DEFAULT_BOOLEAN,
            converter = BooleanOptionsConverter.class
    )
    public Boolean help;

    @Option(
            name = "uid",
            abbrev = 'u',
            help = "Unique ID for this device",
            defaultValue = DEFAULT_STRING
    )
    public String uid; // do not use this... use getUid()

    public String getUid() {
        // If it's the default value
        if (DEFAULT_STRING.equalsIgnoreCase(uid)) {
            // Make one up
            uid = Integer.valueOf((int) (Math.random() * 1000 % 1000)).toString();
        }
        return uid;
    }

    @Option(
            name = "file",
            abbrev = 'f',
            help = "Read program from file",
            defaultValue = DEFAULT_STRING
    )
    public String file;

    @Option(
            name = "address",
            abbrev = 'a',
            help = "Address of where to listen for incoming Protelis messages",
            defaultValue = DEFAULT_STRING
    )
    public String listenaddress;

    @Option(
            name = "port",
            abbrev = 'p',
            help = "Port on which to listen for incoming Protelis messages",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer listenport;

    @Option(
            name = "destinations",
            help = "Comma separated list of address:port destinations for Protelis messages",
            defaultValue = DEFAULT_STRING
    )
    public String destinations;

    @Option(
            name = "staleRounds",
            help = "Number of advertisements missed (in rounds) missed before removing a neighbor",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer staleRounds;

    @Option(
            name = "overallLoggingLevel",
            abbrev = 'd',
            help = "Debug value (trace|debug|info|warning|error|off)",
            defaultValue = DEFAULT_STRING
    )
    public String overallLoggingLevel;

    @Option(
            name="mavlinkFCU",
            help="MAVLink Flight Control Unit (FCU) string (e.g., '/dev/ttyS0,57600' or 'tcp:192.168.1.122:5760')",
            defaultValue = DEFAULT_STRING
    )
    public String mavlinkFCU;

    @Option(
            name = "cotAddress",
            abbrev = 'o',
            help = "Address to share CoT",
            defaultValue = DEFAULT_STRING
    )
    public String cotAddress;

    @Option(
            name = "cotPort",
            abbrev = 't',
            help = "Port to share CoT",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer cotPort;

    @Option(
            name = "visualize",
            abbrev = 'v',
            help = "Enable visualization",
            defaultValue = DEFAULT_BOOLEAN,
            converter = BooleanOptionsConverter.class
    )
    public Boolean visualize;

    @Option(
            name = "visualizePlanner",
            help = "Enable planer visualization",
            defaultValue = DEFAULT_BOOLEAN,
            converter = BooleanOptionsConverter.class
    )
    public Boolean visualizePlanner;

    @Option(
            name = "loadout",
            abbrev = 'l',
            help = "The name of a vehicle loadout appearing in resources.json and capabilities.yaml",
            defaultValue = DEFAULT_STRING
    )
    public String loadout;

    @Option(
            name = "robotType",
            help = "type of robot - used to acquire parameters from robot params yaml file",
            defaultValue = DEFAULT_STRING)
    public String robotType;
    
    @Option(
            name = "dispatcher",
            abbrev = 'j',
            help = "Make this agent the dispatcher",
            defaultValue = DEFAULT_BOOLEAN,
            converter = BooleanOptionsConverter.class
    )
    public Boolean dispatcher;

    @Option(
            name = "isInSim",
            help = "We are running in simulation, vs. real-life",
            defaultValue = DEFAULT_BOOLEAN,
            converter = BooleanOptionsConverter.class
    )
    public Boolean isInSim;
    
    @Option(
            name = "deviceAsAgent",
            help = "Make device a full agent with planner, etc.",
            defaultValue = DEFAULT_BOOLEAN,
            converter = BooleanOptionsConverter.class
    )
    public Boolean deviceAsAgent;
    
    @Option(
            name = "aprilScan",
            abbrev = 's',
            help = "If the april tag scanning should be run",
            defaultValue = DEFAULT_BOOLEAN,
            converter = BooleanOptionsConverter.class
    )
    public Boolean aprilScan;

    @Option(
            name = "aprilTagRosHost",
            help = "Hostname or IP of ROS Bridge for April Tag scanning",
            defaultValue = DEFAULT_STRING
    )
    public String aprilTagRosHost;

    @Option(
            name = "aprilTagRosPort",
            help = "Port of ROS Bridge for April Tag scanning",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer aprilTagRosPort;


    @Option(
            name = "aprilTagTopic",
            help = "Topic to subscribe to for april tag detections",
            defaultValue = DEFAULT_STRING
    )
    public String aprilTagTopic;

    @Option(
            name = "navRosHost",
            help = "Hostname or IP of ROS Bridge for Navigation",
            defaultValue = DEFAULT_STRING
    )
    public String navRosHost;

    @Option(
            name = "navRosPort",
            help = "Port of ROS Bridge for Navigation",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer navRosPort;

    @Option(
            name = "subscriberListenPort",
            help = "Port for pubsub subscriptions",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer subscriberListenPort;



    @Option(
            name = "interval",
            abbrev = 'i',
            help = "Protelis update interval in milliseconds",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer interval;

    @Option(
            name = "netinterval",
            help = "How often (in milliseconds) to send Protelis network messages",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer netinterval;

    @Option(
            name = "moleConnectionString",
            help = "Connection String for the mole experimentation server",
            defaultValue = DEFAULT_STRING
    )
    public String moleConnectionString;

    // TODO: We do not have support for adjustable-pitch (gimbaled) sensors yet.
    @Option(
            name = "sensorPitch",
            help = "Fixed sensor pitch in integer degrees, with 0 being straight forward relative to the platform and 90 being straight down",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer sensorPitch;

    @Option(
            name = "vfov",
            help = "Vertical field of view of the sensor in degrees",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer vfov;

    @Option(
            name = "hfov",
            help = "Horizontal field of view of the sensor in degrees",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer hfov;

    @Option(
            name = "amor",
            help = "Uses amor as the NetworkManager",
            defaultValue = DEFAULT_BOOLEAN,
            converter = BooleanOptionsConverter.class
    )
    public Boolean amor;

    @Option(
            name = "debugAmor",
            help = "DEBUG ONLY: Debug specific configuration for amor",
            defaultValue = DEFAULT_BOOLEAN,
            converter = BooleanOptionsConverter.class
    )
    public Boolean debugAmor;

    @Option(
            name = "role",
            help = "DEBUG ONLY: amor device role {passive, dispatcher, mapper}",
            defaultValue = DEFAULT_STRING
    )
    public String role;

    @Option(
            name = "tactics",
            help = "JSON formatted description of applicable tactics",
            defaultValue = DEFAULT_STRING
    )
    public String tactics;

    @Option(
            name = "capabilities",
            help = "The environment variable containing platform capabilities",
            defaultValue = DEFAULT_STRING
    )
    public String capabilitiesVar;

    @Option(
            name = "advertise-ros-services",
            help = "Whether to wait for ROS bridge and advertise platform control when it's up",
            defaultValue = DEFAULT_BOOLEAN,
            converter = BooleanOptionsConverter.class
    )
    public Boolean advertiseRosServices;

    @Option(
            name = "hdopThreshold",
            help = "Threshold value for GPS HDOP; values below this threshold use GPS for lat/lon, values above use GPS for an anchor and ROS for further values",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer hdopThreshold;

    @Option(
            name = "numSatelliteThreshold",
            help = "Threshold value for number of GPS Satellites; values above this threshold use GPS for lat/lon, values below use GPS for an anchor and ROS for further values",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer numSatelliteThreshold;

    @Option(
            name = "numSolos",
            help = "How many Solo UAVs to include",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer numSolos;

    @Option(
            name = "numRovers",
            help = "How many rovers (UGVs) to include",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer numRovers;

    @Option(
            name = "numDownFacingSolos",
            help = "How many Solo UAVs with downward facing cameras to include",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer numDownFacingSolos;

    @Option(
            name = "numSkydios",
            help = "How many Skydio UAVs with gimballed cameras to include",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer numSkydios;

    @Option(
            name = "numIfos",
            help = "How many Ifo UAVs with to include",
            defaultValue = DEFAULT_INT,
            converter = IntegerOptionsConverter.class
    )
    public Integer numIfos;

    @Option(
            name = "aprilTagConfigFile",
            help = "XML file that holds the april tag signifigance table",
            defaultValue = DEFAULT_STRING
    )

    public String aprilTagConfigFile;

    public static final String DEFAULT_CCAST_PROPERTIES_FILE = "ccastConfig.properties";

    // This one is the only one that really shouldn't have a DEFAULT_STRING for a defaultValue
    @Option(
            name = "propertiesFile",
            help = "Filename of the properties file containing CCAST configuration values",
            defaultValue = DEFAULT_CCAST_PROPERTIES_FILE,
            allowMultiple = true
    )
    public List<String> propertiesFile;

    public Object get(String property) {
        try {
            Field field = this.getClass().getDeclaredField(property);
            field.setAccessible(true);
            Object ret = field.get(this);
            return ret;
        } catch (NoSuchFieldException e) {
            //logger.trace("Could not get " + this.getClass().getSimpleName() + " field: " + property);
        } catch (IllegalAccessException e) {
            //logger.trace("Could not access " + this.getClass().getSimpleName() + " field: " + property);
        }
        return null;
    }

}
