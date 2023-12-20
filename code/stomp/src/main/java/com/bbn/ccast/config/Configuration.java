//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.config;

import com.google.devtools.common.options.OptionsParser;
import org.apache.log4j.Logger;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.lang.reflect.Field;
import java.util.*;

public class Configuration {

	public static final String CCAST_CONFIG_PROPERTIES_FILE = "ccastConfig.properties";
	public static final String SIM_CONFIG_PROPERTIES_FILE = "sim.properties";
	public static final String LOCAL_DEVICE_PROPERTIES_FILE = "localDevice.properties";

    private static final Logger logger = Logger.getLogger(Configuration.class.getName());

	private static final String UID_PROPERTY = "uid";
	private static final String NUM_SOLOS_PROPERTY = "numSolos";
	private static final String NUM_ROVERS_PROPERTY = "numRovers";
	private static final String NUM_DOWN_FACING_SOLOS_PROPERTY = "numDownFacingSolos";
	private static final String NUM_SKYDIOS_PROPERTY = "numSkydios";
	private static final String NUM_IFOS_PROPERTY = "numIfos";
	private static final String NUM_ACCIPITERS_PROPERTY = "numAccipiters";
	private static final String PROTELIS_PROGRAM_FILE_PROPERTY = "protelisProgramFile";
	private static final String APRIL_TAG_CONFIG_FILE_PROPERTY = "aprilTagConfigFile";
	private static final String PROTELIS_LISTEN_ADDRESS_PROPERTY = "listenaddress";
	private static final String PROTELIS_LISTEN_PORT_PROPERTY = "listenport";
	private static final String PROTELIS_NETWORK_ROUND_PROPERTY = "netinterval";
	private static final String PROTELIS_STALE_ROUND_PROPERTY = "staleRounds";
	private static final String PROTELIS_NETWORK_DESTINATIONS_PROPERTY = "destinations";
	private static final String MAVLINK_FCU_PROPERTY = "mavlinkFCU";
	private static final String COT_ADDRESS_PROPERTY = "cotAddress";
	private static final String COT_PORT_PROPERTY = "cotPort";
	private static final String VFOV_PROPERTY = "vfov";
	private static final String HFOV_PROPERTY = "hfov";
	public static final String LOADOUT_PROPERTY = "loadout";
	private static final String SENSOR_PITCH_PROPERTY = "sensorPitch";
	private static final String NUM_SAT_THRESHOLD_PROPERTY = "numSatelliteThreshold";
	private static final String TACTICS_FILE_PROPERTY = "tactics";
	private static final String SPRINTER_TACTICS_FILE_PROPERTY = "sprinter_tactics";
	private static final String CAPABILITIES_PROPERTY = "capabilitiesFile";
	private static final String RESOURCES_PROPERTY = "resourcesFile";
	private static final String APRIL_SCAN_PROPERTY = "aprilScan";
	private static final String NAV_ROS_HOST_PROPERTY = "navRosHost";
	private static final String NAV_ROS_PORT_PROPERTY = "navRosPort";
	private static final String FORCE_TYPE_PROPERTY = "forceType";

	private static final String ADVERTISE_ROS_SERVICES_PROPERTY = "advertiseRosServices";
	private static final String DISPATCHER_PROPERTY = "dispatcher";
	private static final String MOLE_CONNECTION_STRING_PROPERTY = "moleConnectionString";
	private static final String APRIL_TAG_ROS_TOPIC_PROPERTY = "aprilTagRosTopic";
	private static final String APRIL_TAG_ROS_HOST = "aprilTagRosHost";
	private static final String APRIL_TAG_ROS_PORT = "aprilTagRosPort";
	private static final String APRIL_TAG_HTTP_LISTEN_PORT = "aprilTagHttpListenPort";
	private static final String APRIL_TAG_NEIGHBORHOOD_RADIUS = "aprilTagNeighborhoodRadius";
	private static final String APRIL_TAG_DISPATCHER_UPDATE_INTERVAL = "aprilTagDispatcherUpdateInterval";
	private static final String NUM_CAMS_PROPERTY = "numCams";
	private static final String PROTELIS_INTERVAL_PROPERTY = "interval";
	private static final String VISUALIZE_PROPERTY = "visualize";
	private static final String VISUALIZE_MISSION_PLANNER_PROPERTY = "visualizeMissionPlanner";
	private static final String VISUALIZE_MISSION_PLAN_ON_MAP_PROPERTY = "visualizeMissionPlanOnMap";
	private static final String VISUALIZE_TRACK_PLAN_PROPERTY = "visualizeTrackPlan";
	private static final String WORLDWIND_OFFLINE_MODE_PROPERTY = "worldwindOfflineMode";
	private static final String DEVICE_AS_AGENT_PROPERTY = "deviceAsAgent";
	private static final String IS_USING_WORLD_DB_PROPERTY = "isUsingWorldDb";
	private static final String VISUALIZE_PLANNER_PROPERTY = "visualizePlanner";
	private static final String OVERALL_LOGGING_LEVEL = "overallLoggingLevel";
	private static final String SPECIFIC_LOGGING_LEVELS = "specificLoggingLevels";
	private static final String LOGGING_CONFIG_FILE = "loggingConfigFile";
	private static final String LOGGING_PARENT_DIR = "loggingParentDirectory";
	private static final String CCAST_WORKING_DIRECTORY = "CCAST_WORKING_DIRECTORY";
	private static final String PLANNER_PARAMS_PROPERTY = "plannerParams";
	private static final String WORLD_REGIONS_FILE_PROPERTY = "worldRegionsFile";
	private static final String INITIAL_WORLD_REGION_PROPERTY = "initialWorldRegion";
	private static final String PLATFORM_START_POSITIONS_FILE_PROPERTY = "platformStartPositionsFile";
	private static final String START_SITL_SCRIPT_PROPERTY = "startSitlScript";
	private static final String STOP_SITL_SCRIPT_PROPERTY = "stopSitlScript";
	private static final String SHOW_DEBUGGING_VISUALIZATION_IN_SIM_PROPERTY = "showDebuggingVisualizationInSim";
	public static final String DISPATCHER_ID_PROPERTY = "dispatcherID";
	public static final String DISPATCHER_HOST_ADDR_PROPERTY = "dispatcherHostAddr";
	public static final String DISPATCHER_PORT_PROPERTY = "dispatcherPort";
	public static final String FTP_MAP_SERVER_PORT_PROPERTY = "ftpMapServerPort";
	public static final String FTP_MAP_SERVER_DIR = "ftpMapServerDir";
	public static final String DOCKING_STORAGE_FILE_PROPERTY = "dockingStorageFile";
	public static final String USE_RAW_GPS_POSITION_ESTIMATE = "useRawGpsPositionEstimate";
	private static final String ROBOT_PARAMS_FILE_PROPERTY = "robotParamsFile";
	private static final String ROBOT_TYPE_PROPERTY = "robotType";
	private static final String SENSOR_PARAMS_FILE_PROPERTY = "sensorParamsFile";
	public static final String NAV_FRAME_CONTROLLER_PROPERTY = "navFrameController";

	private static final String PRIMARY_AIRSIM_HOST_PROPERTY = "primaryAirSimHost";
	private static final String PRIMARY_AIRSIM_PORT_PROPERTY = "primaryAirSimPort";
	private static final String SECONDARY_AIRSIM_HOST_PROPERTY = "secondaryAirSimHost";
	private static final String SECONDARY_AIRSIM_PORT_PROPERTY = "secondaryAirSimPort";

	public static final String CENTER_MAP_LAT_PROPERTY = "centerMapLat";
	public static final String CENTER_MAP_LON_PROPERTY = "centerMapLon";
	public static final String CENTER_MAP_ALT_PROPERTY = "centerMapAlt";

	public static final String INITIAL_AGENT_LAT_PROPERTY = "initialAgentLat";
	public static final String INITIAL_AGENT_LON_PROPERTY = "initialAgentLon";
	
	public static final String AT_CLUSTER_ID_LOWER_BOUND = "atClusterIdLowerBound";

	public static final String DISPATCHER_POSITION_PROPERTY = "dispatcherPosition";

	public static final String IS_IN_SIM_PROPERTY = "isInSim";
	public static final String ADVERTISE_LOOPBACK_IPS_PROPERTY = "advertiseLoopbackIps";
	public static final String JVM_PER_AGENT = "jvmPerAgent";

	public static final String IS_BLE_ENABLED_PROPERTY = "isBleEnabled";
	public static final String IS_MESSAGE_COMPRESSION_ENABLED_PROPERTY = "isMessageCompressionEnabled";
	private static final String BLE_AUTO_TASKING = "bleAutoTasking";

	private static final String PUB_SUB_LISTEN_PORT = "pubSubListenPort";
	private static final String SUBSCRIBER_LISTEN_PORT = "subscriberListenPort";
	private static final String STANDALONE_RTK_TRANSMITTER_COMMS_PORT = "standaloneRtkCommsPort";
	private static final String USE_STANDALONE_RTK="useStandaloneRtk";
	private static final String RTK_COMMS_SERVER_IP = "rtkCommsServerHost";
	private static final String FLIGHT_PARAMS_FILE_PROPERTY = "flightParamsFile";
	public static final String LOGGING_TELEM = "logJsonTelem";
	private static final String SAVED_RUN_DIRECTORY = "savedRunDirectory";
	private static final String LOG_TELEM_USAGE_IN_SIM = "logTelemNetworkUsageInSim";
	public static final String PRIMARY_DISPATCHER_ADDRESS = "primaryDispatcherAddress";
	private static final String IS_SECONDARY_DISPATCHER = "isSecondaryDispatcher";
	private static final String WINDOW_NAME_PROPERTY = "windowName";
	private static final String SIM_AGENT_PREFIX = "agentPrefix";
	private static final String SECONDARY_DISPATCHER_ADDRESS_PROPERTY = "secondaryDispatcherAddresses";
	private static final String RTK_SERIAL_DEVICE_PROPERTY = "rtkSerialDevice";
	private static final String RTK_ACC_REQ_METERS_PROPERTY = "rtkSurveyAccuracyRequirementMeters";
	private static final String RTK_DUR_REQ_SEC_PROEPRTY = "rtkSurveyDurationRequirementSeconds";
	private static final String ROVER_MAV_PROXY_PORT = "roverMavProxyPort";
	private static final String MY_LVC_TELEM_PORT = "myLvcTelemPort";
	private static final String LVC_MODE_PROPERTY = "lvcDistributedSim";
	public static final String RECEIVE_LVC_TELEM = "receiveLvcTelem";
	private static final String FORWARD_LVC_TELEM = "forwardLvcTelem";
	private static final String STANDALONE_RTK_BASE_STATION_IP = "standaloneRtkBaseStationIp";
	private static final String MQTT_BROKER_PROPERTY = "mqttBroker";
	private static final String MQTT_USER_PROPERTY = "mqttUser";
	private static final String MQTT_PASSWORD_PROPERTY = "mqttPassword";




	private static String JSON_TELEM_PROPERTY = "sendJsonTelem";
	private static final String JSON_TELEM_ADDRESS_PROPERTY = "jsonTelemAddress";
	private static final String JSON_TELEM_PORT_PROPERTY = "jsonTelemPort";

	private static final String STOMP_DEBUG_PROPERTY = "debugToolBar";
	
	public static final String BATTERY_THRESHOLD_BASE_PROPERTY = "batteryThresholdBase";
	public static final String BATTERY_THRESHOLD_SCALE_PROPERTY = "batteryThresholdScale";
	public static final String REPLACEMENT_BATTERY_THRESHOLD_OFFSET_PROPERTY = "replacementBatteryThresholdOffset";

	public static final String GOTO_LOOK_AROUND_DISTANCE_PROPERTY = "gotoLookAroundDistance";

	public static final String LAND_IN_AREA_SEPARATION_PROPERTY = "landInAreaSeparation";

	public static final String DISPATCHER_ASSIGNED_TAKEOFF_ALTITUDES = "dispatcherAssignedTakeoffAltitudes";
	
	public static final String USE_BBN_ROS_BRIDGE = "useBbnRosBridge";
	private static final String BBN_ROS_PORT_PROPERTY = "bbnRosPort";

	public static final String EXECUTION_TOOLBAR_PATH_PROPERTY ="executionToolBarPath";
	public static final String DEBUG_TOOLBAR_PATH_PROPERTY ="debugToolBarPath";

	public static final String NETWORK_INTERFACE_PROPERTY = "networkInterface";

	public static final String LTE_AT_CMD_TERMINAL_DEVICE_NAME = "atCmdDeviceName";

	public static final String EXPERIMENT_DISPATCHER_COMMAND_JSON = "experimentDispatcherCommandJson";
	public static final String EXPERIMENT_MAX_MILLIS_TO_RUN = "experimentMaxMillisToRun";
	public static final String IS_LIDAR_ENABLED = "isLidarEnabled";	

	private static final String SIM_APRIL_TAG_FILE_PROPERTY = "simAprilTags";
	public static final String EXPLORATION_TEMPORARY_MAP_STORAGE_DIRECTORY = "explorationTempMapDir";

	public static final String MIN_DISTANCE_FROM_HOME_TO_RTL_PROPERTY = "minRtlDist";
	public static final String BLE_INTERACTION_RADIUS_PROPERTY = "bleInteractionRadius";

	public static final String COMMS_LOSS_TIMEOUT_MS_PROPERTY = "commsLossTimeoutMs";
	public static final String HAZARDS_ENABLED_PROPERTY = "hazardsEnabled";
	
	public static final String IS_TIME_SCALABLE = "isTimeScalable";
	public static final String INITIAL_TIME_SCALE = "initialTimeScale";


	public static final String SOLO_START_POSITION_INDEX_PROPERTY = "soloStartPositionIndex";
	public static final String SOLO_END_POSITION_INDEX_PROPERTY = "soloEndPositionIndex";
	public static final String ROVER_START_POSITION_INDEX_PROPERTY = "roverStartPositionIndex";
	public static final String ROVER_END_POSITION_INDEX_PROPERTY = "roverEndPositionIndex";

	private CmdOptions cmdOptions;
	private Properties properties;

	public static final String REGIONPROPERTIESFILEPROPERTY = "regionPropertiesFile";
	public static final String BLE_SIM_ENABLED = "ble_sim";

//	private static final Map<String, TacticConfig> tacticConfigMap = new TreeMap<>(String.CASE_INSENSITIVE_ORDER);

	public Configuration() {
	}

	private static Configuration instance = null;

	public static Configuration instance() {
		if (instance == null) {
			instance = new Configuration();
		}
		return instance;
	}

	public static CmdOptions parseCmdOptions(String[] args) {
		OptionsParser parser = OptionsParser.newOptionsParser(CmdOptions.class);
		parser.parseAndExitUponError(args);
		CmdOptions options = parser.getOptions(CmdOptions.class);
		return options;
	}

	public Configuration setCmdOptions(CmdOptions cmdOptions) {
		this.cmdOptions = cmdOptions;
		return this;
	}

	public Configuration setProperty(String key, String value) {
		if (this.properties == null) {
			this.properties = new Properties();
		}
		this.properties.setProperty(key, value);
		return this;
	}

	/**
	 * Add a set of properties to our set.
	 * 
	 * @param propertiesToAdd
	 *            The properties to add.
	 * @param abortIfNonOverrides
	 *            If true, log error and exit process entirely if any of the
	 *            properties in {@code propertiesToAdd} are entirely new properties.
	 *            (This allows us to ensure that local overlays contain only
	 *            overrides, which can be helpful in identifying typos.)
	 * @return
	 */
	public Configuration addProperties(Properties propertiesToAdd, boolean abortIfNonOverrides) {
		
		boolean fatalErrors = false;
		StringBuilder errorReport = new StringBuilder(
				"The following properties are not overrides. Check for typos and try again.\n");

		if (this.properties == null)
			this.properties = new Properties();
		for (String key : propertiesToAdd.stringPropertyNames()) {
			Object originalValue = properties.setProperty(key, propertiesToAdd.getProperty(key));
			if (abortIfNonOverrides && originalValue == null) {
				errorReport.append("  ").append(key).append("\n");
				fatalErrors = true;
			}
		}
		
		if (fatalErrors) {
		
			logger.error(errorReport);
			logger.error("   !!!    Aborting due to apparent misconfiguration.");
			System.exit(1);
		}
		
		instance = this;
		return this;
	}

	public CmdOptions getCmdOptions() {
		return this.cmdOptions;
	}

	public Properties getProperties() {
		return this.properties;
	}

	public Configuration setPropertiesFile(String propertiesFile) throws IOException {
		this.addProperties(loadProperties(propertiesFile), false);
		return this;
	}

	public static Properties loadProperties(String propertiesFile) throws IOException {
		String ccastConfigPath = propertiesFile;
		Properties props = new Properties();
		try (FileInputStream inStream = new FileInputStream(ccastConfigPath)) {
			props.load(inStream);
		}
		return props;
	}

	private String getPropertyOrCmdOption(String property) {
		// Get value from the properties file
		if (properties != null) {
			String ret = properties.getProperty(property);
			// Overload it if we got it on the command line
			if (cmdOptions != null) {
				Object fromCmdOptions = cmdOptions.get(property);
				if (fromCmdOptions != null && !fromCmdOptions.toString().isEmpty()
						&& !isDefault(fromCmdOptions.toString())) {
					ret = fromCmdOptions.toString();
				}
			}
			return ret;
		}
		return null;
	}

	public static boolean isDefault(String val) {
		if (val == null || CmdOptions.DEFAULT_STRING.equalsIgnoreCase(val)
				|| CmdOptions.DEFAULT_INT.equalsIgnoreCase(val)) {
			return true;
		}
		return false;
	}

	public String getUid() {
		return getStringProperty(UID_PROPERTY);
	}
	
	public Integer getNumSolos() {
		return getIntProperty(NUM_SOLOS_PROPERTY);		
	}

	public Integer getNumDownFacingSolos() {
		return getIntProperty(NUM_DOWN_FACING_SOLOS_PROPERTY);		
	}

	public Integer getNumRovers() {
		return getIntProperty(NUM_ROVERS_PROPERTY);		
	}

	public String getForceType() {
		return getStringProperty(FORCE_TYPE_PROPERTY);
	}
	
	public Integer getNumAccipiters() {
		return getIntProperty(NUM_ACCIPITERS_PROPERTY);
	}

	public Integer getNumSkydios() {
		return getIntProperty(NUM_SKYDIOS_PROPERTY);		
	}

	public Integer getNumIfos() {
		return getIntProperty(NUM_IFOS_PROPERTY);
	}

	public File getProtelisProgramFile() {
		File protelisProgramFile = getFileProperty(PROTELIS_PROGRAM_FILE_PROPERTY);
		if (protelisProgramFile == null) {
			protelisProgramFile = getFileFromFilename(cmdOptions.file);
		}
		return protelisProgramFile;
	}

	public File getAprilTagConfigFile() {
		return getFileProperty(APRIL_TAG_CONFIG_FILE_PROPERTY);
	}

	public String getListenAddress() {
		return getStringProperty(PROTELIS_LISTEN_ADDRESS_PROPERTY);
	}

	public Integer getListenPort() {
		return getIntProperty(PROTELIS_LISTEN_PORT_PROPERTY);
	}

	public Integer getNetInterval() {
		return getIntProperty(PROTELIS_NETWORK_ROUND_PROPERTY);
	}

	public Integer getStaleRounds() {
		return getIntProperty(PROTELIS_STALE_ROUND_PROPERTY);
	}

	public String getDestinations() {
		return getStringProperty(PROTELIS_NETWORK_DESTINATIONS_PROPERTY);
	}

	public String getMavlinkFCU() {
		return getStringProperty(MAVLINK_FCU_PROPERTY);
	}
	
	public Double getGotoLookAroundDistance() {
		return getDoubleProperty(GOTO_LOOK_AROUND_DISTANCE_PROPERTY);
	}

	public Boolean shouldDispatcherAssignedTakeoffAltitudes() {
		return getBooleanProperty(DISPATCHER_ASSIGNED_TAKEOFF_ALTITUDES);
	}
	
	public String getCotAddress() {
		return getStringProperty(COT_ADDRESS_PROPERTY);
	}

	public Integer getCotPort() {
		return getIntProperty(COT_PORT_PROPERTY);
	}

	public Boolean getSendJsonTelem() {
		return getBooleanProperty(JSON_TELEM_PROPERTY);
	}

	public Boolean getUseBbnRosBridge(){
		return getBooleanProperty(USE_BBN_ROS_BRIDGE);
	}

	public Boolean getBleAutoTasking(){
		return getBooleanProperty(BLE_AUTO_TASKING);
	}

	public String[] getJsonAddresses() {
		String addrStr = getStringProperty(JSON_TELEM_ADDRESS_PROPERTY);
		if (addrStr == null){
			return new String[0];
		}
		return addrStr.split(",");
	}

	public String getSecondaryAddressessAsString() {
		String addrStr = getStringProperty(SECONDARY_DISPATCHER_ADDRESS_PROPERTY);
		return addrStr;
	}

	public Integer getJsonPort() {
		return getIntProperty(JSON_TELEM_PORT_PROPERTY);
	}

	public Double getVFOV() {
		return getDoubleProperty(VFOV_PROPERTY);
	}

	public Double getHFOV() {
		return getDoubleProperty(HFOV_PROPERTY);
	}

	public String getVehicleLoadout() {
		return getStringProperty(LOADOUT_PROPERTY);
	}

	public Integer getSensorPitch() {
		return getIntProperty(SENSOR_PITCH_PROPERTY);
	}

	public Integer getNumSatelliteThreshold() {
		return getIntProperty(NUM_SAT_THRESHOLD_PROPERTY);
	}

	public File getTacticsFile() {
		return getFileProperty(TACTICS_FILE_PROPERTY);
	}
	public File getSprinterTacticsFile() {
		return getFileProperty(SPRINTER_TACTICS_FILE_PROPERTY);
	}
	public File getSimAprilTagFile() { return getFileProperty(SIM_APRIL_TAG_FILE_PROPERTY) ;}

	public File getCapabilitiesFile() {
		return getFileProperty(CAPABILITIES_PROPERTY);
	}

	public String getLoadout() {
		return getStringProperty(LOADOUT_PROPERTY);
	}

	public File getResourcesFile() {
		return getFileProperty(RESOURCES_PROPERTY);
	}

	public Boolean shouldAprilScan() {
		return getBooleanProperty(APRIL_SCAN_PROPERTY);
	}

	public String getNavRosHost() {
		return getStringProperty(NAV_ROS_HOST_PROPERTY);
	}

	/**
	 * Gets the base Rosbridge port. NB the port that will actually be used is
	 * always the same IRL (the port returned by this method), but in sim different
	 * ports per platform are used.
	 * 
	 * TODO rename this rosBridgePort - actually baseRosBridgePort
	 * 
	 * @return the port.
	 */
	public Integer getNavRosPort() {
		return getIntProperty(NAV_ROS_PORT_PROPERTY);
	}

	public Integer getBbnRosPort() {
		return getIntProperty(BBN_ROS_PORT_PROPERTY);
	}

	public Boolean shouldAdvertiseRosServices() {
		return getBooleanProperty(ADVERTISE_ROS_SERVICES_PROPERTY);
	}

	public Boolean isDispatcher() {
		return getBooleanProperty(DISPATCHER_PROPERTY);
	}

	public String getDispatcherID() {
		return getStringProperty(DISPATCHER_ID_PROPERTY);
	}

	public String getRegionFileString() {return getStringProperty(REGIONPROPERTIESFILEPROPERTY);}

	public String getMoleConnectionString() {
		return getStringProperty(MOLE_CONNECTION_STRING_PROPERTY);
	}

	public String getAprilTagTopic() {
		return getStringProperty(APRIL_TAG_ROS_TOPIC_PROPERTY);
	}

	public String getAprilTagRosHost() {
		return getStringProperty(APRIL_TAG_ROS_HOST);
	}

	public Integer getAprilTagRosPort() {
		return getIntProperty(APRIL_TAG_ROS_PORT);
	}

	public Integer getAprilTagHttpListenPort() { return getIntProperty(APRIL_TAG_HTTP_LISTEN_PORT); }

	public Double getAprilTagNeighborhoodRadius() {
		return getDoubleProperty(APRIL_TAG_NEIGHBORHOOD_RADIUS);
	}

	public Integer getAprilTagDispatcherUpdateInterval() {
		return getIntProperty(APRIL_TAG_DISPATCHER_UPDATE_INTERVAL);
	}

	public String getRtkSerialDevice(){
		return getStringProperty(RTK_SERIAL_DEVICE_PROPERTY);
	}

	public double getRtkSurveyAccReqMeters(){
		Double val = getDoubleProperty(RTK_ACC_REQ_METERS_PROPERTY);
		if (val == null){
			val = 2.0;
		}
		return val;
	}

	public int getRtkSurveyDurReqSec(){
		Integer val = getIntProperty(RTK_DUR_REQ_SEC_PROEPRTY);
		if (val == null){
			val = 180;
		}
		return val;
	}

	public int getNumCams() {
		Integer num = getIntProperty(NUM_CAMS_PROPERTY);
		if (num == null) {
			return 1;
		}
		return num;
	}

	public Integer getInterval() {
		return getIntProperty(PROTELIS_INTERVAL_PROPERTY);
	}

	public Boolean deviceAsAgent() {
		return getBooleanProperty(DEVICE_AS_AGENT_PROPERTY);
	}

	public Boolean isUsingWorldDb() {
		return getBooleanProperty(IS_USING_WORLD_DB_PROPERTY);
	}

	public Boolean visualizePlanner() {
		return getBooleanProperty(VISUALIZE_PLANNER_PROPERTY);
	}

	public Boolean visualize() {
		return getBooleanProperty(VISUALIZE_PROPERTY);
	}

    public Boolean visualizeMissionPlanner() {
    		return getBooleanProperty(VISUALIZE_MISSION_PLANNER_PROPERTY);
    	}

    public Boolean visualizeMissionPlanOnMap() {
		return getBooleanProperty(VISUALIZE_MISSION_PLAN_ON_MAP_PROPERTY);
	}

    public Boolean visualizeTrackPlan() {
		return getBooleanProperty(VISUALIZE_TRACK_PLAN_PROPERTY);
	}

    public Boolean getWorldwindOfflineMode() {
    		return getBooleanProperty(WORLDWIND_OFFLINE_MODE_PROPERTY);
    	}

	public boolean showDebuggingVisualizationInSim() {
		return getBooleanProperty(SHOW_DEBUGGING_VISUALIZATION_IN_SIM_PROPERTY);
	}

	public Boolean stompDebugToolBar() {
		return getBooleanProperty(STOMP_DEBUG_PROPERTY);
	}

	public String getWorkingDirectory() {
		return getStringProperty(CCAST_WORKING_DIRECTORY);
	}

	public String getOverallLoggingLevel() {
		return getStringProperty(OVERALL_LOGGING_LEVEL);
	}

	/**
	 * Gets the specific logging levels from the config file and parses them to make
	 * a map of loggers to their specific levels
	 *
	 * @return a map of logger names to their specified level
	 */
	public Map<String, String> getSpecificLoggingLevels() {
		String levelsString = getStringProperty(SPECIFIC_LOGGING_LEVELS);
		if (levelsString != null) {
			HashMap<String, String> result = new HashMap<>();
			String[] splitCommas = levelsString.split(",");
			for (String s : splitCommas) {
				String[] splitColon = s.split(":");
				if (!splitColon[0].trim().isEmpty() && !splitColon[1].trim().isEmpty()) {
					result.put(splitColon[0].trim(), splitColon[1].trim());
				}
			}
			return result;
		}
		return null;
	}

	public String getPrimaryAirSimHost() {
		return getStringProperty(PRIMARY_AIRSIM_HOST_PROPERTY);
	}

	public Integer getPrimaryAirSimPort() {
		return getIntProperty(PRIMARY_AIRSIM_PORT_PROPERTY);
	}

	public String getSecondaryAirSimHost() {
		return getStringProperty(SECONDARY_AIRSIM_HOST_PROPERTY);
	}

	public Integer getSecondaryAirSimPort() {
		return getIntProperty(SECONDARY_AIRSIM_PORT_PROPERTY);
	}

	public File getLoggingConfigFile() {
		return getFileProperty(LOGGING_CONFIG_FILE);
	}

	public String getLoggingParentDir() {
		return getStringProperty(LOGGING_PARENT_DIR);
	}

	public File getPlannerParamFile() {
		return getFileProperty(PLANNER_PARAMS_PROPERTY);
	}

	public File getWorldRegionsFile() {
		return getFileProperty(WORLD_REGIONS_FILE_PROPERTY);
	}

	public Integer getAtClusterIdLowerBound() {
		return getIntProperty(AT_CLUSTER_ID_LOWER_BOUND);
	}
	
	public String getInitialWorldRegion() {
		return getStringProperty(INITIAL_WORLD_REGION_PROPERTY);
	}

	public File getPlatfromStartPositionsFile() {
		return getFileProperty(PLATFORM_START_POSITIONS_FILE_PROPERTY);
	}

	public String getRobotType() {
		return getStringProperty(ROBOT_TYPE_PROPERTY);
	}

	public File getRobotParamsFile() {
		return getFileProperty(ROBOT_PARAMS_FILE_PROPERTY);
	}

	public File getSensorParamsFile() {
		return getFileProperty(SENSOR_PARAMS_FILE_PROPERTY);
	}

	public String getStartSitlScript() {
		return getStringProperty(START_SITL_SCRIPT_PROPERTY);
	}

	public String getStopSitlScript() {
		return getStringProperty(STOP_SITL_SCRIPT_PROPERTY);
	}

	public String getNavFrameController() { return getStringProperty(NAV_FRAME_CONTROLLER_PROPERTY); }

	public Boolean isInSim() {
		return getBooleanProperty(IS_IN_SIM_PROPERTY);
	}

	public Boolean advertiseLoopback() {
		return getBooleanProperty(ADVERTISE_LOOPBACK_IPS_PROPERTY);
	}
	
	public Boolean isJvmPerAgent() {
		return getBooleanProperty(JVM_PER_AGENT);
	}

	public Boolean isBleEnabled() {
		return getBooleanProperty(IS_BLE_ENABLED_PROPERTY);
	}

	public Boolean isHazardsEnabled() {return getBooleanProperty(HAZARDS_ENABLED_PROPERTY);}

	public Boolean isMessageCompressionEnabled() {
		return getBooleanProperty(IS_MESSAGE_COMPRESSION_ENABLED_PROPERTY);
	}

	public Integer getPubSubManagerListenPort() {
		return getIntProperty(PUB_SUB_LISTEN_PORT);
	}

	public Integer getSubscriberListenPort() {
		return getIntProperty(SUBSCRIBER_LISTEN_PORT);
	}

	public Integer getStandaloneRtkCommsPort(){
		return getIntProperty(STANDALONE_RTK_TRANSMITTER_COMMS_PORT);
	}

	public String getNetInterface() { return getStringProperty(NETWORK_INTERFACE_PROPERTY); }

	public String getFlightParamsFileName() {return getStringProperty(FLIGHT_PARAMS_FILE_PROPERTY);}

	public String getExplorationTempMapDir(){
		String dir = getStringProperty(EXPLORATION_TEMPORARY_MAP_STORAGE_DIRECTORY);
		if (dir == null){
			//Default to /tmp/
			return "/tmp/";
		}
		return dir;
	}

	// Helpers ///////////////////////////////////////////////////////////////////

	public String getStringProperty(String property) {
		return getPropertyOrCmdOption(property);
	}

	public Integer getIntProperty(String property) {
		return intOrNull(getPropertyOrCmdOption(property));
	}

	public Long getLongProperty(String property) {
		return longOrNull(getPropertyOrCmdOption(property));
	}

	public File getFileFromFilename(String filenameFromProperty) {
		String workingDir = getWorkingDirectory();
		File fromProperty = new File(filenameFromProperty);
		if (workingDir != null && !workingDir.isEmpty() && !fromProperty.isAbsolute()) {
			return new File(workingDir, filenameFromProperty);
		}

		return fromProperty;
	}

	public Boolean getBooleanProperty(String property) {
		return boolOrNull(getPropertyOrCmdOption(property));
	}

	public Double getDoubleProperty(String property) {
		return doubleOrNull(getPropertyOrCmdOption(property));
	}

	public List<File> getFilePropertyList(String property) {
		String filenamesFromProperty = getStringProperty(property);
		if (filenamesFromProperty != null && !filenamesFromProperty.isEmpty()) {
			String[] values = filenamesFromProperty.split("\\s*,\\s*");
			if (values.length > 0) {
				ArrayList<File> files = new ArrayList<File>();
				for (int i = 0; i < values.length; i++) {
					File f = getFileFromFilename(values[i]);
					if (f != null) {
						files.add(f);
					} else {
						logger.warn("Invalid file, " + values[i] + ", from property " + property);
					}
				}
				return files;
			}
		}
		return null;
	}

	public File getFileProperty(String property) {
		String filenameFromProperty = getStringProperty(property);
		if (filenameFromProperty != null) {
			return getFileFromFilename(filenameFromProperty);
		}
		return null;
	}

	public static final Integer intOrNull(String i) {
		if (i == null)
			return null;
		try {
			return Integer.parseInt(i);
		} catch (Exception e) {
			/* ignore */ }
		return null;
	}

	public static final Double doubleOrNull(String d) {
		if (d == null)
			return null;
		try {
			return Double.parseDouble(d);
		} catch (Exception e) {
			/* ignore */ }
		return null;
	}

	public static final Long longOrNull(String l) {
		if (l == null)
			return null;
		try {
			return Long.parseLong(l);
		} catch (Exception e) {
			/* ignore */ }
		return null;
	}

	public static final Boolean boolOrNull(Object b) {
		if (b == null)
			return null;
		try {
			if (b instanceof String) {
				return Boolean.parseBoolean((String) b);
			} else if (b instanceof Boolean) {
				return (Boolean) b;
			}
		} catch (Exception e) {
			/* ignore */ }
		return null;
	}

	public static String parseAsString(Object val) {
		// TODO:: does this make sense?
		if (val == null) {
			return null;
		} else if (val instanceof String) {
			return (String) val;
		}
		return val.toString();
	}

	public static double parseAsDouble(Object val) {
		if (val instanceof Number) {
			return ((Number) val).doubleValue();
		} else if (val instanceof String) {
			return Double.parseDouble((String) val);
		}
		return Double.NaN;
	}

	public String getLteAtCmdTerminalDeviceName() {
		return getStringProperty(LTE_AT_CMD_TERMINAL_DEVICE_NAME);
	}

	public String[] getExperimentDispatcherCommandsJson() {
		return new String[] { getStringProperty(EXPERIMENT_DISPATCHER_COMMAND_JSON) };
	}

	public int getExperimentMaxMillisToRun() {
		return getIntProperty(EXPERIMENT_MAX_MILLIS_TO_RUN);
	}

	public boolean isLidarEnabled() {
		return getBooleanProperty(IS_LIDAR_ENABLED);
	}

	public boolean isTimeScalable() {
		return getBooleanProperty(IS_TIME_SCALABLE);
	}

	public double getInitialTimeScale() {
		return getDoubleProperty(INITIAL_TIME_SCALE);
	}

	public Boolean isBleSim() {
		return getBooleanProperty(BLE_SIM_ENABLED);
	}


	public boolean getShouldLogTelem() {
		return getBooleanProperty(LOGGING_TELEM) != null ? getBooleanProperty(LOGGING_TELEM) : false;
	}

	public void setShouldLogTelem(Boolean shouldLogTelem) {
		setProperty(LOGGING_TELEM, shouldLogTelem.toString());
	}
	
    public File getRunStateFromFolder() {
		return getFileProperty(SAVED_RUN_DIRECTORY);
    }

//    public void addTacticConfig(String tacticName, TacticConfig info) {
//		tacticConfigMap.put(tacticName,info);
//	}
//
//    public TacticConfig getTacticConfigFromName(String tacticName) {
//		return tacticConfigMap.get(tacticName);
//	}
//
//	public Map<String, TacticConfig> getTacticConfigMap(){
//		return tacticConfigMap;
//	}

	public boolean shouldLogTelemNetUsageInSim() {
		Boolean shouldlog = getBooleanProperty(LOG_TELEM_USAGE_IN_SIM);
		return shouldlog == null ? false : shouldlog;
	}

    public boolean isSecondaryDispatcher() {
		Boolean b = getBooleanProperty(IS_SECONDARY_DISPATCHER);
		return b == null ? false : b;
    }

	public String getWindowName() {
		 if(getStringProperty(WINDOW_NAME_PROPERTY) == null) {
			return null;
		}
		 return getStringProperty(WINDOW_NAME_PROPERTY);
	}

	public String getAgentPrefix() {
		if(getStringProperty(SIM_AGENT_PREFIX) == null) {
			return "";
		}
		return getStringProperty(SIM_AGENT_PREFIX);
	}


    public int getRoverMavProxyPort() {
		Integer port =getIntProperty(ROVER_MAV_PROXY_PORT);
		if (port == null){
			port = 5762;
		}
		return port;
    }

	public boolean useStandaloneRtk() {
		Boolean val = getBooleanProperty(USE_STANDALONE_RTK);
		if (val == null){
			val = false;
		}
		return val;
	}

	public String getMqttBroker(){
		return getStringProperty(MQTT_BROKER_PROPERTY);
	}

	public String getMqttUsername() {
		return getStringProperty(MQTT_USER_PROPERTY);
	}

	public String getMqttPassword() {
		return getStringProperty(MQTT_PASSWORD_PROPERTY);
	}

    public Integer getMyLvcTelemPort() {
		return getIntProperty(MY_LVC_TELEM_PORT);
	}
	
	public Boolean getLvcDistributedSim() {
		return getBooleanProperty(LVC_MODE_PROPERTY);
	}

	public Boolean receiveLvcTelem() {
		return getBooleanProperty(RECEIVE_LVC_TELEM);
	}

	public boolean forwardLvcTelem() {
		return getBooleanProperty(FORWARD_LVC_TELEM);
	}

	public Integer getSoloStartPositionIndex() {return getIntProperty(SOLO_START_POSITION_INDEX_PROPERTY);}

	public Integer getSoloEndPositionIndex() {return getIntProperty(SOLO_END_POSITION_INDEX_PROPERTY);}

	public Integer getRoverStartPositionIndex() {return getIntProperty(ROVER_START_POSITION_INDEX_PROPERTY);}

	public Integer getRoverEndPositionIndex() {return getIntProperty(ROVER_END_POSITION_INDEX_PROPERTY);}

	public String getRtkBaseStationIp() {
		return getStringProperty(STANDALONE_RTK_BASE_STATION_IP);
    }

	public String getRtkCommsServerIp() {
		return getStringProperty(RTK_COMMS_SERVER_IP);
	}
	
	/**
	 * Check that all configuration properties have at least default values
	 * assigned, logging any unassigned properties.
	 * 
	 * @param haltIfInvalid
	 *            If true, exit the entire process if we detect an invalid state.
	 * 
	 * @return true if no errors detected.
	 */
	public boolean validate(boolean haltIfInvalid) {

		boolean clean = true;

		Class<? extends Configuration> myClass = getClass();
		List<String> badPropsBadProps = new LinkedList<String>();

		// Iterate over all the Strings we've got, making sure they're defined
		for (Field field : myClass.getDeclaredFields()) {

			if (field.getType().equals(String.class)) {
				try {
					// We assume all String fields are property names
					String propertyName = (String) field.get(this);

					// For any property without even a default value, sound the alarm
					if (getPropertyOrCmdOption(propertyName) == null) {
						badPropsBadProps.add(propertyName);
					}
				} catch (IllegalArgumentException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (IllegalAccessException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}

		if (badPropsBadProps.size() == 0) {
			logger.info("All configuration properties defined.");
		} else {
			logger.error("The following configuration properties are undefined:");
			for (String propertyName : badPropsBadProps) {
				logger.error(propertyName);
			}
			if (haltIfInvalid) {
				logger.error("   !!!  Aborting due to apparent code/data mismatch");
				System.exit(1);
			}
		}

		return clean;
	}


	public static Configuration loadArgsAndProperties(String[] args) {
		// Get configuration
		CmdOptions options = Configuration.parseCmdOptions(args);
		Configuration configuration = null;
		try {
			configuration = new Configuration();
			configuration.setCmdOptions(options);

			// .setPropertiesFile(options.propertiesFile);
			List<String> propertiesFiles = options.propertiesFile;
			if (propertiesFiles != null && !propertiesFiles.isEmpty()) {
				for (String propFile : propertiesFiles) {
					System.out.println("Loading properties file: " + propFile);
					configuration.addProperties(Configuration.loadProperties(propFile), false);
				}
			} else {
				System.out.println("Loading DEFAULT CCAST properties file: " + CCAST_CONFIG_PROPERTIES_FILE);
				configuration.addProperties(Configuration.loadProperties(CCAST_CONFIG_PROPERTIES_FILE), false);

				/*
				 * Note: we load localDevice.properties once here just to get sim and/or region
				 * props, then re-load to override anything we might want to override from those
				 * two prop files
				 */
				System.out.println("Loading DEFAULT local properties file: " + LOCAL_DEVICE_PROPERTIES_FILE);
				configuration.addProperties(Configuration.loadProperties(LOCAL_DEVICE_PROPERTIES_FILE), true);

				if (configuration.isInSim()) {
					System.out.println("Loading DEFAULT sim properties file: " + SIM_CONFIG_PROPERTIES_FILE);
					configuration.addProperties(Configuration.loadProperties(SIM_CONFIG_PROPERTIES_FILE), false);

					// Now we need to re-overlay localDevice.properties
					System.out.println("Re-loading DEFAULT local properties file: " + LOCAL_DEVICE_PROPERTIES_FILE);
					configuration.addProperties(Configuration.loadProperties(LOCAL_DEVICE_PROPERTIES_FILE), true);
				}

				String regionPropertiesFilename = configuration.getRegionFileString();
				if (regionPropertiesFilename != null && !regionPropertiesFilename.isEmpty()) {
					System.out.println("Loading DEFAULT region properties file: " + regionPropertiesFilename);
					configuration.addProperties(Configuration.loadProperties(regionPropertiesFilename), false);

					// Now we need to re-overlay localDevice.properties
					System.out.println("Re-loading DEFAULT local properties file: " + LOCAL_DEVICE_PROPERTIES_FILE);
					configuration.addProperties(Configuration.loadProperties(LOCAL_DEVICE_PROPERTIES_FILE), true);
				}
			}
		} catch (IOException e) {
			e.printStackTrace();
			System.exit(1);
		}
		return configuration;
	}
	
	public static void main(String[] args) {

		// Load property files (with no command-line arguments) and validate
		Configuration testConfig = loadArgsAndProperties(new String[0]);

		testConfig.validate(true);
	}

}
