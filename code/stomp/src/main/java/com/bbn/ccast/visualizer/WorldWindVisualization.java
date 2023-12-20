//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */
package com.bbn.ccast.visualizer;

import com.bbn.ccast.AgentTelemPackage;
import com.bbn.ccast.TargetTelemPackage;
import com.bbn.ccast.Utils;
import com.bbn.ccast.mqtt.MaceMessageTransport;
import com.bbn.ccast.nullsim.NullSim;
import com.bbn.ccast.nullsim.SimRover;
import com.bbn.ccast.nullsim.SimVehicle;
import com.bbn.ccast.nullsim.blenodes.PayloadEventHandler;
import com.bbn.ccast.nullsim.blenodes.SimTargetEventHandler;
import com.bbn.ccast.nullsim.blenodes.TargetEventHandler;
import com.bbn.ccast.util.LatLonAlt;
import com.bbn.ccast.util.MathHelper;
import com.bbn.ccast.visualizer.util.CcastScreenSelector;
import com.bbn.ccast.visualizer.util.SelectionHighlightController;
import com.google.gson.JsonObject;
import com.google.gson.JsonElement;
import com.google.gson.JsonParser;
import de.micromata.opengis.kml.v_2_2_0.*;
import gov.nasa.worldwind.Configuration;
import gov.nasa.worldwind.WWObjectImpl;
import gov.nasa.worldwind.WorldWind;
import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.layers.CompassLayer;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.layers.LayerList;
import gov.nasa.worldwind.layers.RenderableLayer;
import gov.nasa.worldwind.ogc.kml.KMLRoot;
import gov.nasa.worldwind.ogc.kml.impl.KMLController;
import gov.nasa.worldwind.render.*;
import gov.nasa.worldwind.symbology.BasicTacticalSymbolAttributes;
import gov.nasa.worldwind.symbology.SymbologyConstants;
import gov.nasa.worldwind.symbology.TacticalSymbolAttributes;
import gov.nasa.worldwind.symbology.milstd2525.MilStd2525TacticalSymbol;
import gov.nasa.worldwind.symbology.milstd2525.SymbolCode;
import org.apache.log4j.Logger;
import org.eclipse.paho.client.mqttv3.MqttException;

import java.util.concurrent.TimeUnit;

import javax.swing.*;
import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;
import javax.xml.stream.XMLStreamException;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;
import java.util.logging.*;


/**
 * Provides a base application framework for simple WorldWind examples. Examine
 * other examples in this package to see how it's used.
 *
 * @version $Id: ApplicationTemplate.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class WorldWindVisualization{

	private static final Logger logger = Logger.getLogger(WorldWindVisualization.class.getName());
	private static final Logger guiLogger = Logger.getLogger("guiLog");
	private static final int FIELD_REFRESH_TIME_MS = 2000;
	private static final String HEATMAP_LAYER_PREFIX = "RSSI Heatmap";
	public static final String PAYLOAD_TELEM_MQTT_TOPIC = "command/server/payload/state";
    protected static final String RESET_TOPIC = "command/server/command";
	public static String TARGET_TELEM_MQTT_TOPIC;
	private static final long TELEM_UPDATE_SLEEP_TIME_MS = 500;
	public static final String CONTROL_NODE_LOADOUT = "CONTROL_NODE";

	private static JAXBContext jaxbContext;
	private static final Object initLock = new Object();

	public static final String WW_APP_NAME = "STOMP";

	public static final String OBSTACLE_LAYER_NAME = "Obstacles";
	public static final String AGENT_LAYER_NAME = "Agents";
	public static final String ALL_TARGETS_LAYER_NAME = "All Targets (White Force View)";
	public static final String DISCOVERED_TARGETS_LAYER_NAME = "Discovered Targets";
	public static final String UNDISCOVERED_TARGETS_LAYER_NAME = "Undiscovered Targets";
	public static final String BLUE_FORCE_TARGETS_LAYER_NAME = "Blue Force Targets";
	public static final String APRIL_TAG_LAYER_NAME = "April Tags";
	public static final String GROUND_TRUTH_APRIL_TAG_LAYER_NAME	= "Ground Truth April Tags";
	public static final String BLE_ARTIFACT_LAYER_NAME = "BLE Artifacts";
	public static final String MOLE_INFORMATION_LAYER_NAME = "Mole Information";
	public static final String EXECUTING_TACTICS_LAYER_NAME = "Tactics";
	public static final String MISSION_PLAN_LAYER_NAME = "Mission Plan";
	public static final String AGENT_PATH_LAYER_NAME = "Agent Paths";

	// Unit constants.
	public static final String UNIT_IMPERIAL = "Imperial";
	public static final String UNIT_METRIC = "Metric";

	protected static final double HUE_BLUE = 240d / 360d;
	protected static final double HUE_RED = 0d / 360d;

	public static boolean showAgentPathOnHover = false;
	private final TargetEventHandler targetEventHandler;
	private final PayloadEventHandler payloadEventHandler;
	private NullSim nullSim;

	private final ConcurrentHashMap<String, AgentTelemPackage> agentTelemMap = new ConcurrentHashMap<>();
	private final ConcurrentHashMap<String, TargetTelemPackage> targetTelemMap = new ConcurrentHashMap<>();


	static {

		// Set location of config document: note - this may be overridden by a similar
		// statement
		// in any other class that previously invoked a WorldWind routine
		System.setProperty("gov.nasa.worldwind.app.config.document", "src/main/resources/config/protelisww.xml");

		// OS-specific graphics customization
		System.setProperty("java.net.useSystemProxies", "true");
		if (Configuration.isMacOS()) {
			System.setProperty("apple.laf.useScreenMenuBar", "true");
			System.setProperty("com.apple.mrj.application.apple.menu.about.name", WW_APP_NAME);
			System.setProperty("apple.awt.application.name", WW_APP_NAME);
			System.setProperty("com.apple.mrj.application.growbox.intrudes", "false");
			System.setProperty("apple.awt.brushMetalLook", "true");
		} else if (Configuration.isWindowsOS()) {
			System.setProperty("sun.awt.noerasebackground", "true"); // prevents flashing during window resizing
		}

	}

	/**
	 * Java window where the visualization and controls will appear
	 */
	private final WorldWindAppFrame frame;

//	private final CcastWorldDatabase ccastWorldDatabase;

	/**
	 * Set of layers specified dynamically by the user rather than in the
	 * configuration file
	 */
	private Map<String, RenderableLayer> extraLayers = new HashMap<>();

	private Map<String, MilStd2525TacticalSymbol> groundTruthSymbols = new HashMap<>();
	private Map<String, MilStd2525TacticalSymbol> symbols = new HashMap<>();

	private static SymbolCode DISPATCHER_SYMBOL = new SymbolCode();
	private static final SymbolCode ROTOR_SYMBOL = new SymbolCode();
	private static final SymbolCode FIXED_WING_SYMBOL = new SymbolCode();
	private static final SymbolCode ROVER_SYMBOL = new SymbolCode();
	private static final SymbolCode DISCOVERED_TARGET_SYMBOL = new SymbolCode(); // The symbol used for targets
	private static final SymbolCode BLE_ARTIFACT_SYMBOL = new SymbolCode();
	private static final SymbolCode UNDISCOVERED_TARGET_SYMBOL = new SymbolCode();

	private static final SymbolCode MOLE_INFO_SYMBOL = new SymbolCode();

	private static TacticalSymbolAttributes HIGHLIGHT_ATTRS = new BasicTacticalSymbolAttributes();
	static {
		HIGHLIGHT_ATTRS.setTextModifierMaterial(Material.YELLOW);
		// HIGHLIGHT_ATTRS.setOpacity(0.5);
		HIGHLIGHT_ATTRS.setInteriorMaterial(Material.YELLOW);
	}

	private CcastScreenSelector screenSelector;
	private SelectionHighlightController selectionHighlightController;
	public final MaceMessageTransport maceMessageTransport;

	/**
	 * Create a new visualization window. Note that by default the application is
	 * bound to close when the window closes
	 *
	 * @param windowName
	 *            Name for the window
	 * @param mapCenter TODO
	 */

	public final com.bbn.ccast.config.Configuration configuration;
	public WorldWindVisualization(final String windowName, final com.bbn.ccast.config.Configuration configuration,
//								  CcastWorldDatabase ccastWorldDatabase,
								  Position mapCenter) {
		this.configuration = configuration;

		TARGET_TELEM_MQTT_TOPIC = configuration.getForceType().equalsIgnoreCase("white") ?
				"white_force/server/target/state" : "blue_force/server/target/state";

//		this.ccastWorldDatabase = ccastWorldDatabase;
		WorldWind.setOfflineMode(configuration.getWorldwindOfflineMode());
		this.frame = start(this, windowName, configuration);

		String broker = configuration.getMqttBroker();
		String uid = "WhiteForceVisualization-" + UUID.randomUUID().toString();

		boolean isWhiteForce = configuration.getForceType().equalsIgnoreCase("white");
		if (isWhiteForce) {
			String user = configuration.getMqttUsername();
			String password = configuration.getMqttPassword();
			logger.info("Connecting to mace mqtt as user: " + user);
			this.maceMessageTransport = new MaceMessageTransport(broker, uid, user, password);
		}else{
			this.maceMessageTransport = new MaceMessageTransport(broker, uid);
		}

		// Allow everything else to start up. This prevents getting several warnings about the inability to connect.
		while (this.maceMessageTransport.isConnected() == false)
		{
			try
			{
				Thread.sleep(1000);
				logger.info("Waiting for MQTT connection.");
			}
			catch(InterruptedException ex)
			{
				Thread.currentThread().interrupt();
			}
		}

		subscribeToMqtt();

		try {
			SwingUtilities.invokeAndWait(new Runnable() {

				@Override
				public void run() {

					init();
//					displayRegionData();
					if (mapCenter != null) {
						centerMapOn(mapCenter);
					}
				}});
		} catch (InvocationTargetException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		boolean isSim = configuration.isInSim();
		if (isSim){
			this.nullSim = NullSim.createAndStartNullSim(configuration, frame);
		}

		targetEventHandler  = new TargetEventHandler(maceMessageTransport);
		payloadEventHandler = new PayloadEventHandler(maceMessageTransport);


		new Thread(this::parseTelemUpdates, "Telem Parsing Thread").start();
	}

	private void subscribeToMqtt(){
		maceMessageTransport.subscribe(PAYLOAD_TELEM_MQTT_TOPIC, new Consumer<String>() {
			@Override
			public void accept(String msg) {
				// logger.debug(String.format("JSON msg on %s: %s", TARGET_TELEM_MQTT_TOPIC, msg));
				AgentTelemPackage payloadTelem = AgentTelemPackage.fromJson(msg);
				handleTelem(payloadTelem);
			}
		});
		maceMessageTransport.subscribe(TARGET_TELEM_MQTT_TOPIC, new Consumer<String>() {
			@Override
			public void accept(String msg) {
				//AgentTelemPackage controlTelem = AgentTelemPackage.fromJson(msg);
				// logger.debug(String.format("JSON msg on %s: %s", TARGET_TELEM_MQTT_TOPIC, msg));
				TargetTelemPackage targetTelem = TargetTelemPackage.fromJson(msg);
				targetTelem.setTimeReceived(System.currentTimeMillis());
				handleTelem(targetTelem);
			}
		});
        maceMessageTransport.subscribe(RESET_TOPIC, new Consumer<String>() {
            @Override
            public void accept(String msg) {
            // Create a new parser to parse the message
		    JsonParser parser = new JsonParser();
            // Get the message as a JSON object
            JsonObject jsonMessage = parser.parse(new String(msg)).getAsJsonObject();
            // Get the entry set from the Object
            Set<Map.Entry<String, JsonElement>> entries = jsonMessage.entrySet();
            // Create a new HashMap to store the entries in
            HashMap<String, JsonElement> jsonMap = new HashMap<String, JsonElement>();
            // Put all of the entries into the JSON HashMap
            for (Map.Entry<String, JsonElement> entry : entries) {
                jsonMap.put(entry.getKey(), entry.getValue());
            }

			// Check if we have the prepare new run command
			if (jsonMap.get("command").getAsString().equals("prepare_new_run")) {
				// Call the reset
				prepareNewRun();
			}
            }
        });
	}

	private void parseTelemUpdates(){
		while(!Thread.currentThread().isInterrupted()) {
			try {
				SwingUtilities.invokeAndWait(new Runnable() {
					@Override
					public void run() {
						for (AgentTelemPackage pkg : agentTelemMap.values()) {
							addOrUpdateSymbol(pkg);
						}
						updateCommandsPanel(agentTelemMap);

						for (TargetTelemPackage pkg : targetTelemMap.values()){
							addOrUpdateSymbol(pkg);
						}
						updateTargetsPanel(targetTelemMap);
					}
				});
			} catch (InterruptedException e) {
				//Do nothing
				Thread.currentThread().interrupt();
			} catch (InvocationTargetException e) {
				logger.error("Failed to handle telem on swing thread: ", e);
			}
			try {
				Thread.sleep(TELEM_UPDATE_SLEEP_TIME_MS);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
			}
		}
	}

	private void handleTelem(AgentTelemPackage payloadTelem) {
		agentTelemMap.put(payloadTelem.getUid(), payloadTelem);
	}

	private void handleTelem(TargetTelemPackage targetTelem){
		targetTelemMap.put(targetTelem.getUid(), targetTelem);
	}

//	void displayRegionData() {
//		// Display obstacles in region
//		if (ccastWorldDatabase == null) {
//			logger.error("Unable to display region data.  database is null.");
//			return;
//		}
//
//		List<File> persistentObstacleFiles = new LinkedList<File>();
//		List<File> localObstacleFiles = new LinkedList<File>();
//
//		HashMap<String, ArrayList<RegionTransition>> neighborRegions = new HashMap<>();
//
//		try {
//			if (ccastWorldDatabase.getCurrentRegion() == null) {
//				logger.error("Unable to display region data.  current region is null.");
//				return;
//			}
//
//			persistentObstacleFiles = ccastWorldDatabase.getCurrentRegion()
//					.getStaticObstacleKMLFiles(configuration.getWorkingDirectory());
//
//			String regionId = ccastWorldDatabase.getCurrentRegionId();
//
//			String basePath = com.bbn.ccast.config.Configuration.instance().getExplorationTempMapDir() + regionId;
//			String obstacleFileName = basePath + File.separator + regionId + Region.DEFAULT_OBSTACLE_KML_FILE_SUFFIX;
//			File obstacleFile = new File(obstacleFileName);
//			if (obstacleFile.canRead()) {
//				localObstacleFiles.add(obstacleFile);
//			}
//
//
//			neighborRegions = ccastWorldDatabase.getCurrentRegion().getNeighborRegions();
//
//		} catch (Exception anyError) {
//			logger.error("Error displaying region data", anyError);
//		}
//
//		// display region boundary
//		displayRegionBoundary(ccastWorldDatabase.getCurrentRegion());
//
//		if (localObstacleFiles != null && !localObstacleFiles.isEmpty()) {
//			// display local region obstacles
//			for (File obstacleFile : localObstacleFiles) {
//				this.displayKMLFile(obstacleFile);
//			}
//
//		} else {
//			// display persistent region obstacles
//			if (persistentObstacleFiles != null && !persistentObstacleFiles.isEmpty()) {
//				for (File obstacleFile : persistentObstacleFiles) {
//					this.displayKMLFile(obstacleFile);
//				}
//			}
//		}
//		// Display region transitions
//		Collection<ArrayList<RegionTransition>> regionTransitionLists = neighborRegions.values();
//		for (ArrayList<RegionTransition> regionTransitionList : regionTransitionLists) {
//			if (regionTransitionList != null && !regionTransitionList.isEmpty()) {
//				for (RegionTransition regionTransition : regionTransitionList) {
//					File regionTransitionFile = regionTransition.getTransitionFile(
//							ccastWorldDatabase.getCurrentRegion().getRegionDirectory(),
//							configuration.getWorkingDirectory());
//					if (regionTransitionFile != null && regionTransitionFile.canRead()) {
//						this.displayKMLFile(regionTransitionFile, regionTransition.getId());
//					}
//				}
//			}
//		}
//	}


//	protected void displayRegionBoundary(Region region) {
//
//		double latMin = region.getLatMinRegion();
//		double latMax = region.getLatMaxRegion();
//		double lonMin = region.getLonMinRegion();
//		double lonMax = region.getLonMaxRegion();
//
//		ArrayList<Position> regionBoundaryPositions = new ArrayList<Position>();
//		regionBoundaryPositions.add(Position.fromDegrees(latMax, lonMax, 0.0));
//		regionBoundaryPositions.add(Position.fromDegrees(latMin, lonMax, 0.0));
//		regionBoundaryPositions.add(Position.fromDegrees(latMin, lonMin, 0.0));
//		regionBoundaryPositions.add(Position.fromDegrees(latMax, lonMin, 0.0));
//
//		Polygon regionPolygon = new Polygon(regionBoundaryPositions);
//		regionPolygon.setAltitudeMode(WorldWind.CLAMP_TO_GROUND);
//		regionPolygon.setAttributes(REGION_BOUNDARY_ATTRIBUTES);
//		regionPolygon.setValue(AVKey.DISPLAY_NAME, region.getPrettyName()+"\u00b0");
//		addVisualization(regionPolygon, OBSTACLE_LAYER_NAME);
//	}


	public com.bbn.ccast.config.Configuration getConfiguration() {
		return this.configuration;
	}

//	public CcastWorldDatabase getCcastWorldDatabase() {
//		return this.ccastWorldDatabase;
//	}

	private void init() {

		orderLayers(AGENT_LAYER_NAME);

		DISPATCHER_SYMBOL.setBattleDimension(SymbologyConstants.BATTLE_DIMENSION_GROUND);
		DISPATCHER_SYMBOL.setOrderOfBattle(SymbologyConstants.ORDER_OF_BATTLE_CIVILIAN);
		DISPATCHER_SYMBOL.setStandardIdentity(SymbologyConstants.STANDARD_IDENTITY_FRIEND);
		DISPATCHER_SYMBOL.setStatus(SymbologyConstants.STATUS_PRESENT);
		DISPATCHER_SYMBOL.setScheme(SymbologyConstants.SCHEME_INTELLIGENCE);
		DISPATCHER_SYMBOL.setCategory(SymbologyConstants.CATEGORY_LOCATIONS);
		DISPATCHER_SYMBOL.setFunctionId(null);

		ROTOR_SYMBOL.setBattleDimension(SymbologyConstants.BATTLE_DIMENSION_AIR);
		ROTOR_SYMBOL.setOrderOfBattle(SymbologyConstants.ORDER_OF_BATTLE_CIVILIAN);
		ROTOR_SYMBOL.setStandardIdentity(SymbologyConstants.STANDARD_IDENTITY_FRIEND);
		ROTOR_SYMBOL.setStatus(SymbologyConstants.STATUS_PRESENT);
		ROTOR_SYMBOL.setScheme(SymbologyConstants.SCHEME_WARFIGHTING);
		ROTOR_SYMBOL.setCategory(SymbologyConstants.CATEGORY_TASKS);
		ROTOR_SYMBOL.setFunctionId("MFQ"); // Drone

		FIXED_WING_SYMBOL.setBattleDimension(SymbologyConstants.BATTLE_DIMENSION_AIR);
		FIXED_WING_SYMBOL.setOrderOfBattle(SymbologyConstants.ORDER_OF_BATTLE_CIVILIAN);
		FIXED_WING_SYMBOL.setStandardIdentity(SymbologyConstants.STANDARD_IDENTITY_FRIEND);
		FIXED_WING_SYMBOL.setStatus(SymbologyConstants.STATUS_PRESENT);
		FIXED_WING_SYMBOL.setScheme(SymbologyConstants.SCHEME_WARFIGHTING);
		FIXED_WING_SYMBOL.setCategory(SymbologyConstants.CATEGORY_TASKS);
		FIXED_WING_SYMBOL.setFunctionId("MF"); // Drone

		// how to symbolized status destroyed / neutralized
		// ROTOR_SYMBOL.setBattleDimension(SymbologyConstants.STATUS_DESTROYED);

		ROVER_SYMBOL.setBattleDimension(SymbologyConstants.BATTLE_DIMENSION_GROUND);
		ROVER_SYMBOL.setOrderOfBattle(SymbologyConstants.ORDER_OF_BATTLE_CIVILIAN);
		ROVER_SYMBOL.setStandardIdentity(SymbologyConstants.STANDARD_IDENTITY_FRIEND);
		ROVER_SYMBOL.setStatus(SymbologyConstants.STATUS_PRESENT);
		ROVER_SYMBOL.setScheme(SymbologyConstants.SCHEME_WARFIGHTING);
		ROVER_SYMBOL.setCategory(SymbologyConstants.CATEGORY_TASKS);
		ROVER_SYMBOL.setFunctionId(null);

		DISCOVERED_TARGET_SYMBOL.setScheme(SymbologyConstants.SCHEME_WARFIGHTING);
		DISCOVERED_TARGET_SYMBOL.setStandardIdentity(SymbologyConstants.STANDARD_IDENTITY_UNKNOWN);
		DISCOVERED_TARGET_SYMBOL.setBattleDimension(SymbologyConstants.BATTLE_DIMENSION_GROUND);
		DISCOVERED_TARGET_SYMBOL.setStatus(SymbologyConstants.STATUS_PRESENT);
//		DISCOVERED_TARGET_SYMBOL.setOrderOfBattle(SymbologyConstants.ORDER_OF_BATTLE_CIVILIAN);
//		DISCOVERED_TARGET_SYMBOL.setCategory(SymbologyConstants.CATEGORY_TASKS);
		DISCOVERED_TARGET_SYMBOL.setFunctionId(null);

		BLE_ARTIFACT_SYMBOL.setBattleDimension(SymbologyConstants.BATTLE_DIMENSION_GROUND);
		BLE_ARTIFACT_SYMBOL.setOrderOfBattle(SymbologyConstants.ORDER_OF_BATTLE_CIVILIAN);
		BLE_ARTIFACT_SYMBOL.setStandardIdentity(SymbologyConstants.STANDARD_IDENTITY_HOSTILE);
		BLE_ARTIFACT_SYMBOL.setStatus(SymbologyConstants.STATUS_PRESENT);
		BLE_ARTIFACT_SYMBOL.setScheme(SymbologyConstants.SCHEME_WARFIGHTING);
		BLE_ARTIFACT_SYMBOL.setCategory(SymbologyConstants.CATEGORY_TASKS);
		BLE_ARTIFACT_SYMBOL.setFunctionId(null);

		MOLE_INFO_SYMBOL.setBattleDimension(SymbologyConstants.BATTLE_DIMENSION_GROUND);
		MOLE_INFO_SYMBOL.setOrderOfBattle(SymbologyConstants.ORDER_OF_BATTLE_CIVILIAN);
		MOLE_INFO_SYMBOL.setStandardIdentity(SymbologyConstants.STANDARD_IDENTITY_UNKNOWN);
		MOLE_INFO_SYMBOL.setStatus(SymbologyConstants.STATUS_PRESENT);
		MOLE_INFO_SYMBOL.setScheme(SymbologyConstants.SCHEME_WARFIGHTING);
		MOLE_INFO_SYMBOL.setCategory(SymbologyConstants.CATEGORY_TASKS);
		MOLE_INFO_SYMBOL.setFunctionId(null);

		UNDISCOVERED_TARGET_SYMBOL.setBattleDimension(SymbologyConstants.BATTLE_DIMENSION_GROUND);
		UNDISCOVERED_TARGET_SYMBOL.setOrderOfBattle(SymbologyConstants.ORDER_OF_BATTLE_CIVILIAN);
		UNDISCOVERED_TARGET_SYMBOL.setStandardIdentity(SymbologyConstants.STANDARD_IDENTITY_PENDING);
		UNDISCOVERED_TARGET_SYMBOL.setStatus(SymbologyConstants.STATUS_PRESENT);
		UNDISCOVERED_TARGET_SYMBOL.setScheme(SymbologyConstants.SCHEME_WARFIGHTING);
		UNDISCOVERED_TARGET_SYMBOL.setCategory(SymbologyConstants.CATEGORY_TASKS);
		UNDISCOVERED_TARGET_SYMBOL.setFunctionId(null);

		ensureLayer(AGENT_LAYER_NAME);
		ensureLayer(DISCOVERED_TARGETS_LAYER_NAME);
		if (this.configuration.getForceType().equals("white")) {
			ensureLayer(ALL_TARGETS_LAYER_NAME).setEnabled(true);
		}
		ensureLayer(OBSTACLE_LAYER_NAME);
		ensureLayer(APRIL_TAG_LAYER_NAME);
		ensureLayer(EXECUTING_TACTICS_LAYER_NAME);
		ensureLayer(AGENT_PATH_LAYER_NAME);
		ensureLayer(MISSION_PLAN_LAYER_NAME);
		ensureLayer(GROUND_TRUTH_APRIL_TAG_LAYER_NAME);
		ensureLayer(BLE_ARTIFACT_LAYER_NAME);
		ensureLayer(MOLE_INFORMATION_LAYER_NAME).setEnabled(false);
		ensureLayer(UNDISCOVERED_TARGETS_LAYER_NAME);
		ensureLayer(BLUE_FORCE_TARGETS_LAYER_NAME);

//		this.frame.getLayerPanel().updateUI();
		// this.frame.getLayerPanel().update(frame.getWwd());

		// Create a screen selector to display a screen selection rectangle and track
		// the objects intersecting
		// that rectangle.
		this.screenSelector = new CcastScreenSelector(frame.getWwd());
		this.screenSelector.enable();

		// Set up a custom highlight controller that highlights objects both under the
		// cursor and inside the
		// selection rectangle. Disable the superclass' default highlight controller to
		// prevent it from interfering
		// with our highlight controller.
		this.selectionHighlightController = new SelectionHighlightController(frame.getWwd(), this.screenSelector);

//		if (device.isDispatcher()) {
//			Dispatcher dispatcher = device;
//			RssiMapper rssiMapper = dispatcher.getRssiMapper();
//			if (rssiMapper != null) {
//				SwingUtilities.invokeLater(new Runnable() {
//					@Override
//					public void run() {
//						initRssiHeatmaps(rssiMapper);
//						frame.getLayerPanel().updateUI();
//					}
//				});
//				//frame.getLayerPanel().updateLayers(frame.getWwd());
//			}
//		}

		frame.getWwjPanel().getHighlightController().dispose();

	}

	public static boolean areAllPositionsZeroAltitude(List<Position> positions) {
		for (Position position : positions) {
			if (position.getAltitude() != 0.0) {
				return false;
			}
		}
		return true;
	}

	/**
	 * Declare a set of layers that will be drawn in the specified order.
	 *
	 * @param names
	 *            List of layer names in order
	 */
	public void orderLayers(final String... names) {
		for (String n : names) {
			ensureLayer(n);
		}
	}

	/**
	 * Add a visualization element.
	 *
	 * @param element
	 *            Visualization object to be added
	 * @param layerName
	 *            Layer to be modified; if the layer doesn't exist, it will be
	 *            created
	 */
	public void addVisualization(final Renderable element, final String layerName) {
		RenderableLayer layer = ensureLayer(layerName);
		layer.addRenderable(element);
	}

	/**
	 * Remove a visualization element.
	 *
	 * @param element
	 *            Visualization object to be removed
	 * @param layerName
	 *            Layer to be modified; if the layer doesn't exist, it will be
	 *            created
	 */
	public void removeVisualization(final Renderable element, final String layerName) {
		RenderableLayer layer = ensureLayer(layerName);
		if(element != null) {
			layer.removeRenderable(element);
		}

	}

	/**
	 * Clear all visualizations from a layer.
	 *
	 * @param layerName
	 *            Layer to be midified; if the layer doesn't exist, it will be
	 *            created
	 */
	public void clearVisualization(final String layerName) {
		RenderableLayer layer = ensureLayer(layerName);
		layer.removeAllRenderables();
	}

	/**
	 * Inform the visualizer that the simulation has updated, and needs to be
	 * re-drawn.
	 */
	public void triggerRedraw() {
		frame.getWwd().redraw();
	}

	public Map<String, AgentTelemPackage> getAgentTelemMap(){
		return agentTelemMap;
	}

	public ConcurrentHashMap<String, TargetTelemPackage> getTargetTelemMap() {
		return targetTelemMap;
	}

	/**
	 * Make sure that a layer exists and is inserted
	 */
	public RenderableLayer ensureLayer(final String layerName) {
		if (!extraLayers.containsKey(layerName)) {
			RenderableLayer newLayer = new RenderableLayer();
			newLayer.setName(layerName);
			insertBeforeCompass(frame.getWwd(), newLayer);
			extraLayers.put(layerName, newLayer);
		}
		return extraLayers.get(layerName);
	}

	/**
	 * Insert a layer that will be drawn on top of all terrain imagery but below
	 * controls.
	 *
	 * @param wwd
	 *            window where it will be drawn
	 * @param layer
	 *            layer to be inserted
	 */
	public static void insertBeforeCompass(final WorldWindow wwd, final Layer layer) {
		// Insert the layer into the layer list just before the compass.
		int compassPosition = 0;
		LayerList layers = wwd.getModel().getLayers();
		for (Layer l : layers) {
			if (l instanceof CompassLayer) {
				compassPosition = layers.indexOf(l);
			}
		}
		layers.add(compassPosition, layer);
	}

	private static WorldWindAppFrame start(WorldWindVisualization viz, final String windowName, com.bbn.ccast.config.Configuration configuration) {
		if (Configuration.isMacOS() && windowName != null) {
			System.setProperty("com.apple.mrj.application.apple.menu.about.name", windowName);
		}

		try {
			final WorldWindAppFrame frame = new WorldWindAppFrame(viz, true, false, configuration);
			frame.setTitle(windowName);
			frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			java.awt.EventQueue.invokeLater(new Runnable() {
				@Override
				public void run() {
					frame.setVisible(true);
				}
			});

			return frame;
		} catch (Exception e) {
			e.printStackTrace();
			return null;
		}
	}

	public void centerMapOn(Position position) {
		// this.getWwd().getView().goTo(Position.fromDegrees(42.389938, -71.146389),
		// 400);
		// this.frame.wwjPanel.getWwd().getView().goTo(position, alt);
		this.frame.getWwd().getView().setEyePosition(position);
	}

//	public Agent getAgent() {
//		return agent;
//	}

//	public Dispatcher getDevice() {
//		return this.device;
//	}

	public CcastScreenSelector getScreenSelector() {
		return this.screenSelector;
	}

	public SelectionHighlightController SelectionHighlightController() {
		return this.selectionHighlightController;
	}

	public KMLController displayKMLFile(File kmlFile) {
		return displayKMLFile(kmlFile, null);
	}

	/**
	 * Adapted from
	 * https://forum.worldwindcentral.com/forum/world-wind-java-forums/development-help/17464-how-to-display-a-kml-file
	 *
	 * @param kmlFile
	 * @return
	 */
	public KMLController displayKMLFile(File kmlFile, String label) {

		KMLRoot kmlRoot = null;
		try {
			kmlRoot = KMLRoot.createAndParse(kmlFile);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (XMLStreamException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// Set the document's display name
		kmlRoot.setField(AVKey.DISPLAY_NAME, OBSTACLE_LAYER_NAME);

		// Schedule a task on the EDT to add the parsed document to a layer
		final KMLRoot finalKMLRoot = kmlRoot;

		// Create a KMLController to adapt the KMLRoot to the World Wind renderable
		// interface.
		KMLController kmlController = new KMLController(finalKMLRoot);
		// kmlLayer.addRenderable(kmlController);
		addVisualization(kmlController, OBSTACLE_LAYER_NAME);

		try {
			displayKMLFilePlacemarkNames(kmlFile, OBSTACLE_LAYER_NAME, label);
		} catch (JAXBException e) {
			logger.error("Unable to display placemark names for kml file: " + kmlFile);
		}

		this.frame.getWwd().redraw();
		return kmlController;
	}

	void displayKMLFilePlacemarkNames(File kmlFile, String layerName, String label) throws JAXBException {
		RenderableLayer layer = ensureLayer(layerName);

		synchronized (initLock) {
			if (jaxbContext == null) {
				jaxbContext = JAXBContext.newInstance(Kml.class);
			}
		}

		Unmarshaller u = jaxbContext.createUnmarshaller();

		if (kmlFile == null || !kmlFile.exists()) {
			logger.warn("kml file (" + kmlFile + ") does not exist.");
		}
		Kml kml = (Kml) u.unmarshal(kmlFile);
		Document document = (Document) kml.getFeature();
		List<Feature> outerFeatureList = document.getFeature();
		List<Feature> allFeatures = new ArrayList<>();
		//TODO
//		List<Feature> allFeatures = ObstacleData.getAllFeatures(outerFeatureList);

		for (Feature tg : allFeatures) {
			if (tg instanceof Placemark) {
				try {
					Coordinate initialPoint = null;
					Geometry g = ((Placemark) tg).getGeometry();
					if (g instanceof de.micromata.opengis.kml.v_2_2_0.Polygon) {
						List<Coordinate> coords = ((de.micromata.opengis.kml.v_2_2_0.Polygon) g).getOuterBoundaryIs()
								.getLinearRing().getCoordinates();
						if (coords != null && !coords.isEmpty()) {
							initialPoint = coords.get(0);
						}
					} else if (g instanceof de.micromata.opengis.kml.v_2_2_0.LineString) {
						List<Coordinate> coords = ((de.micromata.opengis.kml.v_2_2_0.LineString) g).getCoordinates();
						if (coords != null && !coords.isEmpty()) {
							initialPoint = coords.get(0);
						}
					}
					String labelName = label;
					if (labelName == null) {
						if (g.getId() != null) {
							labelName = g.getId();
						} else if (tg.getName() != null) {
							labelName = tg.getName();
						}
					}
					if (initialPoint != null && labelName != null) {
						LatLon ll = LatLon.fromDegrees(initialPoint.getLatitude(), initialPoint.getLongitude());
						Position pos = new Position(ll, initialPoint.getAltitude());
						SurfaceText surfaceText = new SurfaceText(labelName, pos);
						surfaceText.setTextSize(1.5);
						layer.addRenderable(surfaceText);
					}
				} catch (ClassCastException cce) {
					logger.warn("Got an unexpected class cast exception when parsing obstacle data. Exception: "
							+ cce.getMessage());
				}
			} else {
				logger.warn("Got unexpected obstacle data of type: " + tg.getClass().getSimpleName());
			}
		}
	}

//	public void update(Dispatcher device1) {
//
//		try {
//			SwingUtilities.invokeAndWait(new Runnable() {
//
//				@Override
//				public void run() {
//
//					try{
//						guiLogger.trace("Starting gui update cycle");
//
//						if (device1 != null) {
//							// Me
//
//							guiLogger.trace("Updating my symbol.");
//							addOrUpdateSymbol(device1);
//
//							// Other Agents
//							guiLogger.trace("Updating other symbols.");
//							Map<String, UplinkTelemPackage> otherAgents = device1.getUplinkTelemPkg();
//							if (otherAgents != null) {
//								for (UplinkTelemPackage telemPackage : otherAgents.values()) {
//									if (telemPackage != null) {
//										addOrUpdateSymbol(telemPackage);
//									}
//								}
//							}
//
//
//							Dispatcher dispatcher = device1;
//							guiLogger.trace("Updating artifact ineractions.");
//							Collection<ArtifactMessageInformation> artifactInteractions = dispatcher.getArtifactInteractions();
//							for (ArtifactMessageInformation info : artifactInteractions) {
//								addOrUpdateArtifactPanel(info);
//								addOrUpdateSymbol(info);
//							}
//
//							// Update Commands Panel
//							guiLogger.trace("Updating command panel");
//							updateCommandsPanel(device1, otherAgents);
//
//							// Add any tracks if they exist
//							guiLogger.trace("updating tracs");
//							processTracks(device1);
//
//							guiLogger.trace("triggering redraw");
//							triggerRedraw();
//							guiLogger.trace("completed gui update cycle");
//						}
//					}catch (Exception e){
//						logger.warn("Error on swing thread: ", e);
//					}
//
//				}});
//		} catch (InvocationTargetException | InterruptedException e) {
//			logger.error("Error invoking WorldWind update on swing thread: ", e);
//		}
//	}


//	private void processTracks(ProtelisDevice device1) {
//		synchronized(device1.getTrackQueueLock()) {
//			Set<SDOTrack> tracks = device1.getTrackQueue();
//			if (tracks != null && !tracks.isEmpty()) {
//				for (SDOTrack track : tracks) {
//					this.frame.getTrackController().addTrack(track, false);
//				}
//				device1.clearTrackQueue();
//			}
//		}
//	}

	private void updateCommandsPanel(
//			ProtelisDevice device1,
			Map<String, AgentTelemPackage> tacticsFromAll) {
		frame.updateAgentsPanel(
//				device1,
				tacticsFromAll);
	}

	private void updateTargetsPanel(
//			ProtelisDevice device1,
			Map<String, TargetTelemPackage> targetTelemMap) {
		frame.updateTargetsPanel(targetTelemMap);
	}


	private HashSet<MilStd2525TacticalSymbol> symbolsShown = new HashSet<>();

	private static ShapeAttributes PATH_SHAPE_ATTRIBUTES = null;
	static {
		PATH_SHAPE_ATTRIBUTES = new BasicShapeAttributes();
		PATH_SHAPE_ATTRIBUTES.setOutlineWidth(7);
		PATH_SHAPE_ATTRIBUTES.setOutlineMaterial(Material.GREEN);
	}

	private static ShapeAttributes PATH_SHAPE_PATH_BLOCKED_ATTRIBUTES = null;
	static {
		PATH_SHAPE_PATH_BLOCKED_ATTRIBUTES = new BasicShapeAttributes();
		PATH_SHAPE_PATH_BLOCKED_ATTRIBUTES.setOutlineWidth(7);
		PATH_SHAPE_PATH_BLOCKED_ATTRIBUTES.setOutlineMaterial(Material.RED);
	}

	private static ShapeAttributes GOAL_LINE_SHAPE_ATTRIBUTES = null;
	static {
		GOAL_LINE_SHAPE_ATTRIBUTES = new BasicShapeAttributes();
		GOAL_LINE_SHAPE_ATTRIBUTES.setOutlineMaterial(Material.BLUE);
	}

	private static ShapeAttributes REGION_BOUNDARY_ATTRIBUTES = null;
	static {
		REGION_BOUNDARY_ATTRIBUTES = new BasicShapeAttributes();
		REGION_BOUNDARY_ATTRIBUTES.setOutlineWidth(5);
		//REGION_BOUNDARY_ATTRIBUTES.setInteriorOpacity(0.0);
		REGION_BOUNDARY_ATTRIBUTES.setDrawInterior(false);
		REGION_BOUNDARY_ATTRIBUTES.setOutlineMaterial(Material.BLUE);
	}

	private SurfacePolyline polylineFromGoalList(LatLon currentPosition, List<MathHelper.LatLon> goalPositions){
		if (currentPosition == null || goalPositions == null)
			return null;
		List<LatLon> positions = new LinkedList<>();
		positions.add(currentPosition);
		for (MathHelper.LatLon pos : goalPositions){
			positions.add(MathHelper.toNasaLLA(pos));
		}
		SurfacePolyline ret = new SurfacePolyline(positions);
		ret.setAttributes(GOAL_LINE_SHAPE_ATTRIBUTES);
		return ret;
	}

	private SurfacePolyline polylineFromGoal(LatLon currentPosition, LatLon goalPosition) {
		if (currentPosition == null || goalPosition == null)
			return null;
		List<LatLon> positions = new LinkedList<>();
		positions.add(currentPosition);
		positions.add(goalPosition);
		SurfacePolyline ret = new SurfacePolyline(positions);
		ret.setAttributes(GOAL_LINE_SHAPE_ATTRIBUTES);
		return ret;
	}

	private Map<String, WWObjectImpl> agentUidToPathMap = new HashMap<>();
	private Map<String, SurfacePolyline> agentUidToGoalLineMap = new HashMap<>();

	public void showAgentPath(String agentUid) {
//		UplinkTelemPackage telemPackage = device.getUplinkTelemPkg().get(agentUid);
//		Position thisAgentPosition = Position.fromDegrees(telemPackage.getLatitude(), telemPackage.getLongitude(),
//				telemPackage.getAltitude());
//
//
//		// Path or SurfacePolyline
//		WWObjectImpl wwObjImpl = agentUidToPathMap.get(telemPackage.getUid());
//		if (wwObjImpl == null && telemPackage.getCurrentPath() != null && !telemPackage.getCurrentPath().isEmpty()) {
//			List<Position> positionList = MathHelper.toNasaPositionList(telemPackage.getCurrentPath());
//			if (positionList != null && !positionList.isEmpty()) {
//				positionList.add(0, thisAgentPosition);
//				if (telemPackage.getPlatformCotType().toLowerCase().contains("g")) {
//					SurfacePolyline path = new SurfacePolyline(positionList);
//					path.setAttributes(PATH_SHAPE_ATTRIBUTES);
//					path.setHighlightAttributes(PATH_SHAPE_ATTRIBUTES);
//					if (telemPackage.getPlannerBlocked()) {
//						path.setAttributes(PATH_SHAPE_PATH_BLOCKED_ATTRIBUTES);
//						path.setHighlightAttributes(PATH_SHAPE_PATH_BLOCKED_ATTRIBUTES);
//					}
//					addVisualization(path, AGENT_PATH_LAYER_NAME);
//					agentUidToPathMap.put(telemPackage.getUid().toString(), path);
//				} else {
//					Path path = new Path(positionList);
//					path.setAltitudeMode(WorldWind.RELATIVE_TO_GROUND);
//					path.setAttributes(PATH_SHAPE_ATTRIBUTES);
//					path.setHighlightAttributes(PATH_SHAPE_ATTRIBUTES);
//
//					if (telemPackage.getPlannerBlocked()) {
//						path.setAttributes(PATH_SHAPE_PATH_BLOCKED_ATTRIBUTES);
//						path.setHighlightAttributes(PATH_SHAPE_PATH_BLOCKED_ATTRIBUTES);
//					}
//					path.setFollowTerrain(true);
//					addVisualization(path, AGENT_PATH_LAYER_NAME);
//					agentUidToPathMap.put(telemPackage.getUid().toString(), path);
//				}
//			}
//		} else if (telemPackage.getCurrentPath() != null && !telemPackage.getCurrentPath().isEmpty()) {
//			List<Position> positionList = MathHelper.toNasaPositionList(telemPackage.getCurrentPath());
//			positionList.add(0, thisAgentPosition);
//			if (wwObjImpl instanceof SurfacePolyline) {
//				SurfacePolyline path = new SurfacePolyline((SurfacePolyline) wwObjImpl);
//				path.setLocations(positionList);
//				removeVisualization((Renderable) wwObjImpl, AGENT_PATH_LAYER_NAME);
//				addVisualization(path, AGENT_PATH_LAYER_NAME);
//				agentUidToPathMap.put(telemPackage.getUid().toString(), path);
//			} else {
//				Path path = new Path((Path) wwObjImpl);
//				path.setPositions(positionList);
//				removeVisualization((Renderable) wwObjImpl, AGENT_PATH_LAYER_NAME);
//				addVisualization(path, AGENT_PATH_LAYER_NAME);
//				agentUidToPathMap.put(telemPackage.getUid().toString(), path);
//			}
//
//		} else if (wwObjImpl != null) {
//			removeVisualization((Renderable) wwObjImpl, AGENT_PATH_LAYER_NAME);
//			agentUidToPathMap.remove(telemPackage.getUid().toString());
//		}

	}

	public void hideAgentPath(String agentUid) {
//		UplinkTelemPackage telemPackage = (UplinkTelemPackage) device.getAgentUidToTelemPackageMap().get(agentUid);
//		Position thisAgentPosition = Position.fromDegrees(telemPackage.getLatitude(), telemPackage.getLongitude(),
//				telemPackage.getAltitude());
//
//		// Path or SurfacePolyline
//		WWObjectImpl wwObjImpl = agentUidToPathMap.get(telemPackage.getUid().toString());
//		if (wwObjImpl == null && telemPackage.getCurrentPath() != null && !telemPackage.getCurrentPath().isEmpty()) {
//			List<Position> positionList = MathHelper.toNasaPositionList(telemPackage.getCurrentPath());
//			if (positionList != null && !positionList.isEmpty()) {
//				positionList.add(0, thisAgentPosition);
//				if (telemPackage.getPlatformCotType().toLowerCase().contains("g")) {
//					SurfacePolyline path = new SurfacePolyline(positionList);
//					agentUidToPathMap.put(telemPackage.getUid().toString(), path);
//				} else {
//					Path path = new Path(positionList);
//					path.setFollowTerrain(true);
//					agentUidToPathMap.put(telemPackage.getUid().toString(), path);
//				}
//			}
//		} else if (telemPackage.getCurrentPath() != null && !telemPackage.getCurrentPath().isEmpty()) {
//			List<Position> positionList = MathHelper.toNasaPositionList(telemPackage.getCurrentPath());
//			positionList.add(0, thisAgentPosition);
//			if (wwObjImpl instanceof SurfacePolyline) {
//				SurfacePolyline path = new SurfacePolyline((SurfacePolyline) wwObjImpl);
//				path.setLocations(positionList);
//				removeVisualization((Renderable) wwObjImpl, AGENT_PATH_LAYER_NAME);
//				agentUidToPathMap.put(telemPackage.getUid().toString(), path);
//			} else {
//				Path path = new Path((Path) wwObjImpl);
//				path.setPositions(positionList);
//				removeVisualization((Renderable) wwObjImpl, AGENT_PATH_LAYER_NAME);
//				agentUidToPathMap.put(telemPackage.getUid().toString(), path);
//			}
//
//		} else if (wwObjImpl != null) {
//			removeVisualization((Renderable) wwObjImpl, AGENT_PATH_LAYER_NAME);
//			agentUidToPathMap.remove(telemPackage.getUid().toString());
//		}
	}

	public void hideAllAgentPaths() {
		for(String agent1 : agentUidToPathMap.keySet()) {
			hideAgentPath(agent1);
		}
		agentUidToPathMap.clear();
	}

	public void removeAgentSymbol(String uid) {
		MilStd2525TacticalSymbol symbol = symbols.remove(uid);
		removeVisualization(symbol, AGENT_LAYER_NAME);
	}

	public void resetAgentPositions(Position pos) {
		if (!frame.getVisualization().getConfiguration().isInSim()){
			return;
		}

		double ii = 0.0;
		Map<String, SimVehicle> vehicleMap = getNullSim().getAllVehicles();

		for (SimVehicle vehicle : vehicleMap.values()) {
			double[] lla = {pos.getLatitude().getDegrees() + ii, pos.getLongitude().getDegrees(), 0.35};
			if (vehicle instanceof SimRover) {
				lla = new double[] {pos.getLatitude().getDegrees() + ii, pos.getLongitude().getDegrees(), 0.0};
			}

			vehicle.setOriginLla(lla, 0, 0, 0);
			ii += 0.0001;
		}
	}


	// Add or update symbols for payloads on map
	private void addOrUpdateSymbol(AgentTelemPackage telemPackage) {
		boolean isControlNode = telemPackage.getLoadout().equalsIgnoreCase(CONTROL_NODE_LOADOUT);
		MilStd2525TacticalSymbol symbol = symbols.get(telemPackage.getUid());
		// Determine position
//		double elevation = Utils.EARTH.getElevation(Angle.fromDegreesLatitude(telemPackage.getLatitude()),
//				Angle.fromDegreesLongitude(telemPackage.getLongitude()));
//		double altitudePlusElevation = elevation + telemPackage.getAltitude();
		Position pos = Position.fromDegrees(telemPackage.getLatitude(), telemPackage.getLongitude(), telemPackage.getAltitude());
		if (symbol == null) {
			SymbolCode code = ROTOR_SYMBOL;
			//TODO: update to use type when we add it to payload
			if (telemPackage.getUid().toLowerCase().contains("rover") || telemPackage.getCallsign().toLowerCase().contains("rover") ){
				code = ROVER_SYMBOL;
			}
			symbol = new MilStd2525TacticalSymbol(code.toString(), pos);
			addVisualization(symbol, AGENT_LAYER_NAME);
		}
		symbol.setPosition(pos);
		symbol.setShowLocation(false);
		symbol.setShowGraphicModifiers(true);

		if (telemPackage.getNeutralized()) {
			symbol.setStatus(SymbologyConstants.STATUS_DESTROYED);
		} else {
			symbol.setStatus(SymbologyConstants.STATUS_FULLY_CAPABLE);
		}

		// This is the name that shows up above payloads on the map
		symbol.setModifier(SymbologyConstants.UNIQUE_DESIGNATION, telemPackage.getCallsign());
		//symbol.setModifier(SymbologyConstants.DIRECTION_OF_MOVEMENT, Angle.fromDegrees(telemPackage.getHeading()));
		//symbol.setModifier(SymbologyConstants.AZIMUTH, Angle.fromDegrees(telemPackage.getHeading()));

		// If simulated entity, put an S in the center of the icon
/*		if (telemPackage.getIsSimPlatform()) {
			symbol.setModifier(SymbologyConstants.SPECIAL_C2_HEADQUARTERS, "S");
		}*/

		TacticalSymbolAttributes attrs = new BasicTacticalSymbolAttributes();
		attrs.setScale(0.2); // Make the symbol 20% its normal size.
		attrs.setTextModifierMaterial(Material.WHITE);
		symbol.setAttributes(attrs);
		symbol.setShowTextModifiers(true);
		symbol.setHighlightAttributes(HIGHLIGHT_ATTRS);

		symbols.put(telemPackage.getUid(), symbol);
		if(!showAgentPathOnHover) {
			showAgentPath(telemPackage.getUid());
		}

		// Line to Goal
		AgentTelemPackage uplinkTelemPackage = telemPackage;

//			symbol.setModifier(SymbologyConstants.ADDITIONAL_INFORMATION,
//					(uplinkTelemPackage.getArmed() ? "!" : "?")/* + telemPackage.getMode() */);

//		SurfacePolyline lineToGoal = agentUidToGoalLineMap.get(telemPackage.getUid());
//		if (lineToGoal == null){
//			if (uplinkTelemPackage.getCurrentGoalPositionList() != null){
//				lineToGoal = polylineFromGoalList(thisAgentPositionWithAltAddedToEarthElev,
//						uplinkTelemPackage.getCurrentGoalPositionList());
//				addVisualization(lineToGoal, EXECUTING_TACTICS_LAYER_NAME);
//				agentUidToGoalLineMap.put(telemPackage.getUid(), lineToGoal);
//			}else if(uplinkTelemPackage.getCurrentGoalPosition() != null){
//				lineToGoal = polylineFromGoal(thisAgentPositionWithAltAddedToEarthElev,
//						MathHelper.toNasaLLA(uplinkTelemPackage.getCurrentGoalPosition()));
//				addVisualization(lineToGoal, EXECUTING_TACTICS_LAYER_NAME);
//				agentUidToGoalLineMap.put(telemPackage.getUid(), lineToGoal);
//			}
//		}else if (uplinkTelemPackage.getCurrentGoalPositionList() != null){
//			List<LatLon> positionList = new ArrayList<>();
//			positionList.add(thisAgentPositionWithAltAddedToEarthElev);
//			for (MathHelper.LatLon pos : uplinkTelemPackage.getCurrentGoalPositionList()){
//				positionList.add(MathHelper.toNasaLLA(pos));
//			}
//			lineToGoal.setLocations(positionList);
//			agentUidToGoalLineMap.put(telemPackage.getUid(), lineToGoal);
//		}else if (uplinkTelemPackage.getCurrentGoalPosition() != null){
//			List<LatLon> positionList = new ArrayList<>();
//			positionList.add(thisAgentPositionWithAltAddedToEarthElev);
//			positionList.add(MathHelper.toNasaLLA(uplinkTelemPackage.getCurrentGoalPosition()));
//			agentUidToGoalLineMap.put(telemPackage.getUid(), lineToGoal);
//		}else{
//			removeVisualization(lineToGoal, EXECUTING_TACTICS_LAYER_NAME);
//			agentUidToGoalLineMap.remove(telemPackage.getUid());
//		}

		RenderableLayer layer = ensureLayer(AGENT_LAYER_NAME);
		layer.firePropertyChange(AVKey.LAYER, null, this);



	}

	// Add or update symbols for targets on map
	private void addOrUpdateSymbol(TargetTelemPackage targetTelem) {
		String uid = targetTelem.getUid();
		String callsign = targetTelem.getCallsign();
		String lastFourUidDigits = "-" + uid.substring(uid.length() - 4);
		// Determine position
		Position pos = Position.fromDegrees(targetTelem.getLatitude(), targetTelem.getLongitude(), targetTelem.getAltitude());
		// Check if it's a sim target
		boolean isSimTarget = uid.contains("Target_");
		
		
		///////////////////////////////////////////////////////////////////////////////////////
		//                              WHITE FORCE MODE                                     //
		///////////////////////////////////////////////////////////////////////////////////////
		if (this.configuration.getForceType().equals("white"))  {

			MilStd2525TacticalSymbol symbol = symbols.get(targetTelem.getUid());
			// Initial symbol placement
			if (symbol == null) {
				symbol = new MilStd2525TacticalSymbol(DISCOVERED_TARGET_SYMBOL.toString(), pos);
				// Leave symbol flat to ground if in sim
				if (!isSimTarget) {
					symbol.setAltitudeMode(WorldWind.ABSOLUTE);
				}
				addVisualization(symbol, ALL_TARGETS_LAYER_NAME);
			}
			//Update symbol position
			symbol.setPosition(pos);
			symbol.setShowLocation(false);
			symbol.setShowGraphicModifiers(true);

			// Is the target part of a linked network?
			if (!targetTelem.getNetworksCaptured().isEmpty() && !targetTelem.getSuppression()) {
				if (targetTelem.getNetworksCaptured().containsValue(true)) {
					symbol.setStatus(SymbologyConstants.STATUS_DESTROYED); // Only show target is captured if the entire network is
				}
				else if (targetTelem.isCaptured()) {
					symbol.setStatus(SymbologyConstants.STATUS_DAMAGED); // If individual target has capture parameters met, show as "damaged"
				}
				else {
					symbol.setStatus(SymbologyConstants.STATUS_FULLY_CAPABLE); 
				}
			} 
			
			// Is the target a suppression target (temporary capture), and has capture params met?
			else if (targetTelem.isCaptured() && targetTelem.getSuppression()) {
				symbol.setStatus(SymbologyConstants.STATUS_DAMAGED);
			} 
			
			// Is the target non suppression (permanent capture) and has capture params met?
			else if (targetTelem.isCaptured()) {
				symbol.setStatus(SymbologyConstants.STATUS_DESTROYED);
			} 
			
			// if nothing else applies, set target state to FULLY CAPABLE
			else {
				symbol.setStatus(SymbologyConstants.STATUS_FULLY_CAPABLE);
			}

			// Use whole uid for target name on map if in sim, use last four of uid otherwise
			String designation = targetTelem.getType() + "_" + callsign;
			symbol.setModifier(SymbologyConstants.UNIQUE_DESIGNATION, designation);


			TacticalSymbolAttributes attrs = new BasicTacticalSymbolAttributes();
			attrs.setScale(0.2); // Make the symbol 20% its normal size.
			attrs.setTextModifierMaterial(Material.WHITE);
			symbol.setAttributes(attrs);
			symbol.setShowTextModifiers(true);
			symbol.setHighlightAttributes(HIGHLIGHT_ATTRS);

			symbols.remove(targetTelem.getUid());
			symbols.put(targetTelem.getUid(), symbol);
			if(!showAgentPathOnHover) {
				showAgentPath(targetTelem.getUid());
			}

			RenderableLayer layer = ensureLayer(AGENT_LAYER_NAME);
			layer.firePropertyChange(AVKey.LAYER, null, this);
		}

		///////////////////////////////////////////////////////////////////////////////////////
		//                              BLUE FORCE MODE                                      //
		///////////////////////////////////////////////////////////////////////////////////////
		else {
			MilStd2525TacticalSymbol symbol = symbols.get(targetTelem.getUid() + "_blue_force");
			if (symbol == null) {
				symbol = new MilStd2525TacticalSymbol(DISCOVERED_TARGET_SYMBOL.toString(), pos);
				addVisualization(symbol, BLUE_FORCE_TARGETS_LAYER_NAME);
				// Leave symbol flat to ground if in sim
				if (!isSimTarget) {
					// If not in sim, set altitude to relative to ground
					symbol.setAltitudeMode(WorldWind.RELATIVE_TO_GROUND);
				}
			}
			// Update target location
			symbol.setPosition(pos);
			symbol.setShowLocation(false);
			symbol.setShowGraphicModifiers(true);

			// Initilize text designation to be shown next to the target icon on the map
			String designation = targetTelem.getType() + "_" + callsign;

			// Modify target icon or desigation based on several factors

			// Is the target discovered?
			if (!targetTelem.getDiscovered()) {
				designation = "TARGET" + lastFourUidDigits; // hide target type if undiscovered
				if (isSimTarget) {
					designation = uid;
				}
				// If a target is undiscovered, give it a dotted-border (ANTICIPATED)
				symbol.setStatus(SymbologyConstants.STATUS_ANTICIPATED);
			} 

			// Is the target part of a linked network?
			else if (!targetTelem.getNetworksCaptured().isEmpty() && !targetTelem.getSuppression()) {
				if (targetTelem.getNetworksCaptured().containsValue(true)) {
					symbol.setStatus(SymbologyConstants.STATUS_DESTROYED); // Only show target is captured if the entire network is
				}
				else if (targetTelem.isCaptured()) {
					symbol.setStatus(SymbologyConstants.STATUS_DAMAGED); // If individual target has capture parameters met, show as "damaged"
				}
				else {
					symbol.setStatus(SymbologyConstants.STATUS_FULLY_CAPABLE); 
				}
			} 
			
			// Is the target a suppression target (temporary capture), and has capture params met?
			else if (targetTelem.isCaptured() && targetTelem.getSuppression()) {
				symbol.setStatus(SymbologyConstants.STATUS_DAMAGED);
			} 
			
			// Is the target non suppression (permanent capture) and has capture params met?
			else if (targetTelem.isCaptured()) {
				symbol.setStatus(SymbologyConstants.STATUS_DESTROYED);
			} 
			
			// if nothing else applies, set target state to FULLY CAPABLE
			else {
				symbol.setStatus(SymbologyConstants.STATUS_FULLY_CAPABLE);
			}
			
			// Here we prepend the target type to the UID displayed on the map
			symbol.setModifier(SymbologyConstants.UNIQUE_DESIGNATION, designation);


			// Fix target symbol appearance
			TacticalSymbolAttributes attrs = new BasicTacticalSymbolAttributes();
			attrs.setScale(0.2); // Make the symbol 20% its normal size.
			attrs.setTextModifierMaterial(Material.WHITE);
			symbol.setAttributes(attrs);
			symbol.setShowTextModifiers(true);
			symbol.setHighlightAttributes(HIGHLIGHT_ATTRS);

			symbols.remove(targetTelem.getUid() + "_blue_force");
			symbols.put(targetTelem.getUid() + "_blue_force", symbol);
			 if (!showAgentPathOnHover) {
			 	showAgentPath(targetTelem.getUid());
			 }

			 RenderableLayer agentLayer = ensureLayer(AGENT_LAYER_NAME);
			 agentLayer.firePropertyChange(AVKey.LAYER, null, this);
		}

	}

	private MilStd2525TacticalSymbol setSymbolUnknown(MilStd2525TacticalSymbol defaultSymbol, MilStd2525TacticalSymbol symbol, boolean isSimTarget, Position pos) {
		if (!symbol.getIdentifier().equals(defaultSymbol.getIdentifier())) {
			removeVisualization(symbol, ALL_TARGETS_LAYER_NAME);
			symbol = new MilStd2525TacticalSymbol(DISCOVERED_TARGET_SYMBOL.toString(), pos);
			// Leave symbol flat to ground if in sim
			if (!isSimTarget) {
				symbol.setAltitudeMode(WorldWind.ABSOLUTE);
			}
			addVisualization(symbol, ALL_TARGETS_LAYER_NAME);
		}

		symbol.setPosition(pos);
		symbol.setShowLocation(false);
		symbol.setShowGraphicModifiers(true);
		return symbol;
	}

	private MilStd2525TacticalSymbol setSymbolNeutral(MilStd2525TacticalSymbol defaultSymbol, MilStd2525TacticalSymbol symbol, boolean isSimTarget, Position pos) {
		if (!symbol.getIdentifier().equals(defaultSymbol.getIdentifier())) {
			removeVisualization(symbol, ALL_TARGETS_LAYER_NAME);
			SymbolCode code = DISCOVERED_TARGET_SYMBOL;
			code.setStandardIdentity(SymbologyConstants.STANDARD_IDENTITY_NEUTRAL);
			symbol = new MilStd2525TacticalSymbol(code.toString(), pos);
			// Leave symbol flat to ground if in sim
			if (!isSimTarget) {
				symbol.setAltitudeMode(WorldWind.ABSOLUTE);
			}
			addVisualization(symbol, ALL_TARGETS_LAYER_NAME);
		}

		symbol.setPosition(pos);
		symbol.setShowLocation(false);
		symbol.setShowGraphicModifiers(true);
		return symbol;
	}

	public void setAgentGoalLine(String uid, List<MathHelper.LatLon> path, double agentLat, double agentLon, double agentAlt) {
		SurfacePolyline lineToGoal = agentUidToGoalLineMap.get(uid);
		double elevation = agentAlt
				+ Utils.EARTH.getElevation(Angle.fromDegreesLatitude(agentLat),
				Angle.fromDegreesLongitude(agentLon));
		Position thisAgentPositionWithAltAddedToEarthElev = Position.fromDegrees(agentLat,
				agentLon, elevation);
		if (lineToGoal == null){
			if (path != null){
				lineToGoal = polylineFromGoalList(thisAgentPositionWithAltAddedToEarthElev, path);
				addVisualization(lineToGoal, EXECUTING_TACTICS_LAYER_NAME);
				agentUidToGoalLineMap.put(uid, lineToGoal);
			}
		}else if (path != null){
			List<LatLon> positionList = new ArrayList<>();
			positionList.add(thisAgentPositionWithAltAddedToEarthElev);
			for (MathHelper.LatLon pos : path){
				positionList.add(MathHelper.toNasaLLA(pos));
			}
			lineToGoal.setLocations(positionList);
			agentUidToGoalLineMap.put(uid, lineToGoal);
		}else{
			removeVisualization(lineToGoal, EXECUTING_TACTICS_LAYER_NAME);
			agentUidToGoalLineMap.remove(uid);
		}
		RenderableLayer layer = ensureLayer(AGENT_LAYER_NAME);
		layer.firePropertyChange(AVKey.LAYER, null, this);
	}

//	private void addOrUpdateSymbol(ProtelisDevice device1) {
//		MilStd2525TacticalSymbol symbol = symbols.get(device1.getDeviceUID());
//		if (symbol == null) {
//			if (DISPATCHER_SYMBOL.toString() == null) {
//				// Things haven't been initialized correctly yet, so bail!
//				return;
//			}
//
//			symbol = new MilStd2525TacticalSymbol(DISPATCHER_SYMBOL.toString(), device1.getPosition());
//			symbol.setModifier(SymbologyConstants.UNIQUE_DESIGNATION, device1.getDeviceUID());
//			// symbol.setModifier(SymbologyConstants.ADDITIONAL_INFORMATION,
//			// device.getDeviceUID().toString());
//			TacticalSymbolAttributes attrs = new BasicTacticalSymbolAttributes();
//			attrs.setScale(0.2); // Make the symbol 20% its normal size.
//			attrs.setTextModifierMaterial(Material.WHITE);
//			symbol.setShowGraphicModifiers(true);
//			symbol.setAttributes(attrs);
//			symbol.setShowTextModifiers(true);
//			symbol.setShowLocation(false);
//			symbol.setHighlightAttributes(HIGHLIGHT_ATTRS);
//			symbols.put(device1.getDeviceUID(), symbol);
//			addVisualization(symbol, AGENT_LAYER_NAME);
//		} else {
//			symbol.setPosition(device1.getPosition());
//		}
//	}


	private static class MyHandler extends ConsoleHandler
	{
		public void publish(LogRecord logRecord)
		{
			//super.publish(logRecord);
			//Do nothing
		}
	}

	/**
	 * Testing the WorldWindVisualization
	 *
	 * @param args
	 *            Command-line arguments
	 */
	public static void main(final String[] args) {
		Thread.currentThread().setName("WorldWindVisualization Main Thread");

		Configuration.insertConfigurationDocument("config/worldwind.xml");

		WorldWindVisualization visualizer = null;

		com.bbn.ccast.config.Configuration configuration = com.bbn.ccast.config.Configuration.loadArgsAndProperties(args);

//		CcastWorldDatabase ccastWorldDatabase = new CcastWorldDatabase(configuration.getWorldRegionsFile(),
//				configuration.getInitialWorldRegion(), configuration.getWorkingDirectory());

		// --help is only applicable via the command-line arguments
		if (Utils.isNotNullAndTrue(configuration.getCmdOptions().help)) {
//			printUsage();
			System.exit(0);
		}

		// Get the World Wind logger by name.
		java.util.logging.Logger logger = java.util.logging.Logger.getLogger("gov.nasa.worldwind");

		// Turn off logging to parent handlers of the World Wind handler.
		logger.setUseParentHandlers(false);

		// Create a console handler (defined below) that we use to write log messages.
		final ConsoleHandler handler = new MyHandler();

		// Enable all logging levels on both the logger and the handler.
		logger.setLevel(Level.OFF);
		handler.setLevel(Level.OFF);

		gov.nasa.worldwind.util.Logging.logger().setLevel(Level.OFF);
		//Remove all default handlers
		for (Handler defHandler : gov.nasa.worldwind.util.Logging.logger().getHandlers()){
			gov.nasa.worldwind.util.Logging.logger().removeHandler(defHandler);
		}

		Filter filterAll = new Filter() {
			@Override
			public boolean isLoggable(LogRecord record) {
				return false;
			}
		};

		gov.nasa.worldwind.util.Logging.logger().setFilter(filterAll);
		logger.setFilter(filterAll);

		// Add our handler to the logger
		logger.addHandler(handler);

		gov.nasa.worldwind.util.Logging.logger().addHandler(handler);

		LogManager.getLogManager().reset();

		double centerMapLat = configuration
				.getDoubleProperty(com.bbn.ccast.config.Configuration.CENTER_MAP_LAT_PROPERTY);
		double centerMapLon = configuration
				.getDoubleProperty(com.bbn.ccast.config.Configuration.CENTER_MAP_LON_PROPERTY);
		double centerMapAlt = configuration
				.getDoubleProperty(com.bbn.ccast.config.Configuration.CENTER_MAP_ALT_PROPERTY);

		String windowName = (configuration.getWindowName() == null || configuration.getWindowName().isEmpty()) ? "STOMP" : configuration.getWindowName();
		if (Utils.isNotNullAndTrue(configuration.visualize())) {
			visualizer = new WorldWindVisualization(windowName, configuration,
//					ccastWorldDatabase,
					null);
			visualizer.centerMapOn(Position.fromDegrees(centerMapLat, centerMapLon, centerMapAlt));
		}

		Set<String> tags = new HashSet<>(Arrays.asList(new String[] { "all", "alpha" }));
		Set<String> tactics = new HashSet<>(
				Arrays.asList(new String[] { "123456789:GoTo", "978236928:GoTo", "467392992:GoTo" }));
		String loadout = "r1Rover";
//		UplinkTelemPackage fakeAgent = new UplinkTelemPackage("HAVOC", centerMapLat, centerMapLon,
//				30, 0, 270, 60,
//				DeviceType.ROVER.getCotType(),
//
//				System.currentTimeMillis(), tactics, null, "GUIDED", true, 97.3, 8, 3.8, null, null, false, new HashSet<String>(),
////				new LinkedList<CCAST_MAV_SYS_STATUS_SENSOR>(),
//				-77,
//				"testRegion", false, -2, (short) 1, true, false);

//		AgentTelemPackage fakeAgent = new AgentTelemPackage("HAVOC", centerMapLat, centerMapLon,
//				30, 0, System.currentTimeMillis(), new HashSet<String>(), false,
//				new HashSet<String>(), 93.3, false, false);
//
//		visualizer.addOrUpdateSymbol(fakeAgent, false);
//
//		Map<String, AgentTelemPackage> agentTelemList = new HashMap<>();
//		agentTelemList.put(fakeAgent.getUid(), fakeAgent);
//		visualizer.updateCommandsPanel(
////				null,
//				agentTelemList);

//		AgentTelemPackage payload = new AgentTelemPackage("payload1",
//				centerMapLat + 0.0001, centerMapLon + 0.0001, 30.0, 45.0,
//				System.currentTimeMillis(), new HashSet<String>(),
//				true, new HashSet<String>(), 98.0, false, false);
//		AgentTelemPackage controlNode = new AgentTelemPackage("controlNode1",
//				centerMapLat - 0.0001, centerMapLon - 0.0001, 30.0, 270.0,
//				System.currentTimeMillis(), new HashSet<String>(),
//				true, new HashSet<String>(), 78.0, false, true);

//		try {
//			visualizer.maceMessageTransport.publish(PAYLOAD_TELEM_MQTT_TOPIC, payload.toJson(), MaceMessageTransport.QualityOfService.UNRELIABLE);
//			visualizer.maceMessageTransport.publish(CONTROL_NODE_TELEM_MQTT_TOPIC, controlNode.toJson(), MaceMessageTransport.QualityOfService.UNRELIABLE);
//		} catch (MqttException e) {
//			e.printStackTrace();
//		}
	}


//	protected void createHeatmapColorSurfaceAtAltIndex(int altIndex, RssiMapper rssiMapper, RenderableLayer outLayer)
//	{
//		if (rssiMapper == null){
//			logger.warn("RssiMapper is null, not creating RSSI heatmap.");
//			return;
//		}
//
//		AnalyticSurface surface = new AnalyticSurface();
//		double minLat = rssiMapper.getMinLat();
//		double minLon = rssiMapper.getMinLon();
//		double maxLat = rssiMapper.getMaxLat();
//		double maxLon = rssiMapper.getMaxLon();
//
//
//		surface.setSector(Sector.fromDegrees(minLat, maxLat, minLon, maxLon));
//		logger.info("Creating surface with minLat " + minLat + ", maxLat " + maxLat + ", minLon " + minLon + ", maxLon " + maxLon);
//		//surface.setAltitudeMode(WorldWind.RELATIVE_TO_GROUND);
//		surface.setAltitudeMode(WorldWind.CLAMP_TO_GROUND);
//		surface.setAltitude(0);
//		final int width = rssiMapper.getLonGridLength();
//		final int height = rssiMapper.getLatGridLength();
//		surface.setDimensions(width, height);
//		logger.info("Creating surface with " + rssiMapper.getLonGridLength() +"x"
//				+ rssiMapper.getLatGridLength() + " grid.");
//		surface.setClientLayer(outLayer);
//
//		double verticalScale = 1.0;
//		surface.setVerticalScale(verticalScale);
//		outLayer.addRenderable(surface);
//
//		AnalyticSurfaceAttributes attr = new AnalyticSurfaceAttributes();
//		attr.setDrawOutline(false);
//		attr.setDrawShadow(false);
//		attr.setInteriorOpacity(0.5);
//		attr.setOutlineWidth(3);
//		surface.setSurfaceAttributes(attr);
//
//		Timer timer = new Timer(FIELD_REFRESH_TIME_MS, new ActionListener()
//		{
//			protected long startTime = -1;
//
//			public void actionPerformed(ActionEvent e)
//			{
//				if (this.startTime < 0) {
//					this.startTime = System.currentTimeMillis();
//				}
//
//				double[] values = rssiMapper.getHeatmapRssiValuesAtAltIndex(altIndex);
//
//				double min = 100;
//				double max = -40;
//				for (double val : values){
//					if (val < min){
//						min = val;
//					}
//					if (val > max){
//						max = val;
//					}
//				}
//
//				min = -100;
//
//				ArrayList<AnalyticSurface.GridPointAttributes> attributesList = new ArrayList<AnalyticSurface.GridPointAttributes>();
//
////				if (outLayer.isEnabled()) {
////					logger.info("updating heatmap at altIdx " + altIndex + " with values: " + Arrays.toString(values));
////				}
//
//				for (double value : values) {
//					double colorRatio = 1.0 - WWMath.computeInterpolationFactor(value, min, max);
//					Color color;
//					if (value == 0){
//						color = Color.getHSBColor(0.0f, 0.0f, 0.0f);
//					}else {
//						color = Color.getHSBColor((float) WWMath.mixSmooth(colorRatio, HUE_BLUE, HUE_RED), 1.0F, 1.0F);
//					}
//					attributesList.add(AnalyticSurface.createGridPointAttributes(value, color));
//				}
//
//				surface.setValues(attributesList);
//
//				surface.setSector(Sector.fromDegrees(rssiMapper.getMinLat(), rssiMapper.getMaxLat(),
//						rssiMapper.getMinLon(), rssiMapper.getMaxLon()));
//
//				if (surface.getClientLayer() != null) {
//					surface.getClientLayer().firePropertyChange(AVKey.LAYER, null, surface.getClientLayer());
//				}
//			}
//		});
//		timer.start();
//	}

//	private RenderableLayer[] rssiHeatmapLayers;
//	protected void initRssiHeatmaps(RssiMapper rssiMapper)
//	{
//		//Create a heatmap for each altitude bucket.
//		int numHeatmaps = rssiMapper.getAltGridLength();
//		rssiHeatmapLayers = new RenderableLayer[numHeatmaps];
//		for (int i=0; i < rssiHeatmapLayers.length; i++){
//			double alt = rssiMapper.getAltitudeFromIndex(i);
//			String labelStr = String.format("%s %.0fm", HEATMAP_LAYER_PREFIX, alt);
//			rssiHeatmapLayers[i] = ensureLayer(labelStr);
//			rssiHeatmapLayers[i].setPickEnabled(false);
//			rssiHeatmapLayers[i].setName(labelStr);
//
//			createHeatmapColorSurfaceAtAltIndex(i, rssiMapper, rssiHeatmapLayers[i]);
//
//			//Default to hiding
//			rssiHeatmapLayers[i].setEnabled(false);
//		}
//
//	}

	public NullSim getNullSim(){
		return nullSim;
	}

	private static void insertBeforePlacenames(WorldWindow wwd, Layer layer)
	{
		// Insert the layer into the layer list just before the placenames.
		int compassPosition = 0;
		LayerList layers = wwd.getModel().getLayers();
		layers.add(compassPosition, layer);
	}

	public TargetEventHandler getTargetEventHandler() {
		return targetEventHandler;
	}
	public PayloadEventHandler getPayloadEventHandler() {
		return payloadEventHandler;
	}

    public void prepareNewRun() {

		// Get the SimTargetFile
		File[] SimTargetFiles = getNullSim().getWorldWindAppFrame().getSimTargetFiles();

        // Check if we have loaded sim targets before
        if (SimTargetFiles != null) {
			// Reload each of the targets
			for (File file : SimTargetFiles) {
				getNullSim().getSimTargetManager().loadSimTargetConfigsFromFile(file);
			}
		}

        TargetEventHandler targetEventHandler = getTargetEventHandler();    
        // Get the list of target ID's
        String[] targetIDs = getNullSim().getSimTargetManager().getSimTargetIDs();  

        // Set to undiscovered
        targetEventHandler.sendSetDiscoverCommand(false, targetIDs);

        // Display reset command
        logger.debug(String.format("New Run has occurred"));
    }

}
