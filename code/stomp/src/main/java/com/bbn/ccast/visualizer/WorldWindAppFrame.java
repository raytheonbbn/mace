//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer;

import bibliothek.gui.dock.common.*;
import com.bbn.ccast.AgentTelemPackage;
import com.bbn.ccast.AgentEarth;
import com.bbn.ccast.TargetTelemPackage;
import com.bbn.ccast.mqtt.MaceMessageTransport;
import com.bbn.ccast.nullsim.SimVehicle;
import com.bbn.ccast.nullsim.blenodes.TargetEventHandler;
import com.bbn.ccast.visualizer.actions.CCASTScreenShotAction;
import com.bbn.ccast.visualizer.tactics.TacticsDialog;
import com.bbn.ccast.visualizer.tracks.*;
import com.bbn.ccast.visualizer.util.*;
import gov.nasa.worldwind.Configuration;
import gov.nasa.worldwind.Factory;
import gov.nasa.worldwind.WorldWind;
import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.avlist.AVList;
import gov.nasa.worldwind.avlist.AVListImpl;
import gov.nasa.worldwind.cache.FileStore;
import gov.nasa.worldwind.event.RenderingExceptionListener;
import gov.nasa.worldwind.event.SelectListener;
import gov.nasa.worldwind.exception.WWAbsentRequirementException;
import gov.nasa.worldwind.exception.WWRuntimeException;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.globes.ElevationModel;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.layers.ViewControlsLayer;
import gov.nasa.worldwind.layers.ViewControlsSelectListener;
import gov.nasa.worldwind.terrain.CompoundElevationModel;
import gov.nasa.worldwind.util.*;
import gov.nasa.worldwindx.examples.ApplicationTemplate;
import gov.nasa.worldwindx.examples.layermanager.LayerAndElevationManagerPanel;
import org.apache.log4j.Logger;
import org.w3c.dom.Document;
import org.w3c.dom.Element;


import javax.imageio.ImageIO;
import javax.swing.*;
import javax.xml.xpath.XPath;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Paths;
import java.util.List;
import java.util.*;

import static com.bbn.ccast.config.Configuration.*;

public class WorldWindAppFrame extends JFrame {
    private static final Logger logger = Logger.getLogger(WorldWindAppFrame.class.getName());
    private static final long serialVersionUID = 1902870695296019698L;

    private static final int OK = 0;
    private static final int CANCELLED = 2;

    private static final int DEFAULT_WIDTH = 400, DEFAULT_HEIGHT = 400;
    final WorldWindVisualization visualization;
    private Dimension canvasSize = new Dimension(DEFAULT_WIDTH, DEFAULT_HEIGHT);

    private WorldWindAppPanel wwjPanel;
    // private LayerAndElevationManagerPanel layerPanel;
    private StatisticsPanel statsPanel;
    private TargetPanel targetPanel;
    private AgentPanel agentPanel;
    private NetworkPanel networkPanel;
    private JPanel executionToolBarPanel;
    private JPanel debugToolBarPanel;
    private SDOControlPanel controlPanel;
    private SaveSDODialog saveTrackDialog;
    private Action screenShotAction;
    private TrackController trackController;
    private NetworkVisualizationManager networkVisualizationManager;
    private TacticsDialog activeTacticsDialog;
    private JFileChooser chooser;
    private final boolean isTimeScalable;

    private File[] SimTargetFiles;

    private static HashSet<String> filesLoaded = new HashSet<>();

    public void setActiveTacticsDialog(TacticsDialog val) {
        activeTacticsDialog = val;
    }

    private static WorldWindAppFrame instance = null;

    private final com.bbn.ccast.config.Configuration configuration;

    // Help
    protected static final String ONLINE_HELP_URL = "http://www.swarmtactics.com";
    // Analytics
    protected static final String ONLINE_ANALYTICS_URL = "http://localhost:5000/";

    public WorldWindAppFrame(WorldWindVisualization visualization, final boolean includeStatusBar,
                             final boolean includeStatsPanel,
                             com.bbn.ccast.config.Configuration configuration) {

        this.visualization = visualization;
        this.configuration = configuration;
        this.initialize(includeStatusBar, includeStatsPanel);
        this.isTimeScalable = configuration.isTimeScalable();

        Dimension dim = Toolkit.getDefaultToolkit().getScreenSize();
        setLocation(dim.width / 2 - this.getSize().width / 2, dim.height / 2 - this.getSize().height / 2);
        instance = this;
    }

    public static WorldWindAppFrame getInstance() {
        return instance;
    }

    protected void initialize(final boolean includeStatusBar, final boolean includeStatsPanel) {

        // Initialize the Dockable tab control
        CControl control = new CControl(this);
        control.setMissingStrategy(MissingCDockableStrategy.STORE);
        this.setLayout(new GridLayout(1, 1));
        this.add(control.getContentArea());
        try {
            UIManager.setLookAndFeel("javax.swing.plaf.metal.MetalLookAndFeel");
        } catch (ClassNotFoundException | InstantiationException | IllegalAccessException | UnsupportedLookAndFeelException e) {
            e.printStackTrace();
        }
        //UIManager.LookAndFeelInfo laf[] = UIManager.getInstalledLookAndFeels();

        CGrid grid = new CGrid(control);

        double toolbarWidth = 45.0;
        double toolbarHeight = 3.0;
        double wwjPanelWidth = toolbarWidth * 0.66;
        double wwjPanelHeight = 22.0;
        double topRightPanelWidth = toolbarWidth * 0.34;
        double topRightPanelHeight = wwjPanelHeight / 2.0;
        double bottomRightPanelWidth = topRightPanelWidth;
        double bottomRightPanelHeight = topRightPanelHeight;


        // Create the WorldWindow.
        this.wwjPanel = this.createAppPanel(this.canvasSize, includeStatusBar);
        this.wwjPanel.setPreferredSize(canvasSize);
        SingleCDockable mapDockable = new DefaultSingleCDockable("Map", "Map", wwjPanel);
        control.addDockable(mapDockable);
        grid.add(0.0, toolbarHeight, wwjPanelWidth, wwjPanelHeight, mapDockable);
        mapDockable.setVisible(true);

        screenShotAction = new CCASTScreenShotAction(this.getWwd(), this.getIcon("24x24-snapshot.gif"));


        // Create the Execution Toolbar.
        ExecutionToolBar executionToolBar = new ExecutionToolBar(this);
        this.executionToolBarPanel = executionToolBar.setupToolBar();
        // this.executionToolBarPanel = setupToolBar(false);
        DefaultSingleCDockable toolBarDockable = new DefaultSingleCDockable("Execution", "Execution",
                executionToolBarPanel);
        control.addDockable(toolBarDockable);
        grid.add(0, 0, toolbarWidth, toolbarHeight, toolBarDockable);
        toolBarDockable.setVisible(true);
        toolBarDockable.setCloseable(false);
        toolBarDockable.setTitleShown(true);

        // TODO: add a toolbar beneath this to label button groupings

        setupMenuBar(control);

        // load the elevation data in the data store
        loadInstalledDataFromFileStore(WorldWind.getDataFileStore());

        // Make the Layer Panel
        // this.layerPanel = new LayerAndElevationManagerPanel(this.wwjPanel.getWwd());
        // this.layerPanel = new LayerPanel(this.wwjPanel.getWwd(), null);

        // DefaultSingleCDockable layerDockable = new DefaultSingleCDockable("Layers", "Layers", layerPanel);
        // control.addDockable(layerDockable);
        // grid.add(wwjPanelWidth, toolbarHeight, topRightPanelWidth, topRightPanelHeight,
        //         layerDockable);
        // layerDockable.setVisible(true);
        // layerDockable.setCloseable(true);
        // // layerDockable.setExtendedMode(ExtendedMode.MINIMIZED);


        // Make the Agent Panel
        this.agentPanel = new AgentPanel(getWwd(), configuration, this);
        DefaultSingleCDockable agentDockable = new DefaultSingleCDockable("Agents", "Payloads", agentPanel);
        control.addDockable(agentDockable);
        grid.add(wwjPanelWidth, toolbarHeight, topRightPanelWidth, topRightPanelHeight,
                agentDockable);
        agentDockable.setVisible(true);
        agentDockable.setCloseable(true);

        // Make the Dispatcher Command Panel
//		this.dispatcherCommandsPanel = new DispatcherCommandsPanel(new DispatcherCommandsModel());
//		JScrollPane scrollPane = new JScrollPane(this.dispatcherCommandsPanel.getTreeTable());
//		SingleCDockable commandsDockable = new DefaultSingleCDockable("Commands", "Commands",
//				scrollPane);
//		control.addDockable(commandsDockable);
//		grid.add(wwjPanelWidth, toolbarHeight + topRightPanelHeight, bottomRightPanelWidth,
//				bottomRightPanelHeight, commandsDockable);
//		commandsDockable.setVisible(true);

        this.targetPanel = new TargetPanel(this, getWwd(), configuration);
        DefaultSingleCDockable targetDockable = new DefaultSingleCDockable("Targets", "Targets", targetPanel);
        grid.add(wwjPanelWidth, toolbarHeight + topRightPanelHeight, bottomRightPanelWidth,
                bottomRightPanelHeight, targetDockable);
        targetDockable.setVisible(true);
        targetDockable.setCloseable(true);
        control.addDockable(targetDockable);

        // this.networkPanel = new NetworkPanel(this, getWwd(), configuration);
        // DefaultSingleCDockable networkDockable = new DefaultSingleCDockable("Networks", "Networks",
        // networkPanel);
        // grid.add(wwjPanelWidth, toolbarHeight + topRightPanelHeight, bottomRightPanelWidth,
        // 		bottomRightPanelHeight, networkDockable);


        // Make a Tracks Panel
        this.controlPanel = new SDOControlPanel();
        // DefaultSingleCDockable tracksDockable = new DefaultSingleCDockable("Tracks", "Tracks", controlPanel);
        // grid.add(wwjPanelWidth, toolbarHeight + topRightPanelHeight, bottomRightPanelWidth,
        //         bottomRightPanelHeight, tracksDockable);
        // tracksDockable.setVisible(true);
        // tracksDockable.setCloseable(true);
        // control.addDockable(tracksDockable);


        // Deploy the grid - whatever that means
        control.getContentArea().deploy(grid);

        if (includeStatsPanel || System.getProperty("gov.nasa.worldwind.showStatistics") != null) {
            final int defaultStatsWidth = 250;
            this.statsPanel = new StatisticsPanel(this.wwjPanel.getWwd(),
                    new Dimension(defaultStatsWidth, canvasSize.height));
            this.getContentPane().add(this.statsPanel, BorderLayout.EAST);
        }
       trackController = new TrackController(visualization);
       this.trackController.setWwd(this.getWwd());
       this.trackController.setSDOListPanel(this.getSdoListPanel());
       this.trackController.setSDOPanel(this.getSdoPanel());

       this.wwjPanel.setTrackController(trackController);
       this.controlPanel.setTrackController(trackController);

        networkVisualizationManager = new NetworkVisualizationManager(visualization, getWwd());
        Thread networkVisThread = new Thread(networkVisualizationManager);
        networkVisThread.start();

        // Create and install the view controls layer and register a controller for it
        // with the World Window.
        ViewControlsLayer viewControlsLayer = new ViewControlsLayer();
        WorldWindVisualization.insertBeforeCompass(getWwd(), viewControlsLayer);
        this.getWwd().addSelectListener(new ViewControlsSelectListener(this.getWwd(), viewControlsLayer));

        // Register a rendering exception listener that's notified when exceptions occur
        // during rendering.
        this.wwjPanel.getWwd().addRenderingExceptionListener(new RenderingExceptionListener() {
            @Override
            public void exceptionThrown(final Throwable t) {
                if (t instanceof WWAbsentRequirementException) {
                    String message = "Computer does not meet minimum graphics requirements.\n";
                    message += "Please install up-to-date graphics driver and try again.\n";
                    message += "Reason: " + t.getMessage() + "\n";
                    message += "This program will end when you press OK.";

                    JOptionPane.showMessageDialog(WorldWindAppFrame.this, message, "Unable to Start Program",
                            JOptionPane.ERROR_MESSAGE);
                    System.exit(-1);
                }
            }
        });

        // Search the layer list for layers that are also select listeners and register
        // them with the World
        // Window. This enables interactive layers to be included without specific
        // knowledge of them here.
        for (Layer layer : this.wwjPanel.getWwd().getModel().getLayers()) {
            if (layer instanceof SelectListener) {
                this.getWwd().addSelectListener((SelectListener) layer);
            }
            // XXX: HACK to enable Bing layer by default
            if (layer.getName().contains("Bing")) {
                layer.setEnabled(true);
            }
        }
        // layerPanel.updateUI();
        // layerPanel.update(wwjPanel.getWwd());

        this.pack();

        // Center the application on the screen.
        WWUtil.alignComponent(null, this, AVKey.CENTER);
        this.setResizable(true);
    }

    protected void setupMenuBar(CControl control) {
        final File dockableSaveStateFile = visualization.getConfiguration()
                .getFileProperty(DOCKING_STORAGE_FILE_PROPERTY);

        JMenuBar menuBar = new JMenuBar();

        JMenu fileMenu = new JMenu();
        // ======== "File" ========
        fileMenu.setText("File");
        fileMenu.setMnemonic('F');

        // ---- "Open Sim Targets File" ----
        JMenuItem openSimTargetFile = new JMenuItem();
        openSimTargetFile.setText("Load Target Configuration");
        openSimTargetFile.setMnemonic('T');
        openSimTargetFile.setToolTipText("Load a sim target configuration file");
        openSimTargetFile.setAccelerator(
                KeyStroke.getKeyStroke(KeyEvent.VK_O,
                        Toolkit.getDefaultToolkit().getMenuShortcutKeyMaskEx()));
        openSimTargetFile.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                openSimTargetFile();
            }
        });
        fileMenu.add(openSimTargetFile);

        // ---- "Save Sim Targets File" ----
        JMenuItem saveSimTargets = new JMenuItem();
        saveSimTargets.setText("Save Target Configuration");
        saveSimTargets.setToolTipText("Save the current sim target positions and configurations");
        saveSimTargets.setMnemonic('S');
        saveSimTargets.setAccelerator(
                KeyStroke.getKeyStroke(KeyEvent.VK_S,
                        Toolkit.getDefaultToolkit().getMenuShortcutKeyMaskEx()));
        saveSimTargets.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent event) {
                // Show a save track dialog that won't prompt the user to choose a location
                // unless it's
                // necessary.
                saveSimTargets();
            }
        });
        fileMenu.add(saveSimTargets);

        fileMenu.addSeparator();

        // ---- "Screen Shot" ----
        JMenuItem screenShot = new JMenuItem(screenShotAction);
        screenShot.setIcon(null); // Make sure the menu items displays only text.
        screenShot.setMnemonic('L');
        screenShot.setAccelerator(
                KeyStroke.getKeyStroke(KeyEvent.VK_Z,
                        Toolkit.getDefaultToolkit().getMenuShortcutKeyMaskEx()));
        fileMenu.add(screenShot);

        // --------

        if (!Configuration.isMacOS()) {
            // --------
            fileMenu.addSeparator();

            JMenuItem exit = new JMenuItem();
            exit.setText("Exit");
            exit.setMnemonic('X');
            exit.setAccelerator(KeyStroke.getKeyStroke("alt F4"));
            exit.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent event) {
                    exit();
                }
            });
            fileMenu.add(exit);
        } else {
            try {
                OSXAdapter.setQuitHandler(this, getClass().getDeclaredMethod("exit", (Class[]) null));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        menuBar.add(fileMenu);


        /*
         * // ======== "Layers" ======== layerMenu = new LayerMenu();
         * layerMenu.setMnemonic('L'); menuBar.add(layerMenu);
         *
         * this.layerMenu.setWwd(this.getWwd());
         */
        // ======== "Window" ========
        JMenu menu = new JMenu("Window");
        menuBar.add(menu);
        JMenuItem menuItem = new JMenuItem("Save Layout");
        menuItem.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                if (dockableSaveStateFile != null) {
                    logger.info("Saving window layout to " + dockableSaveStateFile.getAbsolutePath());
                    try {
                        control.writeXML(dockableSaveStateFile);
                    } catch (IOException e1) {
                        e1.printStackTrace();
                    }
                } else {
                    logger.warn("No window storage file set with property " + DOCKING_STORAGE_FILE_PROPERTY);
                }
            }
        });
        menu.add(menuItem);
        JMenuItem loadMenu = new JMenuItem("Load Layout");
        loadMenu.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                if (dockableSaveStateFile != null) {
                    logger.info("Loading window layout from " + dockableSaveStateFile.getAbsolutePath());
                    try {
                        control.readXML(dockableSaveStateFile);
                    } catch (IOException e1) {
                        e1.printStackTrace();
                    }
                } else {
                    logger.warn("No window storage file set with property " + DOCKING_STORAGE_FILE_PROPERTY);
                }
            }
        });
        menu.add(loadMenu);

        // ======== "Help" ========
        JMenu helpMenu = new JMenu();
        helpMenu.setText("Help");
        helpMenu.setMnemonic('H');

//        // ---- "CCAST WorldWind Visualizer Help" ----
//        JMenuItem sarHelp = new JMenuItem();
//        sarHelp.setText("CCAST WorldWind Visualizer Help");
//        sarHelp.setMnemonic('H');
//        if (!Configuration.isMacOS())
//            sarHelp.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_F1, 0));
//        else
//            sarHelp.setAccelerator(
//                    KeyStroke.getKeyStroke(KeyEvent.VK_HELP,
//                            Toolkit.getDefaultToolkit().getMenuShortcutKeyMaskEx()));
//        sarHelp.addActionListener(new ActionListener() {
//            @Override
//            public void actionPerformed(ActionEvent e) {
//                showHelp();
//            }
//        });
//        helpMenu.add(sarHelp);

        // ---- "About CCAST WorldWind Visualizer" ----

        if (Desktop.isDesktopSupported()) {
            Desktop desktop = Desktop.getDesktop();
            if (desktop.isSupported(Desktop.Action.APP_ABOUT)) {
                desktop.setAboutHandler(evt -> {
                    SwingUtilities.invokeLater(() -> {
                        showAbout();
                    });
                });
            } else {
                JMenuItem about = new JMenuItem();
                about.setText("Tutorial");
                about.setMnemonic('A');
                about.addActionListener(new ActionListener() {
                    @Override
                    public void actionPerformed(ActionEvent event) {
                        showAbout();
                    }
                });
                helpMenu.add(about);
            }
        }

//        JMenu optionsMenu = new JMenu("Options");
//        JMenuItem toggleAgentPath = new JMenuItem();
//        toggleAgentPath.setText("Display Agent Path Only On Hover: OFF");
//        toggleAgentPath.setToolTipText("Enables or disables displaying the path of an agent ONLY when the " +
//                "mouse hovers over the agent on the map");
//        toggleAgentPath.addActionListener(e -> {
//            WorldWindVisualization.showAgentPathOnHover = !WorldWindVisualization.showAgentPathOnHover;
//            if (WorldWindVisualization.showAgentPathOnHover) {
//                toggleAgentPath.setText("Display Agent Path Only On Hover: ON");
//                WorldWindAppFrame.getInstance().getVisualization().hideAllAgentPaths();
//            } else {
//                toggleAgentPath.setText("Display Agent Path Only On Hover: OFF");
//            }
//        });
//        optionsMenu.add(toggleAgentPath);
//
//
//        JMenuItem clearTelem = new JMenuItem("Clear telem");
//        clearTelem.setToolTipText("Clears all telem packages received by dispatcher so far");
//        clearTelem.addActionListener(e -> {
////			getVisualization().getDevice().purgeAgentTelem();
//        });


//        optionsMenu.add(clearTelem);
//
//        menuBar.add(optionsMenu);
        menuBar.add(helpMenu);

        // Add version number to top right corner
        JMenu versionMenu = new JMenu();
        versionMenu.setText("Version 2.3");
        menuBar.add(Box.createHorizontalGlue());
        menuBar.add(versionMenu);

        setJMenuBar(menuBar);

        this.setJMenuBar(menuBar);
    }


    public void stopAllTactics() {
        logger.info("Stop all tactics.");
        if (!configuration.isInSim()){
            return;
        }
        Map<String, SimVehicle> vehicleMap = getVisualization().getNullSim().getAllVehicles();
        for (SimVehicle vehicle : vehicleMap.values()) {
            if (vehicle != null) {
                vehicle.setTarget(vehicle.getPosition());
            }
        }
    }

//	public void sendDispatcherCommand(CommandType cmd) {
//		String cmdXml = CommandXmlSerializationTools.getInstance(configuration).cmdToXml(cmd);
//		logger.info("STOMP sending dispatcher command: " + cmdXml);
//
//		String host = this.getVisualization().getConfiguration().getStringProperty
//		(DISPATCHER_HOST_ADDR_PROPERTY);
//		Integer port = this.getVisualization().getConfiguration().getIntProperty(DISPATCHER_PORT_PROPERTY);
//
//		try {
//			WorldWindAppPanel.sendDispatcherCommand(cmdXml, host, port);
//		} catch (IOException e) {
//			String message = "Error sending dispatcher command: " + cmdXml;
//			logger.error(message);
//		}
//	}


    public void addTrack(SDOTrack track) {
        trackController.addTrack(track);
    }

    public void addTracks(Collection<SDOTrack> tracks) {
        for (SDOTrack track : tracks) {
            addTrack(track);
        }
    }


    // Track Methods - New/Load/Save/Save As

    public void newTrack() {
        Object inputValue = JOptionPane.showInputDialog(this, "Enter a new track name", "Add New Track",
                JOptionPane.QUESTION_MESSAGE, null, null, null);
        if (inputValue == null)
            return;

        String name = inputValue.toString();

        SDOTrack st = new SDOTrack(name);
        trackController.addTrack(st);

        st.markDirty();
    }

    public boolean hasTrack(String trackName) {
        return filesLoaded.contains(trackName);
    }

    public void newTrackFromFile() {
        File[] files = showOpenTrackDialog("Open a track file");
        if (files == null || files.length == 0)
            return;

        for (File file : files) {
            this.newTrackFromFile(file.getPath(), null);
        }
    }

    public void newTrackFromFile(String filePath, String name) {
        if (filePath == null) {
            String message = "nullValue.FilePathIsNull";
            logger.error(message);
            throw new IllegalArgumentException(message);
        }

        SDOTrack track = null;
        try {
            track = SDOTrack.fromFile(filePath);
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (track == null)
            return;

        if (name != null)
            track.setName(name);

        trackController.addTrack(track);
        filesLoaded.add(filePath);

        track.clearDirtyBit();
    }

    public static void removeTrackFromMap(SDOTrack track) {
        if (track != null && track.getFile() != null && filesLoaded.contains(track.getFile().getPath())) {
            filesLoaded.remove(track.getFile().getPath());
        }
    }

    private int removeTrack(SDOTrack track, boolean forceSavePrompt) {
        if (track != null && track.getFile() != null) {

            String trackPath = track.getFile().getPath();

            int status = OK;
            if (track.isDirty() || forceSavePrompt) {

                int option = SaveSDODialog.showSaveChangesPrompt(this, track);
                // Show a save track dialog that won't prompt the user to choose a location
                // unless it's necessary.
                if (option == JOptionPane.YES_OPTION)
                    status = saveTrack(track, false);
                else if (option == JOptionPane.CANCEL_OPTION)
                    status = CANCELLED;
            }

            if (status != OK)
                return status;

            try {
                track.firePropertyChange(TrackController.TRACK_REMOVE, null, track);
                this.trackController.refreshCurrentTrack();
                this.getWwd().redraw();
            } catch (Exception e) {
                e.printStackTrace();
                return ERROR;
            }

            filesLoaded.remove(trackPath);
            return OK;
        }
        return ERROR;
    }

    private int removeAllTracks(boolean forceSavePrompt) {
        int status = OK;
        for (SDOTrack track : getSdoListPanel().getAllTracks()) {
            status |= removeTrack(track, forceSavePrompt);
            if ((status & CANCELLED) != 0)
                return status;
        }

        return status;
    }

    public void saveSimTargets() {
        if (!configuration.isInSim()){
            return;
        }
        File file = showSaveSimTargetsDialog("Save Sim Targets Configuration File");
        visualization.getNullSim().getSimTargetManager().saveSimTargetConfigsToFile(file);
    }

    public int saveTrack(SDOTrack track, boolean forceSavePrompt) {
        File file = null;
        int format = 0;

        if (track == null) {
            String message = Logging.getMessage("nullValue.TrackIsNull");
            logger.error(message);
            JOptionPane.showMessageDialog(this, "No Track to Save.", "Error", JOptionPane.ERROR_MESSAGE);
            return ERROR;
        }

        file = track.getFile();
        format = track.getFormat();

        // Show the "Save As..." dialog if either:
        // * The current track has no source file.
        // * The caller has specified that the user should prompted to select a file,
        if (file == null || forceSavePrompt) {
            int result = this.showSaveTrackDialog(track, file, format);
            if (result == SaveSDODialog.CANCEL_OPTION)
                return CANCELLED;
            else if (result == SaveSDODialog.ERROR_OPTION)
                return ERROR;

            file = this.saveTrackDialog.getSelectedFile();
            format = this.saveTrackDialog.getFileFormat();
        }

        try {
            // Get the file's last modified time,
            // or zero if the file does not exist.
            long time = file.exists() ? file.lastModified() : 0;

            SDOTrack.toFile(track, file.getPath(), format);

            // If the track was saved successfully (it exists and
            // is newer than is was before the save operation),
            // then adopt the properties of the new
            // file and format, and clear the track's dirty bit.
            if (file.exists() && time <= file.lastModified()) {
                track.setFile(file);
                track.setFormat(format);
                track.setName(file.getName());
                track.clearDirtyBit();
            }
        } catch (Exception e) {
            e.printStackTrace();
            return ERROR;
        }

        return OK;
    }

    private int showSaveTrackDialog(SDOTrack track, File file, int format) {
        if (this.saveTrackDialog == null)
            this.saveTrackDialog = new SaveSDODialog();

        this.saveTrackDialog.setDialogTitle(track);

        if (file != null)
            this.saveTrackDialog.setSelectedFile(file);
        else
            this.saveTrackDialog.setSelectedFile(track);

        if (format != 0)
            this.saveTrackDialog.setFileFormat(format);
        else
            this.saveTrackDialog.setFileFormat(track);

        return this.saveTrackDialog.showSaveDialog(this);
    }

    public void resetSimulation() {

		Object inputValue = JOptionPane.showInputDialog(this, "Enter a log tag", "Log tag",
                JOptionPane.QUESTION_MESSAGE, null, null, "log");

        if (inputValue == null)
            return;

        String tag = inputValue.toString();

        logger.info("Reset using tag: " + tag);
        
        // Get the target event handler
        TargetEventHandler targetEventHandler = visualization.getTargetEventHandler();
        // Send the reset command
        targetEventHandler.sendResetCommand(tag);
    }

    public void viewAnalytics() {
        logger.info("View Analytics.");
        // Open the browser to view the analytics
        try {
            BrowserOpener.browse(new URL(ONLINE_ANALYTICS_URL));
        } catch (Exception e1) {
            System.err.println("Unable to open Help window");
            e1.printStackTrace();
        }
    }

    public void openSimTargetFile() {
        if (!configuration.isInSim()){
            return;
        }

        File[] files = showOpenSimTargetsDialog("Open a sim targets config file");
        if (files == null || files.length == 0)
            return;

        // Save the files so that we can load them in when we press reset
        this.SimTargetFiles = files;

        for (File file : files) {
            visualization.getNullSim().getSimTargetManager().loadSimTargetConfigsFromFile(file);
        }
    }

    public File[] getSimTargetFiles(){
        return SimTargetFiles;
    }

    private File[] showOpenSimTargetsDialog(String title) {
        if (chooser == null) {
            chooser = new JFileChooser("target_configs");
        }

        chooser.setMultiSelectionEnabled(true);
        chooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
        chooser.setDialogType(JFileChooser.OPEN_DIALOG);
        chooser.resetChoosableFileFilters();

//		CompoundFilter filterAll = new CompoundFilter(filterArray, "Accepted Files");
//		chooser.addChoosableFileFilter(filterAll);
//
//		for (TrackReaderFilter filter : filterArray) {
//			chooser.addChoosableFileFilter(filter);
//		}
//
//		chooser.setFileFilter(filterAll);

        /*
         * String s =
         * getUserPreferences().getStringValue(SARKey.CURRENT_BROWSE_DIRECTORY); if (s
         * != null) this.openFileChooser.setCurrentDirectory(new File(s));
         */
        chooser.setDialogTitle(title != null ? title : "Open Sim Targets Configuration File");

        int retVal = chooser.showOpenDialog(this);
        if (retVal != JFileChooser.APPROVE_OPTION)
            return null;

        return chooser.getSelectedFiles();
    }

    private File showSaveSimTargetsDialog(String title) {
        if (chooser == null) {
            chooser = new JFileChooser("target_configs");
        }

        chooser.setMultiSelectionEnabled(false);
        chooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
        chooser.setDialogType(JFileChooser.SAVE_DIALOG);
        chooser.resetChoosableFileFilters();
        SaveSDOFilter filter = new SaveSDOFilter(2, "JSON (*.json)", new String[]{".json"});
        chooser.setFileFilter(filter);
        chooser.setDialogTitle(title != null ? title : "Save Sim Targets Configuration File");

        int retVal = chooser.showSaveDialog(this);
        if (retVal != JFileChooser.APPROVE_OPTION)
            return null;

        File ff = chooser.getSelectedFile();
        if (ff == null)
            return null;

        ff = filter.appendSuffix(ff);

        return ff;
    }

    private File[] showOpenTrackDialog(String title) {
        TrackReaderFilter[] filterArray =
                new TrackReaderFilter[]{new TrackReaderFilter(new MAVLinkTrackReader())};

        if (chooser == null) {
            chooser = new JFileChooser("target_configs");
        }

        chooser.setMultiSelectionEnabled(true);
        chooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
        chooser.resetChoosableFileFilters();

        CompoundFilter filterAll = new CompoundFilter(filterArray, "Accepted Files");
        chooser.addChoosableFileFilter(filterAll);

        for (TrackReaderFilter filter : filterArray) {
            chooser.addChoosableFileFilter(filter);
        }

        chooser.setFileFilter(filterAll);

        /*
         * String s =
         * getUserPreferences().getStringValue(SARKey.CURRENT_BROWSE_DIRECTORY); if (s
         * != null) this.openFileChooser.setCurrentDirectory(new File(s));
         */
        chooser.setDialogTitle(title != null ? title : "Open Track");

        int retVal = chooser.showOpenDialog(this);
        if (retVal != JFileChooser.APPROVE_OPTION)
            return null;

        return chooser.getSelectedFiles();
    }

    public boolean exit() {
        int status = removeAllTracks(false);
        if ((status & CANCELLED) != 0)
            return false;

        dispose();
        System.exit(0);
        return true;
    }

    private ImageIcon getIcon(String imageName) {
        String imagePath = "resources/images/" + imageName;
        Object o = WWIO.getFileOrResourceAsStream(imagePath, this.getClass());
        if (!(o instanceof InputStream))
            return null;

        try {
            BufferedImage icon = ImageIO.read((InputStream) o);
            return new ImageIcon(icon);
        } catch (Exception e) {
            return null;
        }
    }

    protected WorldWindAppPanel createAppPanel(final Dimension size, final boolean statusBar) {
        return new WorldWindAppPanel(this, size, statusBar);
    }

    public WorldWindAppPanel getWwjPanel() {
        return this.wwjPanel;
    }

    public WorldWindow getWwd() {
        return this.wwjPanel.getWwd();
    }

    public WorldWindVisualization getVisualization() {
        return this.visualization;
    }

    public List<SDOTrack> getSdoListPanelTracks() {
        SDOListPanel tracksPanel = getSdoListPanel();
        if (tracksPanel != null) {
            return tracksPanel.getAllTracks();
        }
        return null;
    }

    public SDOTrack getCurrentTrack() {
        SDOListPanel tracksPanel = getSdoListPanel();
        if (tracksPanel != null) {
            return getSdoListPanel().getCurrentTrack();
        }
        return null;
    }

    public SDOListPanel getSdoListPanel() {
        if (controlPanel != null) {
            return controlPanel.getSdoListPanel();
        }
        return null;
    }

    public SDOPanel getSdoPanel() {
        if (controlPanel != null) {
            return controlPanel.getSdoPanel();
        }
        return null;
    }


    /**
     * @return the activeTacticsDialogs
     */
    public TacticsDialog getActiveTacticsDialog() {
        return activeTacticsDialog;
    }

    /**
     * @return the trackController
     */
    public TrackController getTrackController() {
        return trackController;
    }


    // /**
    //  * @return the layerPanel
    //  */
    // public LayerAndElevationManagerPanel getLayerPanel() {
    //     return layerPanel;
    // }

    public NetworkVisualizationManager getNetworkVisualizationManager() {
        return networkVisualizationManager;
    }


    public void updateAgentsPanel(
            Map<String, AgentTelemPackage> tacticsFromAll) {


        agentPanel.addOrUpdateAll(tacticsFromAll);

//		if (isTimeScalable) {
//			device.getTimeServer().setTimeScale(agentPanel.getTimeScale());
//		}

    }

    public void updateTargetsPanel(Map<String, TargetTelemPackage> targetTelemMap) {
        targetPanel.addOrUpdateAll(targetTelemMap);
    }

    public void showHelp() {
        try {
            BrowserOpener.browse(new URL(ONLINE_HELP_URL));
        } catch (Exception e1) {
            System.err.println("Unable to open Help window");
            e1.printStackTrace();
        }
    }

    public void showAbout() {
        // WorldWindAppAboutDialog dialog = new WorldWindAppAboutDialog();
        // dialog.showDialog(this);
        try {
            BrowserOpener.browse(new URL(ONLINE_ANALYTICS_URL + "tutorial"));
        } catch (Exception e1) {
            System.err.println("Unable to open Help window");
            e1.printStackTrace();
        }
    }

    protected void loadInstalledDataFromFileStore(FileStore fileStore) {
        for (File file : fileStore.getLocations()) {
            if (!file.exists())
                continue;

            if (!fileStore.isInstallLocation(file.getPath()))
                continue;

            loadInstalledDataFromDirectory(file);
        }

        // Hack to move the base world elevation model to the start since it's somehow
        // ending up at the end of the list
        // and the list is supposedly ordered from lowest to highest resolution.
        ElevationModel defaultElevationModel = getWwd().getModel().getGlobe().getElevationModel();
        if (defaultElevationModel instanceof CompoundElevationModel) {
            CompoundElevationModel cem = (CompoundElevationModel) defaultElevationModel;
            ElevationModel baseModel = null;
            for (ElevationModel em : cem.getElevationModels()) {
                if (em.getName().equalsIgnoreCase("USA 10m, World 30m, Ocean 900m")) {
                    baseModel = em;
                }
            }
            if (baseModel != null) {
                cem.removeElevationModel(baseModel);
                cem.addElevationModel(0, baseModel);
            }
        }

    }

    // **************************************************************//
    // ******************** Loading Previously Installed Data *****//
    // **************************************************************//

    protected void loadInstalledDataFromDirectory(File dir) {
        String[] names = WWIO.listDescendantFilenames(dir, new DataConfigurationFilter(), false);
        if (names == null || names.length == 0)
            return;

        for (String filename : names) {
            Document doc = null;

            try {
                File dataConfigFile = new File(dir, filename);
                doc = WWXML.openDocument(dataConfigFile);
                doc = DataConfigurationUtils.convertToStandardDataConfigDocument(doc);
            } catch (WWRuntimeException e) {
                e.printStackTrace();
            }

            if (doc == null)
                continue;

            // This data configuration came from an existing file from disk, therefore we
            // cannot guarantee that the
            // current version of World Wind's data installer produced it. This data
            // configuration file may have been
            // created by a previous version of World Wind, or by another program. Set
            // fallback values for any missing
            // parameters that World Wind needs to construct a Layer or ElevationModel from
            // this data configuration.
            AVList params = new AVListImpl();
            setFallbackParams(doc, filename, params);

            this.addToWorldWindow(doc.getDocumentElement(), params);

        }
    }

    protected void addToWorldWindow(Element domElement, AVList params) {
        String type = DataConfigurationUtils.getDataConfigType(domElement);
        if (type == null)
            return;

        if (type.equalsIgnoreCase("Layer")) {
            this.addLayerToWorldWindow(domElement, params);
        } else if (type.equalsIgnoreCase("ElevationModel")) {
            this.addElevationModelToWorldWindow(domElement, params);
        }
    }

    protected void addLayerToWorldWindow(Element domElement, AVList params) {
        Layer layer = null;
        try {
            Factory factory = (Factory) WorldWind.createConfigurationComponent(AVKey.LAYER_FACTORY);
            layer = (Layer) factory.createFromConfigSource(domElement, params);
        } catch (Exception e) {
            String message = Logging.getMessage("generic.CreationFromConfigurationFailed",
                    DataConfigurationUtils.getDataConfigDisplayName(domElement));
            Logging.logger().log(java.util.logging.Level.SEVERE, message, e);
        }

        if (layer == null)
            return;

        layer.setEnabled(true); // TODO: BasicLayerFactory creates layer which is initially disabled

        if (!getWwd().getModel().getLayers().contains(layer))
            ApplicationTemplate.insertBeforeCompass(getWwd(), layer);
    }

    public static void setFallbackParams(Document dataConfig, String filename, AVList params) {
        XPath xpath = WWXML.makeXPath();
        Element domElement = dataConfig.getDocumentElement();

        // If the data configuration document doesn't define a cache name, then compute
        // one using the file's path
        // relative to its file cache directory.
        String s = WWXML.getText(domElement, "DataCacheName", xpath);
        if (s == null || s.length() == 0)
            DataConfigurationUtils.getDataConfigCacheName(filename, params);

        // If the data configuration document doesn't define the data's extreme
        // elevations, provide default values using
        // the minimum and maximum elevations of Earth.
        String type = DataConfigurationUtils.getDataConfigType(domElement);
        if (type.equalsIgnoreCase("ElevationModel")) {
            if (WWXML.getDouble(domElement, "ExtremeElevations/@min", xpath) == null)
                params.setValue(AVKey.ELEVATION_MIN, Earth.ELEVATION_MIN);
            if (WWXML.getDouble(domElement, "ExtremeElevations/@max", xpath) == null)
                params.setValue(AVKey.ELEVATION_MAX, Earth.ELEVATION_MAX);
        }
    }

    protected void addElevationModelToWorldWindow(Element domElement, AVList params) {
        ElevationModel em = null;
        try {
            Factory factory = (Factory) WorldWind.createConfigurationComponent(AVKey.ELEVATION_MODEL_FACTORY);
            em = (ElevationModel) factory.createFromConfigSource(domElement, params);
        } catch (Exception e) {
            String message = Logging.getMessage("generic.CreationFromConfigurationFailed",
                    DataConfigurationUtils.getDataConfigDisplayName(domElement));
            Logging.logger().log(java.util.logging.Level.SEVERE, message, e);
        }

        if (em == null)
            return;

        ElevationModel defaultElevationModel = getWwd().getModel().getGlobe().getElevationModel();
        if (defaultElevationModel instanceof CompoundElevationModel) {
            if (!AgentEarth.containsElevationModelByName((CompoundElevationModel) defaultElevationModel, em)) {
                ((CompoundElevationModel) defaultElevationModel).addElevationModel(em);
            }
        } else {
            CompoundElevationModel cm = new CompoundElevationModel();
            cm.addElevationModel(defaultElevationModel);
            cm.addElevationModel(em);
            getWwd().getModel().getGlobe().setElevationModel(cm);
        }
    }

    public Action getScreenShotAction() {
        return screenShotAction;
    }

    public void setSelectedTarget(String uid) {
        wwjPanel.setSelectedTarget(uid);
    }

    public TargetPanel getTargetPanel() {
        return targetPanel;
    }
}
