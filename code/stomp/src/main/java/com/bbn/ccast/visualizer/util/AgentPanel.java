//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.AgentTelemPackage;
import com.bbn.ccast.Utils;
import com.bbn.ccast.config.Configuration;
import com.bbn.ccast.visualizer.WorldWindAppFrame;

import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import org.apache.log4j.Logger;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.awt.*;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.HashMap;
import java.util.Map;

public class AgentPanel extends JPanel {

    private static final Logger logger = Logger.getLogger(AgentPanel.class.getName());

    private final double MAX_TIME_SCALE = 10.0;
    private final double TIME_SLIDER_SCALAR = 10.0;

    private static final long serialVersionUID = 1L;

    //public static final DateFormat df = new SimpleDateFormat("dd:MM:yy:HH:mm:ss");
    public static final DateFormat df = new SimpleDateFormat("HH:mm:ss");

    private static final double CLICK_JUMP_ALT = 1000;

    public static final double IN_AIR_ALTITUDE = 1.0;

    private static final int UID_COLUMN = 0;

    private final WorldWindow wwd;
    private final Configuration configuration;

    private JTable table;
    private AgentTableModel tableModel;
    private JScrollPane scrollPane;
    private Map<String, AgentTelemPackage> uidToTelemMap = new HashMap<>();
    private JPanel tablePanel;
    private JPanel metaDataPanel;
    private JTextField agentsInAir;
    private JTextField nbrAgents;
    private JTextField nbrAirAgents;
    private JTextField nbrGroundAgents;
    private JLabel timeScaleLabel;

    private double timeScale;
    private WorldWindAppFrame frame;


    public AgentPanel(WorldWindow wwd, Configuration configuration, WorldWindAppFrame frame) {

        super(new BorderLayout());


        this.configuration = configuration;
        this.wwd = wwd;
        this.frame = frame;
        this.initComponents();
    }

    private void initComponents() {

        initAgentMetaData();
        initAgentTable();

        this.metaDataPanel.setMinimumSize(new Dimension(0, 0));

        JSplitPane splitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT, true, this.metaDataPanel,
                this.tablePanel);
        splitPane.setBorder(BorderFactory.createEmptyBorder());
        splitPane.setResizeWeight(0.15);
        this.add(splitPane, BorderLayout.CENTER);

    }

    protected void initAgentTable() {
        tableModel = new AgentTableModel(this.frame);
        table = new JTable(tableModel);
        table.setDefaultRenderer(Object.class, new AgentTableRenderer());
        table.addMouseListener(new java.awt.event.MouseAdapter() {
            @Override
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                int row = table.rowAtPoint(evt.getPoint());
                int col = table.columnAtPoint(evt.getPoint());
                if (row >= 0 && col >= 0) {
                    try {
                        Object val = tableModel.getValueAt(row, UID_COLUMN);
                        logger.info("Clicked UID: " + val);
                        if (val != null && val instanceof String) {
                            String clickedUid = val.toString();
                            AgentTelemPackage clickedTelemPackage = null;
                            synchronized (uidToTelemMap) {
                                clickedTelemPackage = uidToTelemMap.get(clickedUid);
                            }
                            if (clickedTelemPackage != null && clickedTelemPackage.getAltitude() < 10000 ){
                                wwd.getView().setEyePosition(Position.fromDegrees(clickedTelemPackage.getLatitude(), clickedTelemPackage.getLongitude(), CLICK_JUMP_ALT));
                            }
                        }
                    } catch (IllegalArgumentException e) {
                        logger.warn("Table model had null payload: ", e);
                    }
                }
            }
        });

        table.setAutoResizeMode(JTable.AUTO_RESIZE_OFF);
        scrollPane = new JScrollPane(table);

        tablePanel = new JPanel();
        tablePanel.setLayout(new BorderLayout(0, 0));

        tablePanel.add(scrollPane, BorderLayout.CENTER);

        if (configuration.isTimeScalable()) {

            timeScale = configuration.getInitialTimeScale();

            timeScaleLabel = new JLabel();
            updateTimeScaleLabel();

            // Don't let scale get to zero, else we'll hit a singularity
            JSlider timeScaleSlider = new JSlider(1, (int) (TIME_SLIDER_SCALAR * MAX_TIME_SCALE));
            timeScaleSlider.setMinorTickSpacing((int) TIME_SLIDER_SCALAR);
            timeScaleSlider.setValue((int) (timeScale * TIME_SLIDER_SCALAR));
            timeScaleSlider.addChangeListener(new ChangeListener() {

                @Override
                public void stateChanged(ChangeEvent event) {
                    timeScale = timeScaleSlider.getValue() / TIME_SLIDER_SCALAR;
                    updateTimeScaleLabel();
                }
            });

            JPanel timePanel = new JPanel();
            timePanel.add(timeScaleLabel);
            timePanel.add(timeScaleSlider);

            tablePanel.add(timePanel, BorderLayout.SOUTH);
        }

        table.setShowGrid(true);
    }

    private void updateTimeScaleLabel() {
        timeScaleLabel.setText("Time Scale: " + timeScale);
    }

    protected void initAgentMetaData() {
        metaDataPanel = new JPanel();
        metaDataPanel.setLayout(new FlowLayout());

        JLabel label = new JLabel("Total Payloads");
        metaDataPanel.add(label);

        nbrAgents = new JTextField();
        nbrAgents.setEditable(false);
        nbrAgents.setColumns(4);
        nbrAgents.setToolTipText("Total number of payloads");
        metaDataPanel.add(nbrAgents);

        label = new JLabel("Rovers");
        metaDataPanel.add(label);

        nbrGroundAgents = new JTextField();
        nbrGroundAgents.setEditable(false);
        nbrGroundAgents.setColumns(4);
        nbrGroundAgents.setToolTipText("Number of rovers");
        metaDataPanel.add(nbrGroundAgents);

        label = new JLabel("Air Assets");
        metaDataPanel.add(label);

        nbrAirAgents = new JTextField();
        nbrAirAgents.setEditable(false);
        nbrAirAgents.setColumns(4);
        nbrAirAgents.setToolTipText("Number of air assets");
        metaDataPanel.add(nbrAirAgents);

        label = new JLabel("Payloads In Air");
        metaDataPanel.add(label);

        agentsInAir = new JTextField();
        agentsInAir.setEditable(false);
        agentsInAir.setColumns(4);
        agentsInAir.setToolTipText("Number of payloads in the air");
        metaDataPanel.add(agentsInAir);
    }

    private boolean isAgentInAir(AgentTelemPackage telemPackage) {
//		if ((telemPackage.getPlatformCotType().equals(DeviceType.ROTOR_CRAFT.getCotType()) ||
//			 telemPackage.getPlatformCotType().equals(DeviceType.FIXED_WING.getCotType())) &&
//			telemPackage.getArmed() &&
//			telemPackage.getAltitude() > IN_AIR_ALTITUDE) {
//			return true;
//		}
        double elev = Utils.EARTH.getElevation(Angle.fromDegreesLatitude(telemPackage.getLatitude()),
                Angle.fromDegreesLongitude(telemPackage.getLongitude()));
        double alt = telemPackage.getAltitude() - elev;
        if (isAirAgent(telemPackage) &&
                alt > IN_AIR_ALTITUDE) {
            return true;
        }
        return false;
    }

    private boolean isAirAgent(AgentTelemPackage telemPackage) {
//		if (telemPackage.getPlatformCotType().equals(DeviceType.ROTOR_CRAFT.getCotType()) ||
//			 telemPackage.getPlatformCotType().equals(DeviceType.FIXED_WING.getCotType())) {
//			return true;
//		}
//		return false;
        String uid = telemPackage.getUid().toLowerCase();
        if (uid.contains("quad")){
            return true;
        }
        return false;
    }

    private boolean isGroundAgent(AgentTelemPackage telemPackage) {
//		if (telemPackage.getPlatformCotType().equals(DeviceType.ROVER.getCotType())) {
//			return true;
//		}
        String uid = telemPackage.getUid().toLowerCase();
        if (uid.contains("rover")){
            return true;
        }
        return false;
    }

    public void addOrUpdateAll(Map<String, AgentTelemPackage> tacticsFromAll) {
        tableModel.clear();

        int totalNbrAgents = 0;
        int nbrAgentsInAir = 0;
        int totalNbrAirAgents = 0;
        int totalNbrGroundAgents = 0;

        for (String uid : tacticsFromAll.keySet()) {
            AgentTelemPackage telemPackage = tacticsFromAll.get(uid);
            totalNbrAgents++;

            if (isAirAgent(telemPackage)) {
                totalNbrAirAgents++;
            }
            if (isGroundAgent(telemPackage)) {
                totalNbrGroundAgents++;
            }
            if (isAgentInAir(telemPackage)) {
                nbrAgentsInAir++;
            }
            synchronized (uidToTelemMap) {
                uidToTelemMap.put(uid, telemPackage);
            }
        }

        tableModel.addOrUpdate(tacticsFromAll);
        nbrAgents.setText("" + totalNbrAgents);
        nbrGroundAgents.setText("" + totalNbrGroundAgents);
        nbrAirAgents.setText("" + totalNbrAirAgents);
        agentsInAir.setText("" + nbrAgentsInAir);
    }

    public double getTimeScale() {
        return timeScale;
    }
}
