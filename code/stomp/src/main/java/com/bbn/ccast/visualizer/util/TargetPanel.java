//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.JScrollPane;
import javax.swing.JSlider;
import javax.swing.JSplitPane;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.JToolBar;
import javax.swing.SwingUtilities;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import javax.swing.event.PopupMenuEvent;
import javax.swing.event.PopupMenuListener;

import com.bbn.ccast.TargetTelemPackage;
import com.bbn.ccast.config.Configuration;
import com.bbn.ccast.nullsim.NullSim;
import com.bbn.ccast.visualizer.WorldWindAppFrame;

import org.apache.log4j.Logger;

import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.geom.Position;

public class TargetPanel extends JPanel {

    private static final Logger logger = Logger.getLogger(TargetPanel.class.getName());

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
    private TargetTableModel tableModel;
    private JScrollPane scrollPane;
    private Map<String, TargetTelemPackage> uidToTelemMap = new HashMap<>();
    private JPanel tablePanel;
    private JPanel metaDataPanel;
    private JTextField nbrPeriodicTargets;
    private JTextField nbrTargets;
    private JTextField nbrMassTargets;
    private JTextField nbrIdleTargets;
    private JTextField nbrLinkedTargets;
    private JLabel timeScaleLabel;

    private double timeScale;
    private WorldWindAppFrame frame;
    private Point clickLocation;
    private int popupSelectedRow;

    public TargetPanel(WorldWindAppFrame frame, WorldWindow wwd, Configuration configuration) {

        super(new BorderLayout());

        this.configuration = configuration;
        this.wwd = wwd;
        this.frame = frame;
        this.initComponents();
    }

    private void initComponents() {

        initAgentMetaData();
        initTargetTable();

        this.metaDataPanel.setMinimumSize(new Dimension(0, 0));

        JSplitPane splitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT, true, this.metaDataPanel, this.tablePanel);
        splitPane.setBorder(BorderFactory.createEmptyBorder());
        splitPane.setResizeWeight(0.15);
        this.add(splitPane, BorderLayout.CENTER);

    }

    protected void initTargetTable() {
        tableModel = new TargetTableModel(this.configuration, this.frame);
        table = new JTable(tableModel);
        table.setDefaultRenderer(Object.class, new TargetTableRenderer());

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
                            TargetTelemPackage clickedTelemPackage = null;
                            synchronized (uidToTelemMap) {
                                clickedTelemPackage = uidToTelemMap.get(clickedUid);
                            }
                            if (clickedTelemPackage != null && clickedTelemPackage.getAltitude() < 10000 ){
                                wwd.getView().setEyePosition(Position.fromDegrees(clickedTelemPackage.getLatitude(), clickedTelemPackage.getLongitude(), CLICK_JUMP_ALT));
                            }
                        }
                    } catch (IllegalArgumentException e) {
                        logger.warn("Table model had null target: ", e);
                    }
                }
            }
        });



        final JPopupMenu popupMenu = new JPopupMenu();
        popupMenu.addPopupMenuListener(new PopupMenuListener() {
            @Override
            public void popupMenuWillBecomeVisible(PopupMenuEvent e) {
                SwingUtilities.invokeLater(new Runnable() {
                    @Override
                    public void run() {
                        int rowAtPoint = table.rowAtPoint(SwingUtilities.convertPoint(popupMenu, new Point(0, 0),  table));
                        if (rowAtPoint > -1) {
                            table.setRowSelectionInterval(rowAtPoint, rowAtPoint);
                        }
                    }
                });
            }

            @Override
            public void popupMenuWillBecomeInvisible(PopupMenuEvent popupMenuEvent) {

            }

            @Override
            public void popupMenuCanceled(PopupMenuEvent popupMenuEvent) {

            }
        });

//        if(clickedTelemPackage != null) {
//            if (evt.getButton() == MouseEvent.BUTTON1) {
//                wwd.getView().setEyePosition(Position.fromDegrees(clickedTelemPackage.getLatitude(), clickedTelemPackage.getLongitude(), CLICK_JUMP_ALT));
//            }else if(evt.getButton() == MouseEvent.BUTTON2){
//                //TODO: Show mouse popup menu
//            }
//        }

        JMenuItem centerMap = new JMenuItem("Center Map On Target");
        centerMap.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent actionEvent) {
                int selectedRow = popupSelectedRow;
                try{
                    Object val = tableModel.getValueAt(selectedRow, UID_COLUMN);
                    logger.info("Clicked UID: " + val);
                    if(val != null && val instanceof String) {
                        String clickedUid = val.toString();
                        TargetTelemPackage clickedTelemPackage = null;
                        synchronized (uidToTelemMap) {
                            clickedTelemPackage = uidToTelemMap.get(clickedUid);
                        }
                        // Do an additional check that the GPS is not in default "no signal" position
                        if(clickedTelemPackage != null && clickedTelemPackage.getAltitude() < 10000){
                            wwd.getView().setEyePosition(Position.fromDegrees(clickedTelemPackage.getLatitude(), clickedTelemPackage.getLongitude(), CLICK_JUMP_ALT));
                        }
                    }
                }catch (IllegalArgumentException e){
                    logger.warn("Table model had null payload: ", e);
                }
            }
        });

        JMenuItem setTargetPosition = new JMenuItem("Set Target Position");
        setTargetPosition.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent actionEvent) {
                int selectedRow = popupSelectedRow;
                try{
                    Object val = tableModel.getValueAt(selectedRow, UID_COLUMN);
                    logger.info("Clicked UID: " + val);
                    if(val != null && val instanceof String) {
                        String clickedUid = val.toString();
                        TargetTelemPackage clickedTelemPackage = null;
                        synchronized (uidToTelemMap) {
                            clickedTelemPackage = uidToTelemMap.get(clickedUid);
                        }
                        if(clickedTelemPackage != null) {
                            frame.setSelectedTarget(clickedTelemPackage.getUid());
                            tableModel.setSelectedRow(selectedRow);
                            //TODO: prompt the user to select a new position with right-click
                        }
                    }
                }catch (IllegalArgumentException e){
                    logger.warn("Table model had null payload: ", e);
                }
            }
        });

//        JMenuItem setTargetType = new JMenuItem("Set Target Type");
//        setTargetType.addActionListener(new ActionListener() {
//            @Override
//            public void actionPerformed(ActionEvent actionEvent) {
//                int selectedRow = popupSelectedRow;
//                try{
//                    Object val = tableModel.getValueAt(selectedRow, UID_COLUMN);
//                    logger.info("Clicked UID: " + val);
//                    if(val != null && val instanceof String) {
//                        String clickedUid = val.toString();
//                        TargetTelemPackage clickedTelemPackage = null;
//                        synchronized (uidToTelemMap) {
//                            clickedTelemPackage = uidToTelemMap.get(clickedUid);
//                        }
//                        if(clickedTelemPackage != null) {
//                           //popup target config menu
//                            TargetConfigPanel targetConfigPanel = new TargetConfigPanel(frame, clickedTelemPackage);
//                            targetConfigPanel.setup();
//                        }
//                    }
//                }catch (IllegalArgumentException e){
//                    logger.warn("Table model had null payload: ", e);
//                }
//            }
//        });

        // JMenuItem removeTarget = new JMenuItem("Remove Target");
        // removeTarget.addActionListener(new ActionListener() {
        //     @Override
        //     public void actionPerformed(ActionEvent actionEvent) {
        //         if (!frame.getVisualization().getConfiguration().isInSim()){
        //             return;
        //         }

        //         int selectedRow = popupSelectedRow;
        //         try{
        //             Object val = tableModel.getValueAt(selectedRow, UID_COLUMN);
        //             logger.info("Clicked UID: " + val);
        //             if(val != null && val instanceof String) {
        //                 String clickedUid = val.toString();
        //                 TargetTelemPackage clickedTelemPackage = null;
        //                 synchronized (uidToTelemMap) {
        //                     clickedTelemPackage = uidToTelemMap.get(clickedUid);
        //                 }
        //                 if(clickedTelemPackage != null) {
        //                     logger.info("Removing sim target " + clickedTelemPackage.getUid());
        //                     NullSim nullSim = frame.getVisualization().getNullSim();
        //                     nullSim.removeSimTarget(clickedTelemPackage.getUid());
        //                 }
        //             }
        //         }catch (IllegalArgumentException e){
        //             logger.warn("Table model had null payload: ", e);
        //         }
        //     }
        // });


        // TODO: add useful right click opitons and remove the old ones
        // popupMenu.add(centerMap);
        // popupMenu.add(setTargetPosition);
        // popupMenu.add(setTargetType);
        //TODO: uncomment when we can shutdown the python for a target from here: popupMenu.add(removeTarget);


        table.addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                if (e.isPopupTrigger())
                {
                    int row = table.rowAtPoint( e.getPoint() );
                    int column = table.columnAtPoint( e.getPoint() );
                    popupSelectedRow = row;

                    popupMenu.show(e.getComponent(), e.getX(), e.getY());
                }
            }
            @Override
            public void mouseReleased(MouseEvent e) {
                if (e.isPopupTrigger())
                {
                    int row = table.rowAtPoint( e.getPoint() );
                    int column = table.columnAtPoint( e.getPoint() );
                    popupSelectedRow = row;

                    popupMenu.show(e.getComponent(), e.getX(), e.getY());
                }
            }
        });

        table.setAutoResizeMode(JTable.AUTO_RESIZE_OFF);
        scrollPane = new JScrollPane(table);

        tablePanel = new JPanel();
        tablePanel.setLayout(new BorderLayout(0,0));

        tablePanel.add(scrollPane, BorderLayout.CENTER);

        if (configuration.isTimeScalable()) {

            timeScale = configuration.getInitialTimeScale();

            timeScaleLabel = new JLabel();
            updateTimeScaleLabel();

            // Don't let scale get to zero, else we'll hit a singularity
            JSlider timeScaleSlider = new JSlider(1, (int)(TIME_SLIDER_SCALAR * MAX_TIME_SCALE));
            timeScaleSlider.setMinorTickSpacing((int)TIME_SLIDER_SCALAR);
            timeScaleSlider.setValue((int)(timeScale * TIME_SLIDER_SCALAR));
            timeScaleSlider.addChangeListener(new ChangeListener() {

                @Override
                public void stateChanged(ChangeEvent event) {
                    timeScale = timeScaleSlider.getValue() / TIME_SLIDER_SCALAR;
                    updateTimeScaleLabel();
                }});

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

        JLabel label = new JLabel("Total Targets");
        metaDataPanel.add(label);

        nbrTargets = new JTextField();
        nbrTargets.setEditable(false);
        nbrTargets.setColumns(4);
        nbrTargets.setToolTipText("Total number of targets");
        metaDataPanel.add(nbrTargets);

        label = new JLabel("Idle Targets");
        metaDataPanel.add(label);

        nbrIdleTargets = new JTextField();
        nbrIdleTargets.setEditable(false);
        nbrIdleTargets.setColumns(4);
        nbrIdleTargets.setToolTipText("Number of idle targets");
        metaDataPanel.add(nbrIdleTargets);

        label = new JLabel("Mass Targets");
        metaDataPanel.add(label);

        nbrMassTargets = new JTextField();
        nbrMassTargets.setEditable(false);
        nbrMassTargets.setColumns(4);
        nbrMassTargets.setToolTipText("Number of mass targets");
        metaDataPanel.add(nbrMassTargets);

        label = new JLabel("Periodic Targets");
        metaDataPanel.add(label);

        nbrPeriodicTargets = new JTextField();
        nbrPeriodicTargets.setEditable(false);
        nbrPeriodicTargets.setColumns(4);
        nbrPeriodicTargets.setToolTipText("Number of periodic targets");
        metaDataPanel.add(nbrPeriodicTargets);

        label = new JLabel("Linked Targets");
        metaDataPanel.add(label);

        nbrLinkedTargets = new JTextField();
        nbrLinkedTargets.setEditable(false);
        nbrLinkedTargets.setColumns(4);
        nbrLinkedTargets.setToolTipText("Number of linked targets");
        metaDataPanel.add(nbrLinkedTargets);

        JToolBar targetToolbar = new JToolBar();
        targetToolbar.setLayout(new FlowLayout());
		targetToolbar.setFloatable(false);
		targetToolbar.setRollover(true);

//        JButton button = AbstractToolBar.makeToolBarButton(null, "New Network", "New Network");
//		button.addActionListener(e -> {
//			logger.info("Network config button clicked");
//            Vector<TargetTelemPackage> targetTelems = new Vector<TargetTelemPackage>(this.uidToTelemMap.values());
//			TargetNetworkConfigPanel networkDialog = new TargetNetworkConfigPanel(frame, targetTelems);
//			networkDialog.setup();
//		});
//		targetToolbar.add(button);
        metaDataPanel.add(targetToolbar);
    }


    public void addOrUpdateAll(Map<String, TargetTelemPackage> targetTelemMap) {
        tableModel.clear();

        int totalNbrTargets = 0;
        int totalIdleTargets = 0;
        int totalMassTargets = 0;
        int totalPeriodicTargets = 0;
        int totalLinkedAgents = 0;

        for(String uid : targetTelemMap.keySet()) {
            TargetTelemPackage telemPackage = targetTelemMap.get(uid);
            totalNbrTargets++;

            switch (telemPackage.getType().toUpperCase()){
                case "IDLE":
                    totalIdleTargets++;
                    break;
                case "MASS":
                    totalMassTargets++;
                    break;
                case "PERI":
                    totalPeriodicTargets++;
                    break;
                case "LINK":
                    totalLinkedAgents++;
                    break;
                default:
                    logger.warn("Unknown target node type: " + telemPackage.getType());
                    break;
            }
            synchronized (uidToTelemMap) {
                uidToTelemMap.put(uid, telemPackage);
            }
        }

        tableModel.addOrUpdate(targetTelemMap);
        this.nbrTargets.setText(""+totalNbrTargets);
        this.nbrIdleTargets.setText(""+totalIdleTargets);
        this.nbrMassTargets.setText(""+totalMassTargets);
        this.nbrPeriodicTargets.setText(""+totalPeriodicTargets);
        this.nbrLinkedTargets.setText(""+totalLinkedAgents);
    }

    public double getTimeScale() {
        return timeScale;
    }

    public void clearSelection() {
        tableModel.setSelectedRow(-1);
    }

    public TargetTableModel getTableModel() {
        return tableModel;
    }
}
