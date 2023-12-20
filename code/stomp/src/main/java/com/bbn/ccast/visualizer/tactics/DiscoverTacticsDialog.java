//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tactics;

import com.bbn.ccast.nullsim.SimVehicle;
import com.bbn.ccast.nullsim.SimVehicle.Intent;
import com.bbn.ccast.nullsim.blenodes.SimTargetManager;
import com.bbn.ccast.nullsim.blenodes.SimTargetType;
import com.bbn.ccast.nullsim.blenodes.TargetEventHandler;
import com.bbn.ccast.visualizer.WorldWindAppFrame;
import com.bbn.ccast.visualizer.tracks.SDOTrack;
import com.bbn.ccast.visualizer.util.RadioButtonPanel;
import org.apache.log4j.Logger;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.ParseException;
import java.util.*;

public class DiscoverTacticsDialog extends TacticsDialog {
    private static final Logger logger = Logger.getLogger(DiscoverTacticsDialog.class.getName());

    private JRadioButton discoveredButton;
    private JRadioButton undiscoveredButton;

    private volatile boolean discovered = false;

    public DiscoverTacticsDialog(WorldWindAppFrame parent, Dialog.ModalityType modalityType,
                                 EditingActivity editingActivity) {
        super(parent, "Set Target Discovered Status", modalityType, editingActivity, "Intent",
                EnumSet.noneOf(DataSources.class),
                EnumSet.noneOf(OnCompletionActivities.class),
                false);
        setupDiscoverPanel();
        agentPanel.setVisible(false);
        targetPanel.setVisible(true);
    }

    @Override
    public void setupCustomPanel(){
        setupDiscoverPanel();
    }


    private void setupDiscoverPanel() {
        customPanel = new JPanel();
        GridLayout gridLayout = new GridLayout(5, 2, 1, 1);
        customPanel.setPreferredSize(new Dimension(100, 100));
        customPanel.setLayout(gridLayout);
        customPanel.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));

        discoveredButton = new JRadioButton("Discovered");
        discoveredButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent actionEvent) {
                discovered = true;
            }
        });
        undiscoveredButton = new JRadioButton("Undiscovered");
        undiscoveredButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent actionEvent) {
                discovered = false;
            }
        });

        JRadioButton[] radioButtonArray = new JRadioButton[2];
        radioButtonArray[0] = discoveredButton;
        radioButtonArray[1] = undiscoveredButton;
        RadioButtonPanel radioPanel = new RadioButtonPanel(RadioButtonPanel.VERTICAL, "Discovered State", radioButtonArray);
        radioPanel.setLayout(new FlowLayout());
        radioButtonArray[1].setSelected(true);

        customPanel.add(radioPanel);

    }

    @Override
    protected void setupPositionsPanel() {
        return;
    }

    @Override
    protected boolean executeCommand(String[] agentIDs) {
        
        String[] targetIDs = getUidsFromCheckBoxGroupLabels(targetCheckBoxGroup.getSelected());
        boolean sim = worldWindAppFrame.getVisualization().getConfiguration().isInSim();

        double range = 0;
        int payloadsRequired = 0;
        double captureCountdown = 0;

        boolean isDiscovered = discovered;
        TargetEventHandler targetEventHandler = worldWindAppFrame.getVisualization().getTargetEventHandler();
        targetEventHandler.sendSetDiscoverCommand(isDiscovered, targetIDs);
        return true;
    }

    @Override
    protected boolean executeCommand(String[] agentIDs, SDOTrack sdoPositionTrack) {
        return executeCommand(agentIDs);
    }
}
