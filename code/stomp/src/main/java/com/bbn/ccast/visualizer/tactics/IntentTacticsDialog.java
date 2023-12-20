//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tactics;

import com.bbn.ccast.TargetTelemPackage;
import com.bbn.ccast.nullsim.SimVehicle;
import com.bbn.ccast.nullsim.SimVehicle.Intent;
import com.bbn.ccast.nullsim.blenodes.PayloadEventHandler;
import com.bbn.ccast.nullsim.blenodes.SimTargetManager;
import com.bbn.ccast.nullsim.blenodes.SimTargetType;
import com.bbn.ccast.nullsim.blenodes.TargetEventHandler;
import com.bbn.ccast.visualizer.WorldWindAppFrame;
import com.bbn.ccast.visualizer.tracks.SDOTrack;
import com.bbn.ccast.nullsim.blenodes.SimTargetConfig;
import org.apache.log4j.Logger;

import java.awt.*;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.text.NumberFormat;
import java.text.ParseException;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

import javax.swing.*;
import javax.swing.text.NumberFormatter;

public class IntentTacticsDialog extends TacticsDialog {
    private static final Logger logger = Logger.getLogger(IntentTacticsDialog.class.getName());

    private PayloadEventHandler payloadEventHandler;
    private TargetEventHandler targetEventHandler;
    private final JPanel agentIntentPanel = new JPanel();
    private final JPanel targetTypePanel = new JPanel();
    private JComboBox<String> intentSelection1;
    private JComboBox<String> intentSelection2;
    private JComboBox<String> intentSelection3;
    private JComboBox<String> suppSelection;
    private JLabel suppressionLabel;
    private JLabel intentLabel2;
    private JLabel intentLabel3;
    private JComboBox<SimTargetType> typeSelection;
    private JLabel networkNameLabel;
    private JFormattedTextField networkNameTextField;
    private JLabel rangeLabel;
    private JFormattedTextField rangeTextField;
    private JLabel payloadsRequiredLabel;
    private JFormattedTextField payloadsRequiredTextField;
    private JLabel countdownLabel;
    private JFormattedTextField captureCountdownTextField;
    private NumberFormat formatDouble;
    private NumberFormat formatInt;
    private NumberFormat formatCaptureCountdown;
    private JLabel discoverDistanceLabel;
    private JFormattedTextField discoverDistanceTextField;
    private boolean isTarget;
    private volatile boolean threadLocked;

    public IntentTacticsDialog(WorldWindAppFrame parent, Dialog.ModalityType modalityType,
                               EditingActivity editingActivity) {
        super(parent, "Payload/Target Configuration", modalityType, editingActivity, "Intent",
                EnumSet.noneOf(DataSources.class),
                EnumSet.noneOf(OnCompletionActivities.class),
                false);

        isTarget = false;
        typeSelection();
        intentSelection();
        targetOrAgentToggle();
        refreshTypeSelection();
    }

    private void targetOrAgentToggle() {
        JPanel typePanel = new JPanel();
        JToggleButton payloadButton = new JToggleButton("Payloads");
        JToggleButton targetButton = new JToggleButton("Targets");

        ItemListener payloadListener = e -> {
            if (e.getStateChange() == ItemEvent.SELECTED) {
                targetButton.setSelected(false);
                setTargetPanelVisibility(false);
                setAgentPanelVisibility(true);
                targetTypePanel.setVisible(false);
                agentIntentPanel.setVisible(true);
                isTarget = false;
                refreshTypeSelection();
            } else if (e.getStateChange() == ItemEvent.DESELECTED && !targetButton.isSelected()) {
                payloadButton.setSelected(true);
            }
        };
        ItemListener targetListener = e -> {
            if (e.getStateChange() == ItemEvent.SELECTED) {
                payloadButton.setSelected(false);
                setAgentPanelVisibility(false);
                setTargetPanelVisibility(true);
                targetTypePanel.setVisible(true);
                agentIntentPanel.setVisible(false);
                isTarget = true;
                refreshTypeSelection();
            } else if (e.getStateChange() == ItemEvent.DESELECTED && !payloadButton.isSelected()) {
                targetButton.setSelected(true);
            }
        };

        payloadButton.addItemListener(payloadListener);
        targetButton.addItemListener(targetListener);
        typePanel.add(payloadButton);
        typePanel.add(targetButton);

        payloadButton.setSelected(true);
        payloadListener.itemStateChanged(new ItemEvent(payloadButton, ItemEvent.ITEM_STATE_CHANGED, null,
                ItemEvent.SELECTED));

        GridBagConstraints gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0; // was 3
        gbc.gridheight = 1;
        gbc.gridwidth = 2;
        gbc.anchor = GridBagConstraints.NORTH;
        gbc.fill = GridBagConstraints.NONE;
        gbc.weightx = 0.0;
        gbc.weighty = 0.0;

        dialog.getContentPane().add(typePanel, gbc);
    }

    private void intentSelection() {
        GridLayout gridLayout = new GridLayout(3, 2, 1, 1);
        agentIntentPanel.setPreferredSize(new Dimension(360, 100));
        agentIntentPanel.setLayout(gridLayout);
        agentIntentPanel.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));

        intentSelection1 = new JComboBox<>();
        intentSelection2 = new JComboBox<>();
        intentSelection3 = new JComboBox<>();

        refreshIntentSelection(true);

        intentSelection1.setSelectedIndex(0);
        intentSelection1.setPreferredSize(new Dimension(100, 30));
        JLabel intentLabel1 = new JLabel("Intent 1");
        agentIntentPanel.add(intentLabel1);
        agentIntentPanel.add(intentSelection1);

        intentSelection2.setSelectedIndex(0);
        intentSelection2.setPreferredSize(new Dimension(100, 30));
        intentSelection2.setVisible(false);
        intentLabel2 = new JLabel("Intent 2");
        intentLabel2.setVisible(false);
        agentIntentPanel.add(intentLabel2);
        agentIntentPanel.add(intentSelection2);

        intentSelection3.setSelectedIndex(0);
        intentSelection3.setPreferredSize(new Dimension(100, 30));
        intentSelection3.setVisible(false);
        intentLabel3 = new JLabel("Intent 3");
        intentLabel3.setVisible(false);
        agentIntentPanel.add(intentLabel3);
        agentIntentPanel.add(intentSelection3);

        ArrayList<Object> selected = new ArrayList<>();
        selected.add(intentSelection1.getSelectedItem());
        selected.add(intentSelection2.getSelectedItem());
        selected.add(intentSelection3.getSelectedItem());

        Runnable checkSelected = () -> {
            while (dialog.isVisible()) {
                if (intentSelection1.getSelectedItem() != null && intentSelection2.getSelectedItem() != null
                        && intentSelection3.getSelectedItem() != null) {
                    if (!Objects.equals(intentSelection1.getSelectedItem(), selected.get(0))
                            || !Objects.equals(intentSelection2.getSelectedItem(), selected.get(1))
                            || !Objects.equals(intentSelection3.getSelectedItem(), selected.get(2))) {
                        if (intentSelection1.getSelectedItem().toString().equals("IDLE")) {
                            while (threadLocked) Thread.onSpinWait();
                            refreshIntentSelection(true);
                            intentLabel2.setVisible(false);
                            intentSelection2.setVisible(false);
                            intentLabel3.setVisible(false);
                            intentSelection3.setVisible(false);
                        } else {
                            while (threadLocked) Thread.onSpinWait();
                            intentLabel2.setVisible(true);
                            intentSelection2.setVisible(true);
                            intentLabel3.setVisible(true);
                            intentSelection3.setVisible(true);
                            refreshIntentSelection(false);
                        }
                        selected.clear();
                        selected.add(intentSelection1.getSelectedItem());
                        selected.add(intentSelection2.getSelectedItem());
                        selected.add(intentSelection3.getSelectedItem());
                    }
                }
            }
        };

        Thread intentThread = new Thread(checkSelected);
        intentThread.start();

        dataPanel.add(agentIntentPanel, new GridBagConstraints());
    }

    private synchronized void refreshIntentSelection(boolean init) {
        threadLocked = true;
        ArrayList<String> selected = new ArrayList<>();
        if (!init) {
            selected.add(intentSelection1.getSelectedItem().toString());
            selected.add(intentSelection2.getSelectedItem().toString());
            selected.add(intentSelection3.getSelectedItem().toString());
        }

        DefaultComboBoxModel<String> dcbm1 = new DefaultComboBoxModel<>();
        DefaultComboBoxModel<String> dcbm2 = new DefaultComboBoxModel<>();
        DefaultComboBoxModel<String> dcbm3 = new DefaultComboBoxModel<>();

        dcbm2.addElement("none");
        dcbm3.addElement("none");
        for (Intent intent : Intent.values()) {
            if (!intent.equals(Intent.FAIL)) {
                if (!intent.equals(Intent.IDLE)) {
                    dcbm2.addElement(intent.name());
                    dcbm3.addElement(intent.name());
                }
                dcbm1.addElement(intent.name());
            }
        }

        intentSelection1.setModel(dcbm1);
        intentSelection2.setModel(dcbm2);
        intentSelection3.setModel(dcbm3);

        if (init) {
            intentSelection1.setSelectedIndex(0);
            intentSelection2.setSelectedIndex(0);
            intentSelection3.setSelectedIndex(0);
        }

        if (!init) {
            ArrayList<JComboBox<String>> cbs = new ArrayList<>();
            cbs.add(intentSelection1);
            cbs.add(intentSelection2);
            cbs.add(intentSelection3);

            for (JComboBox<String> cb : cbs) {
                for (String ss : selected) {
                    for (int ii = 0; ii < cb.getItemCount(); ii++) {
                        if (cb.getItemAt(ii).equals(ss)) {
                            if (cbs.indexOf(cb) == selected.indexOf(ss))
                                cb.setSelectedItem(ss);
                            else if (!ss.equals("none"))
                                cb.removeItem(ss);
                        }
                    }
                }
            }
        }
        threadLocked = false;
    }

    private void typeSelection() {
        // GridLayout (NumRows, NumColumns, RowGap, ColGap)
        GridLayout gridLayout = new GridLayout(7, 2, 1, 1);
        targetTypePanel.setPreferredSize(new Dimension(360, 180));
        targetTypePanel.setLayout(gridLayout);
        // targetTypePanel.setBorder(BorderFactory.createEmptyBorder(1, 1, 1, 1));

        DefaultComboBoxModel<SimTargetType> dcbm = new DefaultComboBoxModel<>();
        for (SimTargetType type : SimTargetType.values()) {
            dcbm.addElement(type);
        }
        JLabel typeLabel = new JLabel("Type");
        targetTypePanel.add(typeLabel);

        typeSelection = new JComboBox<>(dcbm);
        typeSelection.setSelectedIndex(0);
        typeSelection.setPreferredSize(new Dimension(100, 30));
        targetTypePanel.add(typeSelection);

        formatDouble = NumberFormat.getInstance();
        formatDouble.setMaximumFractionDigits(1);
        NumberFormatter doubleFormatter = new NumberFormatter(formatDouble);
        doubleFormatter.setValueClass(Double.class);
        doubleFormatter.setMinimum(0.0);
        doubleFormatter.setMaximum(Double.MAX_VALUE);
        doubleFormatter.setAllowsInvalid(false);

        formatInt = NumberFormat.getInstance();
        NumberFormatter intFormatter = new NumberFormatter(formatInt);
        intFormatter.setValueClass(Integer.class);
        intFormatter.setMinimum(0);
        intFormatter.setMaximum(Integer.MAX_VALUE);
        intFormatter.setAllowsInvalid(false);

        discoverDistanceLabel = new JLabel("Discover Distance (m)");
        discoverDistanceLabel.setVisible(false);
        discoverDistanceLabel.setToolTipText("A payload must be at or within this distance to discover this target.");
        targetTypePanel.add(discoverDistanceLabel);
        discoverDistanceTextField = new JFormattedTextField(formatDouble);
        discoverDistanceTextField.setPreferredSize(new Dimension(100, 25));
        discoverDistanceTextField.setText("5");
        discoverDistanceTextField.setVisible(false);
        targetTypePanel.add(discoverDistanceTextField);

        rangeLabel = new JLabel("Capture Range (m)");
        rangeLabel.setVisible(false);
        rangeLabel.setToolTipText("A payload must be at or within this distance to capture this target.");
        targetTypePanel.add(rangeLabel);
        rangeTextField = new JFormattedTextField(formatDouble);
        rangeTextField.setPreferredSize(new Dimension(100, 25));
        rangeTextField.setText("5");
        rangeTextField.setVisible(false);
        targetTypePanel.add(rangeTextField);

        payloadsRequiredLabel = new JLabel("Payloads Required");
        payloadsRequiredLabel.setVisible(false);
        payloadsRequiredLabel.setToolTipText("The number of payloads that must be in capture range to disable this target.");
        targetTypePanel.add(payloadsRequiredLabel);
        payloadsRequiredTextField = new JFormattedTextField(formatInt);
        payloadsRequiredTextField.setPreferredSize(new Dimension(100, 25));
        payloadsRequiredTextField.setText("1");
        payloadsRequiredTextField.setVisible(false);
        targetTypePanel.add(payloadsRequiredTextField);

        formatCaptureCountdown = NumberFormat.getInstance();
        formatCaptureCountdown.setMaximumFractionDigits(1);
        NumberFormatter countdownFormatter = new NumberFormatter(formatCaptureCountdown);
        countdownFormatter.setValueClass(Double.class);
        countdownFormatter.setMinimum(0.0);
        countdownFormatter.setMaximum(Double.MAX_VALUE);
        countdownFormatter.setAllowsInvalid(false);

        countdownLabel = new JLabel("Capture Countdown (s)");
        countdownLabel.setVisible(false);
        targetTypePanel.add(countdownLabel);
        captureCountdownTextField = new JFormattedTextField(formatDouble);
        captureCountdownTextField.setPreferredSize(new Dimension(100, 25));
        captureCountdownTextField.setText("30");
        captureCountdownTextField.setVisible(false);
        targetTypePanel.add(captureCountdownTextField);

        networkNameLabel = new JLabel("Network Name");
        networkNameLabel.setVisible(false);
        targetTypePanel.add(networkNameLabel);
        networkNameTextField = new JFormattedTextField();
        networkNameTextField.setPreferredSize(new Dimension(100, 25));
        networkNameTextField.setText("");
        networkNameTextField.setVisible(false);
        targetTypePanel.add(networkNameTextField);

        suppressionLabel = new JLabel("Suppression");
        targetTypePanel.add(suppressionLabel);
        DefaultComboBoxModel<String> suppression = new DefaultComboBoxModel<>();
        suppression.addElement("False");
        suppression.addElement("True");
        suppressionLabel.setToolTipText("Should this target remain captured when no longer interacting with payload(s)?");

        suppSelection = new JComboBox<>(suppression);
        suppSelection.setSelectedIndex(0);
        suppSelection.setPreferredSize(new Dimension(100, 30));
        targetTypePanel.add(suppSelection);

        typeSelection.addActionListener(e -> refreshTypeSelection());

        dataPanel.add(targetTypePanel, new GridBagConstraints());

        this.payloadEventHandler =
                new PayloadEventHandler(this.worldWindAppFrame.getVisualization().maceMessageTransport);
        this.targetEventHandler = worldWindAppFrame.getVisualization().getTargetEventHandler();
    }

    private void refreshTypeSelection() {

        if (!isTarget) {
            networkNameLabel.setVisible(false);
            networkNameTextField.setVisible(false);

            rangeTextField.setVisible(false);
            rangeLabel.setVisible(false);

            payloadsRequiredTextField.setVisible(false);
            payloadsRequiredLabel.setVisible(false);

            countdownLabel.setVisible(false);
            captureCountdownTextField.setVisible(false);

            suppressionLabel.setVisible(false);
            suppSelection.setVisible(false);

            discoverDistanceLabel.setVisible(false);
            discoverDistanceTextField.setVisible(false);
        } else {
            SimTargetType type = (SimTargetType) typeSelection.getSelectedItem();

            if (type == SimTargetType.IDLE) {
                typeSelection.setToolTipText("Idle type target.");
                networkNameLabel.setVisible(false);
                networkNameTextField.setVisible(false);

                rangeTextField.setVisible(false);
                rangeLabel.setVisible(false);

                payloadsRequiredTextField.setVisible(false);
                payloadsRequiredLabel.setVisible(false);

                countdownLabel.setVisible(false);
                captureCountdownTextField.setVisible(false);

                suppressionLabel.setVisible(false);
                suppSelection.setVisible(false);

                discoverDistanceLabel.setVisible(true);
                discoverDistanceTextField.setVisible(true);
            } else if (type == SimTargetType.MASS) {
                typeSelection.setToolTipText("Mass type target.");
                networkNameLabel.setVisible(false);
                networkNameTextField.setVisible(false);

                rangeTextField.setVisible(true);
                rangeLabel.setVisible(true);
                
                payloadsRequiredTextField.setText("1");
                payloadsRequiredTextField.setVisible(true);
                payloadsRequiredLabel.setVisible(true);

                countdownLabel.setVisible(false);
                captureCountdownTextField.setVisible(false);

                suppressionLabel.setVisible(true);
                suppSelection.setVisible(true);

                discoverDistanceLabel.setVisible(true);
                discoverDistanceTextField.setVisible(true);
            } else if (type == SimTargetType.PERI) {
                typeSelection.setToolTipText("Periodic type target.");
                networkNameLabel.setVisible(false);
                networkNameTextField.setVisible(false);

                rangeTextField.setVisible(true);
                rangeLabel.setVisible(true);

                // Periodic targets behave the same as mass with only one req payload, so default to 2
                payloadsRequiredTextField.setText("2");
                payloadsRequiredTextField.setVisible(true);
                payloadsRequiredLabel.setVisible(true);
                
                countdownLabel.setVisible(true);
                captureCountdownTextField.setVisible(true);

                suppressionLabel.setVisible(true);
                suppSelection.setVisible(true);

                discoverDistanceLabel.setVisible(true);
                discoverDistanceTextField.setVisible(true);
            } else if (type == SimTargetType.LINK) {
                typeSelection.setToolTipText("Linked type target.");
                networkNameLabel.setVisible(true);
                networkNameTextField.setVisible(true);

                rangeTextField.setVisible(true);
                rangeLabel.setVisible(true);

                payloadsRequiredTextField.setVisible(false);
                payloadsRequiredLabel.setVisible(false);

                countdownLabel.setVisible(false);
                captureCountdownTextField.setVisible(false);

                suppressionLabel.setVisible(true);
                suppSelection.setVisible(true);

                discoverDistanceLabel.setVisible(true);
                discoverDistanceTextField.setVisible(true);
            }
        }
    }

    private boolean execute(String[] checkBoxLabels) {
        String[] payloadOrTargetIDs = getUidsFromCheckBoxGroupLabels(checkBoxLabels);
        
        boolean sim = worldWindAppFrame.getVisualization().getConfiguration().isInSim();

        if (isTarget) {
//            SimTargetConfig lastSelectedTargetConfig = SimTargetManager.instance.getSimTargetConfigFromID(targetCheckBoxGroup.getLastSelected());
//            System.out.println(lastSelectedTargetConfig);
            System.out.println(targetCheckBoxGroup.getLastSelected());
            //TODO: figure out a way to populate the current stats of a target
            // SimTargetManager.instance.getSimTargetConfigFromID(//put ID HERE)

            SimTargetType type = (SimTargetType) typeSelection.getSelectedItem();
            boolean suppression = Objects.equals(suppSelection.getSelectedItem(), "True");

            double captureRange = 5;
            int payloadsRequired = 0;
            double captureCountdown = 0;
            double discoverDistance = 5;

            try {
                discoverDistance = formatDouble.parse(discoverDistanceTextField.getText()).doubleValue();
            } catch (ParseException e1) {
                logger.error("Invalid discover distance: " + discoverDistanceTextField.getText());
            }

            

            assert type != null;
            if (type.equals(SimTargetType.MASS) || type.equals(SimTargetType.PERI)) {
                try {
                    captureRange = formatDouble.parse(rangeTextField.getText()).doubleValue();
                } catch (ParseException ex) {
                    logger.error("Invalid capture range: " + rangeTextField.getText());
                }
                try {
                    payloadsRequired = formatInt.parse(payloadsRequiredTextField.getText()).intValue();
                } catch (ParseException ex) {
                    logger.error("Invalid payloads required: " + payloadsRequiredTextField.getText());
                }
            }

            // Check user input
            if (discoverDistance < captureRange) {
                String warningMessage = "Discover Distance cannot be less than Capture Distance!";
                JOptionPane.showMessageDialog(worldWindAppFrame, warningMessage, "Warning",
                        JOptionPane.ERROR_MESSAGE);
                return false;
            }

            

            switch (type) {
                case MASS:
                    for (String uid : payloadOrTargetIDs) {
                        targetEventHandler.sendMassTargetConfig(uid, captureRange, payloadsRequired, suppression);
                        if (sim)
                            SimTargetManager.instance.configureMassTarget(uid, captureRange, payloadsRequired, suppression, discoverDistance);
                    }
                    break;
                case PERI:
                    try {
                        captureCountdown =
                                formatCaptureCountdown.parse(captureCountdownTextField.getText()).doubleValue();
                    } catch (ParseException ex) {
                        logger.error(String.format("Invalid capture countdown: %s",
                                captureCountdownTextField.getText()));
                    }

                    for (String uid : payloadOrTargetIDs) {
                        targetEventHandler.sendPeriTargetConfig(uid, captureRange, payloadsRequired,
                                captureCountdown, suppression);
                        if (sim)
                            SimTargetManager.instance.configurePeriTarget(uid, captureRange, payloadsRequired,
                                    captureCountdown, suppression, discoverDistance);
                    }
                    break;
                case LINK:
                    var networkName = networkNameTextField.getText();
                    if (networkName == null || networkName.isEmpty()) {
                        logger.error("Empty network name");
                        return false;
                    }

                    try {
                        captureRange = formatDouble.parse(rangeTextField.getText()).doubleValue();
                    } catch (ParseException e) {
                        logger.error("Invalid capture range: " + rangeTextField.getText());
                    }

                    java.util.List<String> targets = Arrays.asList(payloadOrTargetIDs);
                    targetEventHandler.sendNetworkConfig(networkName, captureRange, targets, suppression);
                    if (sim)
                        SimTargetManager.instance.configureLinkedTargets(captureRange, networkName, targets, suppression, discoverDistance);
                    break;
                default:
                    for (String uid : payloadOrTargetIDs) {
                        targetEventHandler.sendIdleTargetConfig(uid);
                        if (sim) {
                            SimTargetManager.instance.configureIdleTarget(uid, discoverDistance);
                        }
                    }
                    break;
            }



            
        } else {
            Map<String, SimVehicle> vehicleMap = new HashMap<>();
            if (sim)
                vehicleMap = worldWindAppFrame.getVisualization().getNullSim().getAllVehicles();

            Collection<Intent> payloadIntent = new ArrayList<Intent>();
            if (!Intent.valueOf(intentSelection1.getSelectedItem().toString()).equals(Intent.IDLE)) {
                if (!intentSelection2.getSelectedItem().toString().equals("none")) {
                    payloadIntent.add(Intent.valueOf(intentSelection2.getSelectedItem().toString()));
                }
                if (!intentSelection3.getSelectedItem().toString().equals("none")) {
                    payloadIntent.add(Intent.valueOf(intentSelection3.getSelectedItem().toString()));
                }
            }
            payloadIntent.add(Intent.valueOf(intentSelection1.getSelectedItem().toString()));

            for (String uid : payloadOrTargetIDs) {
                // send intent configuration here
                payloadEventHandler.sendPayloadConfig(uid, payloadIntent);
                if (sim) {
                    SimVehicle vehicle = vehicleMap.get(uid);
                    if (vehicle != null) {
                        vehicle.clearIntent();
                        vehicle.setIntent(payloadIntent);
                    }
                }
            }
        }
        return true;
    }

    @Override
    protected boolean executeTactic() {
        if (isTarget) {
            String[] targetIDs = getUidsFromCheckBoxGroupLabels(targetCheckBoxGroup.getSelected());
            return executeCommand(targetIDs);
        } else {
            String[] agentIDs = getUidsFromCheckBoxGroupLabels(agentCheckBoxGroup.getSelected());
            if (sdoPositionTrack != null) {
                return executeCommand(agentIDs, sdoPositionTrack);
            } else {
                return executeCommand(agentIDs);
            }
        }
    }

    @Override
    protected boolean executeCommand(String[] checkBoxLabels) {
        String[] payloadOrTargetIDs = getUidsFromCheckBoxGroupLabels(checkBoxLabels);

        //Alert user if no payloads/targets selected
        if (payloadOrTargetIDs.length == 0){
            String warningMessage = "No payloads selected!";
            if (isTarget){ warningMessage = "No targets selected!"; }
            JOptionPane.showMessageDialog(worldWindAppFrame, warningMessage, "Warning",
                    JOptionPane.ERROR_MESSAGE);
            // return false to keep window open
            return false;
        }

        
        return execute(payloadOrTargetIDs);
    }

    @Override
    protected boolean executeCommand(String[] checkBoxLabels, SDOTrack sdoPositionTrack) {
        String[] agentIDs = getUidsFromCheckBoxGroupLabels(checkBoxLabels);
        execute(agentIDs);
        return true;
    }
}
