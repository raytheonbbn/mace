//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tactics;

import com.bbn.ccast.nullsim.blenodes.SimTargetType;
import com.bbn.ccast.visualizer.WorldWindAppFrame;
import com.bbn.ccast.visualizer.util.MapLocation;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;

public class ChooseMapDialog implements ActionListener {
    private JDialog dialog;
    private JComboBox<String> typeComboBox;
    private final WorldWindAppFrame frame;

    public ChooseMapDialog(WorldWindAppFrame frame) {
        this.frame = frame;
    }

    public void setup() {
        dialog = new JDialog(frame);
        dialog.setPreferredSize(new Dimension(400, 90));
        GridLayout gridLayout = new GridLayout(1, 3, 5, 5);
        JPanel formPanel = new JPanel();
        formPanel.setLayout(new FlowLayout());
        formPanel.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        dialog.add(formPanel);


        DefaultComboBoxModel<String> dcbm = new DefaultComboBoxModel<>();
        for (MapLocation loc : MapLocation.values()) {
            dcbm.addElement(loc.getName());
        }
        JLabel typeLabel = new JLabel("Map Of:");
        formPanel.add(typeLabel);

        this.typeComboBox = new JComboBox<String>(dcbm);
        typeComboBox.setSelectedIndex(0);
        typeComboBox.setPreferredSize(new Dimension(200, 30));
        formPanel.add(typeComboBox);

        JButton exec = new JButton();
        exec.setText("Apply");
        formPanel.add(exec);
        exec.addActionListener(this);
        dialog.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
        dialog.pack();
        dialog.setVisible(true);
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        MapLocation map = MapLocation.RAYTHEON;
        String selectedName = (String) typeComboBox.getSelectedItem();
        for (MapLocation loc : MapLocation.values()) {
            if (selectedName.equals(loc.getName()))
                map = loc;
        }
        frame.getVisualization().centerMapOn(map.getPos());
        frame.getVisualization().resetAgentPositions(map.getPos());
        dialog.setVisible(false);
        dialog.dispatchEvent(new WindowEvent(dialog, WindowEvent.WINDOW_CLOSING));
    }
}
