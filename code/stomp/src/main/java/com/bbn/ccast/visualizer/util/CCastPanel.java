//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionListener;

public abstract class CCastPanel extends JPanel {

    public CCastPanel(BorderLayout borderLayout) {
        super(borderLayout);

    }

    protected JButton createButton(String name, ActionListener action) {
        JButton button = new JButton(name);
        button.setFont(new Font("T", 1, 11));
        button.setPreferredSize(new Dimension(140,30));
        button.addActionListener(action);
        return button;
    }

}
