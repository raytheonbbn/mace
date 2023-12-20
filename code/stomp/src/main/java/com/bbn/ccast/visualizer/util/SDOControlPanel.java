//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */

package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.visualizer.tracks.TrackController;

import javax.swing.*;
import java.awt.*;

/**
 * @author tag
 * @version $Id: ControlPanel.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class SDOControlPanel extends JPanel {
	private static final long serialVersionUID = 1L;

	protected SDOListPanel sdoListPanel;
	protected SDOPanel sdoPanel;

	public SDOControlPanel() {
		super(new BorderLayout());
		this.initComponents();
		this.layoutComponents();
		this.setVisible(true);

	}

	public void setTrackController(TrackController trackController) {
		if (this.sdoListPanel != null) {
			this.sdoListPanel.setTrackController(trackController);
		}
	}
	
	public SDOListPanel getSdoListPanel() {
		return this.sdoListPanel;
	}

	public SDOPanel getSdoPanel() {
		return this.sdoPanel;
	}

	/*
	 * public AnalysisPanel getAnalysisPanel() { return this.analysisPanel; }
	 */
	protected void initComponents() {
		this.sdoPanel = new SDOPanel();
		this.sdoListPanel = new SDOListPanel(sdoPanel);		
	}

	protected void layoutComponents() {
		this.setLayout(new BorderLayout(0, 0)); // hgap, vgap

		// this.analysisPanel.setBorder(BorderFactory.createEmptyBorder(30, 10, 0, 10));
		// // top, left, bottom, right

		// Create a vertical split pane with a continuous layout. Put the track table
		// panel in the top, and put the
		// track controls panel in the bottom.
		JSplitPane splitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT, true, this.sdoListPanel, this.sdoPanel);
		splitPane.setBorder(BorderFactory.createEmptyBorder());
		splitPane.setResizeWeight(0.5);
//		this.add(this.sdoListPanel, BorderLayout.CENTER);
		this.add(splitPane, BorderLayout.CENTER);
	}
}
