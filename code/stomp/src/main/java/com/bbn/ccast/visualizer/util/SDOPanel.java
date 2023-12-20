//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */

package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.visualizer.tracks.SDOTrack;
import com.bbn.ccast.visualizer.tracks.TrackController;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

/**
 * @author tag
 * @version $Id: TrackPanel.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class SDOPanel extends JPanel {

	private static final long serialVersionUID = 1L;

	private String elevationUnit;
	private String angleFormat;

	private JCheckBox visibilityFlag;
	private JCheckBox unorderedTrackList;
	private JScrollPane scrollPane;
	private PositionTable positionTable;

	public SDOPanel() {
		this.initComponents();
		this.layoutComponents();

		this.scrollPane.addMouseListener(new PositionsContextMenu(this.positionTable));
		this.positionTable.addMouseListener(new PositionsContextMenu(this.positionTable));
	}

	private void initComponents() {
		this.setToolTipText("Track Positions");

		this.visibilityFlag = new JCheckBox();
		this.scrollPane = new JScrollPane();
		this.positionTable = new PositionTable();
		this.unorderedTrackList = new JCheckBox();
	}

	protected void layoutComponents() {
		setLayout(new BorderLayout(0, 0)); // hgap, vgap
		this.setOpaque(false);

		// ======== topPanel ========
		JPanel topPanel = new JPanel();
		{
			topPanel.setLayout(new BoxLayout(topPanel, BoxLayout.LINE_AXIS));
			topPanel.setBorder(BorderFactory.createEmptyBorder(0, 5, 10, 5)); // top, left, bottom, right
			topPanel.setOpaque(false);

			// ---- visibilityFlag ----
			this.visibilityFlag.setText("Show Track");
			this.visibilityFlag.setSelected(true);
			this.visibilityFlag.setOpaque(false);
			this.visibilityFlag.setToolTipText("Display track on the globe");
			this.visibilityFlag.addActionListener(new ActionListener() {
				@Override
				public void actionPerformed(ActionEvent e) {
					visibilityActionPerformed();
				}
			});
			topPanel.add(this.visibilityFlag);
			topPanel.add(Box.createHorizontalStrut(15));

			// ---- ordered track list
			this.unorderedTrackList.setText("Unordered List");
			this.unorderedTrackList.setSelected(false);
			this.unorderedTrackList.setOpaque(false);
			this.unorderedTrackList.setToolTipText("Track points not in any order");
			this.unorderedTrackList.addActionListener(new ActionListener() {
				@Override
				public void actionPerformed(ActionEvent e) {
					unorderedTrackListActionPerformed();
				}
			});
			topPanel.add(this.unorderedTrackList);
			topPanel.add(Box.createHorizontalStrut(15));	
		}
		this.add(topPanel, BorderLayout.NORTH);

		// ======== scrollPane ========
		{
			this.scrollPane.setVerticalScrollBarPolicy(ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS);

			// ---- positionTable ----
			this.positionTable.setPreferredScrollableViewportSize(new Dimension(340, 300));
			this.scrollPane.setViewportView(this.positionTable);
		}
		this.add(this.scrollPane, BorderLayout.CENTER);
	}
		
	/**
	 * @return the visibilityFlag
	 */
	public JCheckBox getVisibilityFlag() {
		return visibilityFlag;
	}

	public JCheckBox getUnorderedTrackList() {
		return this.unorderedTrackList;
	}

	public void setTrack(SDOTrack sarTrack) {
		this.positionTable.setSDOTrack(sarTrack);
	}

	public SDOTrack getTrack() {
		return this.positionTable.getSDOTrack();
	}
	
	/**
	 * @return the positionTable
	 */
	public PositionTable getPositionTable() {
		return positionTable;
	}

	public String getElevationUnit() {
		return this.elevationUnit;
	}

	public void setElevationUnit(String unit) {
		//String oldValue = this.elevationUnit;
		this.elevationUnit = unit;

		this.positionTable.setElevationUnit(unit);
		this.positionTable.updateTableData();
//		this.changeOffsetUnit(oldValue, this.elevationUnit);
	}

	public String getAngleFormat() {
		return this.angleFormat;
	}

	public void setAngleFormat(String format) {
		this.angleFormat = format;
		this.positionTable.setAngleFormat(format);
		this.positionTable.updateTableData();
	}

	private void visibilityActionPerformed() {
		String vis = this.visibilityFlag.isSelected() ? TrackController.TRACK_ENABLE : TrackController.TRACK_DISABLE;
		if (this.positionTable != null) {
			this.positionTable.getSDOTrack().firePropertyChange(vis, null, this.positionTable.getSDOTrack());
		}
	}
	
	private void unorderedTrackListActionPerformed() {
		this.positionTable.getSDOTrack().setUnordered(this.unorderedTrackList.isSelected());
		String vis = this.unorderedTrackList.isSelected() ? TrackController.TRACK_UNORDERED : TrackController.TRACK_ORDERED;
		if (this.positionTable != null) {
			this.positionTable.getSDOTrack().firePropertyChange(vis, null, this.positionTable.getSDOTrack());	
		}
	}

}
