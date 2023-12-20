//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.actions;

import com.bbn.ccast.visualizer.WorldWindAppFrame;
import com.bbn.ccast.visualizer.tracks.SDOTrack;
import com.bbn.ccast.visualizer.tracks.TrackController;
import com.bbn.ccast.visualizer.util.SDOListPanel;

import javax.swing.*;
import java.awt.event.ActionEvent;

public class DeleteTrackAction extends AbstractAction {

	private static final long serialVersionUID = 1L;

	protected final SDOListPanel sdoListPanel;

	public DeleteTrackAction(final SDOListPanel sdoListPanel) {
		this.sdoListPanel = sdoListPanel;

		int numSelectedPositions = sdoListPanel.getTable().getSelectedRowCount();
		if (numSelectedPositions <= 1)
			putValue(NAME, "Delete Selected Track");
		else
			putValue(NAME, "Delete Selected Tracks");

		putValue(LONG_DESCRIPTION, "Delete Selected Tracks");

		if (numSelectedPositions == 0)
			this.setEnabled(false);
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		int[] rows = sdoListPanel.getTable().getSelectedRows();
		if (rows.length > 0) {
			for (int index : rows) {
				SDOTrack track = sdoListPanel.getTableModel().getRows().get(index);
				WorldWindAppFrame.removeTrackFromMap(track);
				track.firePropertyChange(TrackController.TRACK_REMOVE, null, track);
			}
			sdoListPanel.getTableModel().removeTracks(rows);
		}	
	}
}
