//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */
package com.bbn.ccast.visualizer.actions;

import com.bbn.ccast.visualizer.tracks.SDOTrack;
import com.bbn.ccast.visualizer.util.PositionTable;

import javax.swing.*;
import java.awt.event.ActionEvent;

/**
 * @author tag
 * @version $Id: DeletePositionsAction.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class DeletePositionsAction extends AbstractAction {

	private static final long serialVersionUID = 1L;

	protected final PositionTable table;

	public DeletePositionsAction(final PositionTable table) {
		this.table = table;

		int numSelectedPositions = table.getSelectedRowCount();
		if (numSelectedPositions <= 1)
			putValue(NAME, "Delete Selected Position");
		else
			putValue(NAME, "Delete Selected Positions");

		putValue(LONG_DESCRIPTION, "Remove Positions from Track");

		if (numSelectedPositions == 0)
			this.setEnabled(false);
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		SDOTrack st = table.getSDOTrack();
		if (st != null)
			st.removePositions(this.table.getSelectedRows());
	}
}
