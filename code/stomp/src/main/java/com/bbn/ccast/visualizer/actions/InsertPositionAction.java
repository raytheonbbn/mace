//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */
package com.bbn.ccast.visualizer.actions;

import com.bbn.ccast.visualizer.tracks.PositionType;
import com.bbn.ccast.visualizer.tracks.SDOPosition;
import com.bbn.ccast.visualizer.tracks.SDOTrack;
import com.bbn.ccast.visualizer.util.PositionTable;

import javax.swing.*;
import java.awt.event.ActionEvent;

/**
 * @author tag
 * @version $Id: InsertPositionAction.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class InsertPositionAction extends AbstractAction {

	private static final long serialVersionUID = 1L;

	private final boolean above;
	protected final PositionTable table;

	public InsertPositionAction(final boolean above, final PositionTable table) {
		this.table = table;
		this.above = above;
		if (this.above) {
			putValue(NAME, "Insert New Position Above Selection");
			putValue(LONG_DESCRIPTION, "Insert a new position above the selected positions");
		} else {
			putValue(NAME, "Insert New Position Below Selection");
			putValue(LONG_DESCRIPTION, "Insert a new position below the selected positions");
		}

		if (table.getSelectedRowCount() == 0)
			this.setEnabled(false);
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		SDOTrack st = table.getSDOTrack();
		if (st == null)
			return;

		int index = table.getSelectionModel().getMinSelectionIndex();
		if (!this.above)
			index = table.getSelectionModel().getMaxSelectionIndex()+1;

		if (index < 0)
			return;

		st.insertPosition(index, new SDOPosition(PositionType.GOTO));

		table.getSelectionModel().setSelectionInterval(index, index);
	}
}
