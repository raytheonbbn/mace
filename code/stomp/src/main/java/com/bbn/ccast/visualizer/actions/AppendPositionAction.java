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
 * @version $Id: AppendPositionAction.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class AppendPositionAction extends AbstractAction {

	private static final long serialVersionUID = 1L;
	protected final PositionTable table;

	public AppendPositionAction(final PositionTable table) {
		this.table = table;
		putValue(NAME, "Append New Position to Track");
		putValue(LONG_DESCRIPTION, "Add a new position to the end of the Track");
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		SDOTrack st = table.getSDOTrack();
		if (st == null)
			return;

		if (st.size() != 0) {	
			SDOPosition aboveSDOP = st.get(st.size() - 1);
			SDOPosition newLookAt = new SDOPosition(PositionType.GOTO, aboveSDOP.getLatitudeDegrees(), aboveSDOP.getLongitudeDegrees(), aboveSDOP.getElevation(), aboveSDOP.getDuration(), 0.0);
			st.appendPosition(newLookAt);
		}
		else
			st.appendPosition(new SDOPosition(PositionType.GOTO));

		table.getSelectionModel().setSelectionInterval(st.size() - 1, st.size() - 1);
	}
}
