//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */
package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.visualizer.actions.*;

import javax.swing.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;

/**
 * @author tag
 * @version $Id: PositionsContextMenu.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class PositionsContextMenu extends MouseAdapter {
	private final PositionTable positionTable;

	public PositionsContextMenu(final PositionTable positionTable) {
		this.positionTable = positionTable;
	}

	@Override
	public void mousePressed(MouseEvent mouseEvent) {
		this.checkPopup(mouseEvent);
	}

	@Override
	public void mouseClicked(MouseEvent mouseEvent) {
		this.checkPopup(mouseEvent);
	}

	@Override
	public void mouseReleased(MouseEvent mouseEvent) {
		this.checkPopup(mouseEvent);
	}

	private void checkPopup(MouseEvent e) {
		if (!e.isPopupTrigger())
			return;

		JMenuItem mi;
		JPopupMenu pum = new JPopupMenu();

		mi = new JMenuItem(new DeletePositionsAction(positionTable));
		pum.add(mi);

		pum.addSeparator();

		mi = new JMenuItem(new AppendPositionAction(positionTable));
		pum.add(mi);

		mi = new JMenuItem(new InsertPositionAction(true, positionTable));
		pum.add(mi);

		mi = new JMenuItem(new InsertPositionAction(false, positionTable));
		pum.add(mi);

		pum.addSeparator();

		mi = new JMenuItem(new AppendLookAtAction(positionTable));
		pum.add(mi);
		
		mi = new JMenuItem(new InsertLookAtAction(true, positionTable));
		pum.add(mi);

		mi = new JMenuItem(new InsertLookAtAction(false, positionTable));
		pum.add(mi);

		pum.show(positionTable, e.getX(), e.getY());
	}
}
