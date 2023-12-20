//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.visualizer.actions.AddTrackAction;
import com.bbn.ccast.visualizer.actions.DeleteTrackAction;

import javax.swing.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;

public class TracksContextMenu extends MouseAdapter {
	private final SDOListPanel sdoListPanel;

	public TracksContextMenu(final SDOListPanel sdoListPanel) {
		this.sdoListPanel = sdoListPanel;
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

		mi = new JMenuItem(new DeleteTrackAction(sdoListPanel));
		pum.add(mi);

		pum.addSeparator();

		mi = new JMenuItem(new AddTrackAction(sdoListPanel));
		pum.add(mi);

		pum.show(sdoListPanel, e.getX(), e.getY());
	}
}
