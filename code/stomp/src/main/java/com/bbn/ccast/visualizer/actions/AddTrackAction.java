//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.actions;

import com.bbn.ccast.visualizer.tracks.SDOTrack;
import com.bbn.ccast.visualizer.util.SDOListPanel;

import javax.swing.*;
import java.awt.event.ActionEvent;

public class AddTrackAction extends AbstractAction {

	private static final long serialVersionUID = 1L;

	protected final SDOListPanel sdoListPanel;

	public AddTrackAction(final SDOListPanel sdoListPanel) {
		this.sdoListPanel = sdoListPanel;
		
		putValue(NAME, "Add New Track");
		putValue(LONG_DESCRIPTION, "Add a new Track");
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		Object inputValue = JOptionPane.showInputDialog(sdoListPanel.getTopLevelAncestor(), "Enter a new track name", "Add New Track",
				JOptionPane.QUESTION_MESSAGE, null, null, null);
		if (inputValue == null)
			return;

		String name = inputValue.toString();

		SDOTrack st = new SDOTrack(name);
		sdoListPanel.getTrackController().addTrack(st);
/*		
		sdoListPanel.addTrack(st);
		st.markDirty();
		
		int cnt = sdoListPanel.getTable().getRowCount();
		sdoListPanel.getTable().getSelectionModel().setSelectionInterval(cnt-1, cnt-1);
*/		
	}
}
