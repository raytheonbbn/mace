//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import javax.swing.*;
import java.awt.*;

public class RadioButtonPanel extends JPanel {

	private static final long serialVersionUID = 1L;

	private enum Orientation {
		_VERTICAL, _HORIZONTAL
	}

	public static final Orientation VERTICAL = Orientation._VERTICAL;
	public static final Orientation HORIZONTAL = Orientation._HORIZONTAL;

	private ButtonGroup _buttonGroup = new ButtonGroup();

	/**
	 * This more general constructor allows specification of the orientation, either
	 * horizontal or vertical.
	 * 
	 * @param orientation
	 *            RadioButtonPanel.Orientation.VERTICAL or
	 *            RadioButtonPanel.Orientation.HORIZONTAL (I could have used a
	 *            "magic value", like a true/false, 0/1, ... but the extra typing
	 *            may be worth it because the meaning will be clear.)
	 *
	 * @param title
	 *            Text to use in the titled border. No border if this is null.
	 * @param buttons
	 *            The radio buttons.
	 */
	public RadioButtonPanel(Orientation orientation, String title, JRadioButton... buttons) {
		// ... Create a button group.

		// ... Select layout.
		if (orientation == VERTICAL) {
			this.setLayout(new GridLayout(buttons.length, 1));
		} else {
			this.setLayout(new FlowLayout(FlowLayout.LEADING));
		}

		// ... Add buttons to the button group and the layout.
		for (JRadioButton button : buttons) {
			_buttonGroup.add(button);
			this.add(button);
		}

		// ... Add a border if required.
		if (title != null) {
			this.setBorder(BorderFactory.createTitledBorder(BorderFactory.createEtchedBorder(), title));
		}
	}

	/**
	 * This most common constructor creates a vertical layout of the radio buttons
	 * with an etched titled border (if title is non-null).
	 * 
	 * @param title
	 *            Text to use in the titled border. No border if this is null.
	 * @param buttons
	 *            The radio buttons.
	 */
	public RadioButtonPanel(String title, JRadioButton... buttons) {
		this(VERTICAL, title, buttons);
	}

	/** Makes all radio buttons in the group unselected. */
	public void clearSelection() {
		_buttonGroup.clearSelection();
	}
	
	public void setSelected(ButtonModel model, boolean isSelected) {
		_buttonGroup.setSelected(model, isSelected);
	}
}
