//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/**
 * 
 */
package com.bbn.ccast.visualizer.util;

import gov.nasa.worldwind.symbology.milstd2525.MilStd2525TacticalSymbol;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * @author ddiller
 *
 */
public class CheckBoxGroup extends JPanel {

	private static final long serialVersionUID = 1L;
	public static final String delimeter = ";";
	
	private JCheckBox all;
	protected List<JCheckBox> checkBoxes;
	private JButton selectedFromMapButton;

	private String lastCheckboxSelected;
	

	public CheckBoxGroup(CcastScreenSelector screenSelector, boolean includeSelectedFromMap, String typeName,
			String... options) {
		this.checkBoxes = new ArrayList<>(25);
		setLayout(new BorderLayout());
		JPanel header = new JPanel(new FlowLayout(FlowLayout.LEFT, 1, 1));
		all = new JCheckBox("Select All...");
		all.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				for (JCheckBox cb : checkBoxes) {
					cb.setSelected(all.isSelected());
				}
			}
		});
		

		header.add(all);
		add(header, BorderLayout.NORTH);

		JPanel content = new ScrollablePane(new GridBagLayout());
		content.setBackground(UIManager.getColor("List.background"));
		if (options.length > 0) {

			GridBagConstraints gbc = new GridBagConstraints();
			gbc.gridwidth = GridBagConstraints.REMAINDER;
			gbc.anchor = GridBagConstraints.NORTHWEST;
			gbc.weightx = 1;
			// Why are the checkboxes created twice? This bit seems to populate all but the last checkbox
			 for (int index = 0; index < options.length - 1; index++) {
			 	JCheckBox cb = new JCheckBox(options[index]);
			 	cb.setOpaque(false);
			 	checkBoxes.add(cb);
			 	content.add(cb, gbc);
			 	cb.addActionListener(new ActionListener() {
					 @Override
					 public void actionPerformed(ActionEvent e) {
					 	setLastSelected(cb);
					 }
			 	});
			 }

		 	// This bit creates just the last one
			JCheckBox cb = new JCheckBox(options[options.length - 1]);
			cb.setOpaque(false);
			checkBoxes.add(cb);
			gbc.weighty = 1;
			content.add(cb, gbc);
			cb.addActionListener(new ActionListener() {
				@Override
				public void actionPerformed(ActionEvent e) {
					setLastSelected(cb);
				}
			});

		}

		add(new JScrollPane(content));

		// if (includeSelectedFromMap) {
		// 	JPanel footer = new JPanel(new FlowLayout(FlowLayout.CENTER, 1, 1));
		// 	selectedFromMapButton = new JButton("Add Selected " + typeName);

		// 	selectedFromMapButton.addActionListener(new ActionListener() {
		// 		@Override
		// 		public void actionPerformed(ActionEvent ev) {
		// 			List<?> selectedObjects = screenSelector.getSelectedObjects();
		// 			for (Object selectedObj : selectedObjects) {
		// 				if (selectedObj instanceof MilStd2525TacticalSymbol) {
		// 					MilStd2525TacticalSymbol symbol = (MilStd2525TacticalSymbol) selectedObj;
		// 					Object id = symbol.getModifier("T");
		// 					for (JCheckBox checkbox : checkBoxes) {
		// 						if (getDelimitedString(checkbox.getText()).equalsIgnoreCase(id.toString())) {
		// 							checkbox.setSelected(true);
		// 						}
		// 					}
		// 				}
		// 			}
		// 		}
		// 	});

//			footer.add(selectedFromMapButton);
//			add(footer, BorderLayout.SOUTH);
//		}
	}

	public void setSelected(String[] items) {
		List<String> list = new ArrayList<String>();
		if (items != null) {
			list = Arrays.asList(items);
		}
		for (JCheckBox cb : checkBoxes) {
			if (list.contains(getDelimitedString(cb.getText()))) {
				cb.setSelected(true);
			} else {
				cb.setSelected(false);
			}
		}
	}
	
	public String[] getSelected() {
		List<String> selectedList = new ArrayList<String>();
		for (JCheckBox cb : checkBoxes) {
			if (cb.isSelected()) {
				selectedList.add(getDelimitedString(cb.getText()));
			}
		}
		String[] strArray = new String[selectedList.size()];
		strArray = selectedList.toArray(new String[0]);
		return strArray;
	}

	public void setLastSelected(JCheckBox cb) {
		if (cb.isSelected()) {
			this.lastCheckboxSelected = getDelimitedString(cb.getText());
		}
		System.out.println("Set last selected checkbox to " + this.lastCheckboxSelected);
	}

	public String getLastSelected() {
		return this.lastCheckboxSelected;
	}

	public class ScrollablePane extends JPanel implements Scrollable {

		private static final long serialVersionUID = 1L;

		public ScrollablePane(LayoutManager layout) {
			super(layout);
		}

		public ScrollablePane() {
		}

		@Override
		public Dimension getPreferredScrollableViewportSize() {
			return new Dimension(200, 200);
		}

		@Override
		public int getScrollableUnitIncrement(Rectangle visibleRect, int orientation, int direction) {
			return 32;
		}

		@Override
		public int getScrollableBlockIncrement(Rectangle visibleRect, int orientation, int direction) {
			return 32;
		}

		@Override
		public boolean getScrollableTracksViewportWidth() {
			boolean track = false;
			Container parent = getParent();
			if (parent instanceof JViewport) {
				JViewport vp = (JViewport) parent;
				track = vp.getWidth() > getPreferredSize().width;
			}
			return track;
		}

		@Override
		public boolean getScrollableTracksViewportHeight() {
			boolean track = false;
			Container parent = getParent();
			if (parent instanceof JViewport) {
				JViewport vp = (JViewport) parent;
				track = vp.getHeight() > getPreferredSize().height;
			}
			return track;
		}

	}
	
	public static String getDelimitedString(String str) {
		int end = str.indexOf(delimeter);
		if (end == -1) {
			return str;
		}
		return str.substring(0, end);
	}
	
}