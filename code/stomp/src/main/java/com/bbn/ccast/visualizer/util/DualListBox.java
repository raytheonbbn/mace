//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
Definitive Guide to Swing for Java 2, Second Edition
By John Zukowski     
ISBN: 1-893115-78-X
Publisher: APress
*/
package com.bbn.ccast.visualizer.util;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Iterator;

public class DualListBox extends JPanel {

	private static final long serialVersionUID = 1L;

	private static final Insets EMPTY_INSETS = new Insets(0, 0, 0, 0);
	private static final String ADD_BUTTON_LABEL = "Add >>";
	private static final String REMOVE_BUTTON_LABEL = "<< Remove";
	private static final String DEFAULT_SOURCE_CHOICE_LABEL = "Available Choices";
	private String sourceChoiceLabelString;
	private String destChoiceLabelString;
	private static final String DEFAULT_DEST_CHOICE_LABEL = "Your Choices";
	private JLabel sourceLabel;
	private JList<?> sourceList;
	private SortedListModel sourceListModel;
	private JList<Object> destList;
	private UnSortedListModel destListModel;
	private JLabel destLabel;
	private JButton addButton;
	private JButton removeButton;
	
	public DualListBox() {
		this (DEFAULT_SOURCE_CHOICE_LABEL, DEFAULT_DEST_CHOICE_LABEL);
	}
	
	public DualListBox(String sourceChoiceLabelString, String destChoiceLabelString) {
		this.sourceChoiceLabelString = sourceChoiceLabelString;
		this.destChoiceLabelString = destChoiceLabelString;
		initScreen();
	}

	public UnSortedListModel getDestListModel() {
		return destListModel;
	}
	
	public String getSourceChoicesTitle() {
		return sourceLabel.getText();
	}

	public void setSourceChoicesTitle(String newValue) {
		sourceLabel.setText(newValue);
	}

	public String getDestinationChoicesTitle() {
		return destLabel.getText();
	}

	public void setDestinationChoicesTitle(String newValue) {
		destLabel.setText(newValue);
	}

	public void clearSourceListModel() {
		sourceListModel.clear();
	}

	public void clearDestinationListModel() {
		destListModel.clear();
	}

	public void addSourceElements(ListModel<?> newValue) {
		fillSortedListModel(sourceListModel, newValue);
	}

	public void setSourceElements(ListModel<?> newValue) {
		clearSourceListModel();
		addSourceElements(newValue);
	}

	private void fillSortedListModel(SortedListModel model, ListModel<?> newValues) {
		int size = newValues.getSize();
		for (int i = 0; i < size; i++) {
			model.add(newValues.getElementAt(i));
		}
	}

	private void fillDefaultListModel(UnSortedListModel model, Object[] newValue) {
		int size = newValue.length;
		for (int i = 0; i < size; i++) {
			model.add(newValue[i]);
		}
	}

	public void addSourceElements(Object newValue[]) {
		fillListModel(sourceListModel, newValue);
	}

	public void setSourceElements(Object newValue[]) {
		clearSourceListModel();
		addSourceElements(newValue);
	}

	public void addDestinationElements(Object newValue[]) {
		fillDefaultListModel(destListModel, newValue);
	}

	private void fillListModel(SortedListModel model, Object newValues[]) {
		model.addAll(newValues);
	}

	public Iterator<?> sourceIterator() {
		return sourceListModel.iterator();
	}

	public Iterator<?> destinationIterator() {
		return destListModel.iterator();
	}

	public ListCellRenderer<?> getSourceCellRenderer() {
		return sourceList.getCellRenderer();
	}

	public ListCellRenderer<?> getDestinationCellRenderer() {
		return destList.getCellRenderer();
	}

	public void setVisibleRowCount(int newValue) {
		sourceList.setVisibleRowCount(newValue);
		destList.setVisibleRowCount(newValue);
	}

	public int getVisibleRowCount() {
		return sourceList.getVisibleRowCount();
	}

	public void setSelectionBackground(Color newValue) {
		sourceList.setSelectionBackground(newValue);
		destList.setSelectionBackground(newValue);
	}

	public Color getSelectionBackground() {
		return sourceList.getSelectionBackground();
	}

	public void setSelectionForeground(Color newValue) {
		sourceList.setSelectionForeground(newValue);
		destList.setSelectionForeground(newValue);
	}

	public Color getSelectionForeground() {
		return sourceList.getSelectionForeground();
	}

	public boolean moveItemFromSourceToDest(Object item) {
		boolean res = sourceListModel.removeElement(item);
		if (res) {
			destListModel.add(item);
		}
		return res;
	}
	
	private void clearSourceSelected() {
		Object selected[] = sourceList.getSelectedValuesList().toArray();
		for (int i = selected.length - 1; i >= 0; --i) {
			sourceListModel.removeElement(selected[i]);
		}
		sourceList.getSelectionModel().clearSelection();
	}

	private void clearDestinationSelected() {
		Object selected[] = destList.getSelectedValuesList().toArray();
		for (int i = selected.length - 1; i >= 0; --i) {
			destListModel.removeElement(selected[i]);
		}
		destList.getSelectionModel().clearSelection();
	}

	private void initScreen() {
		setBorder(BorderFactory.createEtchedBorder());
		setLayout(new GridBagLayout());
		sourceLabel = new JLabel(this.sourceChoiceLabelString);
		sourceListModel = new SortedListModel();
		sourceList = new JList<Object>(sourceListModel);
		add(sourceLabel, new GridBagConstraints(0, 0, 1, 1, 0, 0, GridBagConstraints.CENTER, GridBagConstraints.NONE,
				EMPTY_INSETS, 0, 0));
		add(new JScrollPane(sourceList), new GridBagConstraints(0, 1, 1, 5, .5, 1, GridBagConstraints.CENTER,
				GridBagConstraints.BOTH, EMPTY_INSETS, 0, 0));

		addButton = new JButton(ADD_BUTTON_LABEL);
		add(addButton, new GridBagConstraints(1, 2, 1, 2, 0, .25, GridBagConstraints.CENTER, GridBagConstraints.NONE,
				EMPTY_INSETS, 0, 0));
		addButton.addActionListener(new AddListener());
		removeButton = new JButton(REMOVE_BUTTON_LABEL);
		add(removeButton, new GridBagConstraints(1, 4, 1, 2, 0, .25, GridBagConstraints.CENTER, GridBagConstraints.NONE,
				new Insets(0, 5, 0, 5), 0, 0));
		removeButton.addActionListener(new RemoveListener());

		destLabel = new JLabel(this.destChoiceLabelString);
		destListModel = new UnSortedListModel();
		destList = new JList<Object>(destListModel);
		add(destLabel, new GridBagConstraints(2, 0, 1, 1, 0, 0, GridBagConstraints.CENTER, GridBagConstraints.NONE,
				EMPTY_INSETS, 0, 0));
		add(new JScrollPane(destList), new GridBagConstraints(2, 1, 1, 5, .5, 1.0, GridBagConstraints.CENTER,
				GridBagConstraints.BOTH, EMPTY_INSETS, 0, 0));
	}

	private class AddListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			Object selected[] = sourceList.getSelectedValuesList().toArray();
			addDestinationElements(selected);
			clearSourceSelected();
		}
	}

	private class RemoveListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			Object selected[] = destList.getSelectedValuesList().toArray();
			addSourceElements(selected);
			clearDestinationSelected();
		}
	}
}
