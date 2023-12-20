//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import javax.accessibility.Accessible;
import javax.swing.*;
import javax.swing.plaf.basic.BasicComboPopup;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.InputEvent;
import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class CheckedComboBox<E extends CheckableItem> extends JComboBox<E> {
	private static final long serialVersionUID = 1L;
	private boolean keepOpen;
	private transient ActionListener listener;

	protected CheckedComboBox() {
		super();
	}

	public CheckedComboBox(ComboBoxModel<E> aModel) {
		super(aModel);
	}

	public CheckedComboBox(E[] m) {
		super(m);
	}

	@Override
	public Dimension getPreferredSize() {
		return new Dimension(200, 20);
	}

	@Override
	public void updateUI() {
		setRenderer(null);
		removeActionListener(listener);
		super.updateUI();
		listener = e -> {
			if ((e.getModifiers() & InputEvent.MOUSE_EVENT_MASK) != 0) {
				updateItem(getSelectedIndex());
				keepOpen = true;
			}
		};
		setRenderer(new CheckBoxCellRenderer<CheckableItem>());
		addActionListener(listener);
		getInputMap(JComponent.WHEN_FOCUSED).put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "checkbox-select");
		getActionMap().put("checkbox-select", new AbstractAction() {
			private static final long serialVersionUID = 1L;

			@Override
			public void actionPerformed(ActionEvent e) {
				Accessible a = getAccessibleContext().getAccessibleChild(0);
				if (a instanceof BasicComboPopup) {
					BasicComboPopup pop = (BasicComboPopup) a;
					updateItem(pop.getList().getSelectedIndex());
				}
			}
		});
	}

	private void updateItem(int index) {
		if (isPopupVisible()) {
			E item = getItemAt(index);
			item.selected ^= true;
			removeItemAt(index);
			insertItemAt(item, index);
			setSelectedItem(item);
		}
	}

	@Override
	public void setPopupVisible(boolean v) {
		if (keepOpen) {
			keepOpen = false;
		} else {
			super.setPopupVisible(v);
		}
	}

	@Override
	public Object[] getSelectedObjects() {
		ArrayList<Object> objects = new ArrayList<Object>();
		for (int index = 0; index < this.getModel().getSize(); index++) {
			E item = this.getItemAt(index);
			if (item.isSelected()) {
				objects.add(item.text);
			}
		}
		return objects.toArray(new Object[0]);
	}
	
	
	public void setSelectedObjects(Object[] array) {
		for (Object e : array) {
			for (int index = 0; index < this.getModel().getSize(); index++) {
				E item = this.getItemAt(index);
				if (item.text.equals(e)) {
					item.setSelected(true);
				}
			}
		}
	}
}

class CheckBoxCellRenderer<E extends CheckableItem> implements ListCellRenderer<E> {
	private final JLabel label = new JLabel(" ");
	private final JCheckBox check = new JCheckBox(" ");

	@Override
	public Component getListCellRendererComponent(JList<? extends E> list, E value, int index, boolean isSelected,
			boolean cellHasFocus) {
		if (index < 0) {
			String txt = getCheckedItemString(list.getModel());
			label.setText(txt.isEmpty() ? " " : txt);
			return label;
		}
		check.setText(Objects.toString(value, ""));
		check.setSelected(value.isSelected());
		if (isSelected) {
			check.setBackground(list.getSelectionBackground());
			check.setForeground(list.getSelectionForeground());
		} else {
			check.setBackground(list.getBackground());
			check.setForeground(list.getForeground());
		}
		return check;
	}

	private static <E extends CheckableItem> String getCheckedItemString(ListModel<E> model) {
		return IntStream.range(0, model.getSize()).mapToObj(model::getElementAt).filter(CheckableItem::isSelected)
				.map(Objects::toString).sorted().collect(Collectors.joining(", "));
	}
}
