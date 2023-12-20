//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import javax.swing.*;
import java.awt.*;
import java.util.*;

public class SortedListModel extends AbstractListModel<Object> {

	private static final long serialVersionUID = 1L;
	SortedSet<Object> model;

	public SortedListModel() {
		model = new TreeSet<Object>();
	}

	@Override
	public int getSize() {
		return model.size();
	}

	@Override
	public Object getElementAt(int index) {
		return model.toArray()[index];
	}

	public void add(Object element) {
		if (model.add(element)) {
			fireContentsChanged(this, 0, getSize());
		}
	}

	public void addAll(Object elements[]) {
		Collection<Object> c = Arrays.asList(elements);
		model.addAll(c);
		fireContentsChanged(this, 0, getSize());
	}

	public void clear() {
		model.clear();
		fireContentsChanged(this, 0, getSize());
	}

	public boolean contains(Object element) {
		return model.contains(element);
	}

	public Object firstElement() {
		return model.first();
	}

	public Iterator<Object> iterator() {
		return model.iterator();
	}

	public Object lastElement() {
		return model.last();
	}

	public boolean removeElement(Object element) {
		boolean removed = model.remove(element);
		if (removed) {
			fireContentsChanged(this, 0, getSize());
		}
		return removed;
	}
	
	public static void main(String args[]) {
		JFrame f = new JFrame("Dual List Box Tester");
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		DualListBox dual = new DualListBox();
		dual.addSourceElements(new String[] { "One", "Two", "Three" });
		dual.addSourceElements(new String[] { "Four", "Five", "Six" });
		dual.addSourceElements(new String[] { "Seven", "Eight", "Nine" });
		dual.addSourceElements(new String[] { "Ten", "Eleven", "Twelve" });
		dual.addSourceElements(new String[] { "Thirteen", "Fourteen", "Fifteen" });
		dual.addSourceElements(new String[] { "Sixteen", "Seventeen", "Eighteen" });
		dual.addSourceElements(new String[] { "Nineteen", "Twenty", "Thirty" });
		f.getContentPane().add(dual, BorderLayout.CENTER);
		f.setSize(400, 300);
		f.setVisible(true);
	}

}