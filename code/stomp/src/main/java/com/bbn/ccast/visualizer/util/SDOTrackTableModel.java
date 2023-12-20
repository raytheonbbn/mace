//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.visualizer.tracks.SDOTrack;
import org.apache.log4j.Logger;

import javax.swing.event.TableModelEvent;
import javax.swing.event.TableModelListener;
import javax.swing.table.AbstractTableModel;
import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SDOTrackTableModel extends AbstractTableModel implements TableModelListener {

	private static final Logger logger = Logger.getLogger(SDOTrackTableModel.class.getName());

	public final static String NAME = "Name";
	public final static String COLOR = "Color";
	private final String[] columnNames = { NAME, COLOR };

	private static final long serialVersionUID = 1L;

	List<SDOTrack> rows = new ArrayList<SDOTrack>();

	public SDOTrackTableModel() {
		this.addTableModelListener(this);
	}

	/**
	 * @return the rows
	 */
	public List<SDOTrack> getRows() {
		return rows;
	}

	/**
	 * @param rows the rows to set
	 */
	public void setRows(List<SDOTrack> rows) {
		this.rows = rows;
	}

	@Override
	public int getRowCount() {
		return rows.size();
	}


	@Override
	public int getColumnCount() {
		return columnNames.length;
	}

	Color getColor(int rowIndex) {
		if (rowIndex == -1) {
			return null;
		}
		
		return rows.get(rowIndex).getColor();
	}
	
	String geName(int rowIndex) {
		if (rowIndex == -1) {
			return null;
		}
		
		return rows.get(rowIndex).getName();
	}
	
	@Override
    public Object getValueAt(int rowIndex, int columnIndex) {
		if (rowIndex == -1) {
			return null;
		}
		
		SDOTrack msg = rows.get(rowIndex);
		Object returnValue = null;
		
		if (columnIndex == 0) {
			returnValue = msg.getName();
		} else if (columnIndex == 1) {
			returnValue = null; //msg.getColor();
		} else {
			throw new IllegalArgumentException("Invalid column index");
		}
		
		return returnValue;
    }

	public void addOrUpdate(SDOTrack msg, boolean fireChange) {
		rows.add(msg);
		if (fireChange) {
			fireTableDataChanged();
		}
	}

	@Override
	public String getColumnName(int column) {
		return columnNames[column];
	}

	public void clear() {
		rows.clear();
		fireTableDataChanged();
	}

	@Override
	public void tableChanged(TableModelEvent e) {
		//fireTableDataChanged();		
	}
	
	public void removeTracks(int[] trackIndices) {
		Arrays.sort(trackIndices);
		for (int i = trackIndices.length - 1; i >= 0; i--) {
			if (trackIndices[i] < 0 || trackIndices[i] >= this.rows.size())
				continue;

			this.rows.remove(trackIndices[i]);
		}

		fireTableDataChanged();
	}
}
