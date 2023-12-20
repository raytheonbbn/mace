//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import javax.swing.table.AbstractTableModel;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class MapTableModel<K,V> extends AbstractTableModel {

	private static final long serialVersionUID = 1L;
	
    private final String[] columnNames;
    Map<K, V[]> rows = new HashMap<>();

    public MapTableModel(String[] columnNames) {
        this.columnNames = columnNames;
    }

    @Override
    public int getRowCount() {
        return rows.size();
    }

    @Override
    public int getColumnCount() {
        return columnNames.length;
    }

    @Override
    public Object getValueAt(int rowIndex, int columnIndex) {

    	Set<K> keySet = rows.keySet();
        
    	// We know this is okay, since we know this is made from a Set<K>
		@SuppressWarnings("unchecked")
		K[] array = (K[])keySet.toArray();
		
		K key = array[rowIndex];

		return rows.get(key)[columnIndex];
    }

    public void addOrUpdate(K key, V[] values) {
        rows.put(key, values);
        fireTableDataChanged();
    }

    @Override
    public String getColumnName(int column) {
        return columnNames[column];
    }

    public void clear() {
        rows.clear();
        fireTableDataChanged();
    }
}
