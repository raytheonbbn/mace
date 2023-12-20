//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.visualizer.WorldWindVisualization;
import com.bbn.ccast.visualizer.tracks.SDOTrack;
import com.bbn.ccast.visualizer.tracks.TrackController;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.render.PatternFactory;

import javax.swing.*;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;
import javax.swing.table.DefaultTableCellRenderer;
import javax.swing.table.JTableHeader;
import javax.swing.table.TableColumn;
import javax.swing.table.TableColumnModel;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.util.List;

public class SDOListPanel extends JPanel {

	private static final long serialVersionUID = 1L;

	@SuppressWarnings("unused")
	private static final int NAME_COLUMN = 0;
	private static final int COLOR_COLUMN = 1;

	private String elevationUnit = WorldWindVisualization.UNIT_METRIC;
	private String angleFormat = Angle.ANGLE_FORMAT_DD;

	private JTable table;
	private SDOTrackTableModel tableModel;
	private JScrollPane scrollPane;

	protected int sortCol = 0;
	protected boolean isSortAsc = true;
	
	private TrackController trackController;

	public SDOListPanel(SDOPanel sdoPanel) {
		super(new BorderLayout());
		initComponents();

		this.scrollPane.addMouseListener(new TracksContextMenu(this));
		this.table.addMouseListener(new TracksContextMenu(this));

		this.table.getSelectionModel().addListSelectionListener(new ListSelectionListener() {
			@Override
			public void valueChanged(ListSelectionEvent e) {
				SDOTrack track = getCurrentTrack();
				if (track == null) {
					sdoPanel.setTrack(null);
					return;
				}

				sdoPanel.setTrack(track);
			}
		});

		this.table.getColumnModel().getColumn(COLOR_COLUMN).setCellRenderer(new ColorColumnCellRenderer());
		
	}

	public void setTrackController(TrackController trackController) {
		this.trackController = trackController;
	}
	
	public TrackController getTrackController() {
		return this.trackController;
	}
	
	private void initComponents() {
		tableModel = new SDOTrackTableModel();
		table = new JTable(tableModel);
		JTableHeader header = table.getTableHeader();
		header.addMouseListener(new ColumnListener(table));

		scrollPane = new JScrollPane(table);
		this.add(scrollPane, BorderLayout.CENTER);
		table.setShowGrid(true);
	}

	public SDOTrack getCurrentTrack() {
		int selectedRow = this.table.getSelectedRow();
		if (selectedRow >= 0) {
			return this.tableModel.getRows().get(selectedRow);
		}
		return null;
	}

	public void setCurrentTrack(SDOTrack track) {
		int index = this.getTrackPanelIndex(track);
		if (index < 0)
			return;
		this.table.setRowSelectionInterval(index, index);
	}

	public List<SDOTrack> getAllTracks() {
		return this.tableModel.getRows();
	}

	public void addTrack(SDOTrack track) {
		tableModel.addOrUpdate(track, true);
		this.setCurrentTrack(track);
	}

	/**
	 * @return the table
	 */
	public JTable getTable() {
		return table;
	}

	/**
	 * @param table
	 *            the table to set
	 */
	public void setTable(JTable table) {
		this.table = table;
	}

	/**
	 * @return the tableModel
	 */
	public SDOTrackTableModel getTableModel() {
		return tableModel;
	}

	/**
	 * @param tableModel
	 *            the tableModel to set
	 */
	public void setTableModel(SDOTrackTableModel tableModel) {
		this.tableModel = tableModel;
	}

	public String getElevationUnit() {
		return this.elevationUnit;
	}

	public String getAngleFormat() {
		return this.angleFormat;
	}

	private int getTrackPanelIndex(SDOTrack track) {
		int i = 0;
		for (SDOTrack row : this.tableModel.getRows()) {
			if (row == track) {
				return i;
			}
			i++;
		}
		return -1;

	}

	private static Icon makeColorCircle(Color color) {
		BufferedImage bi = PatternFactory.createPattern(PatternFactory.PATTERN_CIRCLE, new Dimension(16, 16), .9f,
				color);

		return new ImageIcon(bi);
	}

	public class ColorColumnCellRenderer extends DefaultTableCellRenderer {
		private static final long serialVersionUID = 1L;

		@Override
		  public Component getTableCellRendererComponent(JTable tbl, Object value, boolean isSelected, boolean hasFocus, int row, int col) {

			  //Cells are by default rendered as a JLabel.
			  JLabel l = (JLabel) super.getTableCellRendererComponent(tbl, value, isSelected, hasFocus, row, col);
			  l.setIcon(makeColorCircle(tableModel.getColor(row)));
			  //l.setBackground(tableModel.getColor(row));
			  return l;
		}
	}
		
	
	class ColumnListener extends MouseAdapter {
		protected JTable commsTable;

		public ColumnListener(JTable t) {
			commsTable = t;
		}

		@Override
		public void mouseClicked(MouseEvent e) {
			TableColumnModel colModel = commsTable.getColumnModel();
			int columnModelIndex = colModel.getColumnIndexAtX(e.getX());
			int modelIndex = colModel.getColumn(columnModelIndex).getModelIndex();

			if (modelIndex < 0)
				return;
			if (sortCol == modelIndex)
				isSortAsc = !isSortAsc;
			else
				sortCol = modelIndex;

			for (int i = 0; i < commsTable.getColumnCount(); i++) {
				TableColumn column = colModel.getColumn(i);
				column.setHeaderValue(commsTable.getColumnName(column.getModelIndex()));
			}
			commsTable.getTableHeader().repaint();

		}
	}
	
	
}
