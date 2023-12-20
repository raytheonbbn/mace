//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */
package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.util.ConversionUtils;
import com.bbn.ccast.visualizer.WorldWindVisualization;
import com.bbn.ccast.visualizer.tracks.PositionType;
import com.bbn.ccast.visualizer.tracks.SDOPosition;
import com.bbn.ccast.visualizer.tracks.SDOTrack;
import gov.nasa.worldwind.geom.Angle;

import javax.swing.*;
import javax.swing.border.LineBorder;
import javax.swing.table.AbstractTableModel;
import javax.swing.table.DefaultTableCellRenderer;
import javax.swing.table.TableCellRenderer;
import javax.swing.table.TableColumnModel;
import java.awt.*;
import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.text.NumberFormat;

/**
 * @author tag
 * @version $Id: PositionTable.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class PositionTable extends JTable {

	private static final long serialVersionUID = 1L;
	
	private static final int ITEM_NUM_COLUMN = 0;
	private static final int ITEM_TYPE_COLUMN = 1;
	private static final int LATITUDE_COLUMN = 2;
	private static final int LONGITUDE_COLUMN = 3;
	private static final int ALTITUDE_COLUMN = 4;
	private static final int DURATION_COLUMN = 5;
	private static final int RADIUS_COLUMN = 6;

	private SDOTrack sdoTrack;
	private String elevationUnit;
	private String angleFormat;

	private final PropertyChangeListener propertyListener = new PropertyChangeListener() {
		@Override
		public void propertyChange(PropertyChangeEvent propertyChangeEvent) {
			Object newValue = propertyChangeEvent.getNewValue();
			if (newValue != null && newValue instanceof Integer) {
				updateTableRow((Integer) newValue);
			} else {
				updateTableData();
			}
		}
	};

	public PositionTable() {
		this.setToolTipText("Track Positions");
		this.setModel(new LLATableModel(this));

		// Force the JTable to commit a cell edit if that cell looses focus.
		this.putClientProperty("terminateEditOnFocusLost", true);

		TableCellRenderer tcr = this.getTableHeader().getDefaultRenderer();

		// Setup latitude and longitude columns
		this.getTableHeader().getColumnModel().getColumn(LATITUDE_COLUMN)
				.setHeaderRenderer(new AngleHeaderRenderer(tcr, this));
		this.getTableHeader().getColumnModel().getColumn(LONGITUDE_COLUMN)
				.setHeaderRenderer(new AngleHeaderRenderer(tcr, this));
		this.getColumnModel().getColumn(LATITUDE_COLUMN).setCellRenderer(new AngleCellRenderer(this));
		this.getColumnModel().getColumn(LONGITUDE_COLUMN).setCellRenderer(new AngleCellRenderer(this));
		this.getColumnModel().getColumn(LATITUDE_COLUMN).setCellEditor(new AngleCellEditor(this, -90, 90));
		this.getColumnModel().getColumn(LONGITUDE_COLUMN).setCellEditor(new AngleCellEditor(this, -180, 180));

		// Setup altitude column
		this.getTableHeader().getColumnModel().getColumn(ALTITUDE_COLUMN)
				.setHeaderRenderer(new AltitudeHeaderRenderer(tcr, this));
		this.getColumnModel().getColumn(ALTITUDE_COLUMN).setCellRenderer(new AltitudeCellRenderer(this));
		this.getColumnModel().getColumn(ALTITUDE_COLUMN).setCellEditor(new AltitudeCellEditor(this));

		// Setup duration column
		this.getTableHeader().getColumnModel().getColumn(DURATION_COLUMN)
				.setHeaderRenderer(new DurationHeaderRenderer(tcr, this));
		this.getColumnModel().getColumn(DURATION_COLUMN).setCellRenderer(new DurationCellRenderer(this));
		this.getColumnModel().getColumn(DURATION_COLUMN).setCellEditor(new DurationCellEditor(this));

		// Setup radius column
		this.getTableHeader().getColumnModel().getColumn(RADIUS_COLUMN)
				.setHeaderRenderer(new RadiusHeaderRenderer(tcr, this));
		this.getColumnModel().getColumn(RADIUS_COLUMN).setCellRenderer(new RadiusCellRenderer(this));
		this.getColumnModel().getColumn(RADIUS_COLUMN).setCellEditor(new RadiusCellEditor(this));
		
		
		{
			TableColumnModel cm = this.getColumnModel();
			cm.getColumn(0).setResizable(false);
			cm.getColumn(0).setMinWidth(25);
			cm.getColumn(0).setPreferredWidth(25);

			cm.getColumn(1).setResizable(false);
			cm.getColumn(1).setMinWidth(45);
			cm.getColumn(1).setPreferredWidth(45);

			cm.getColumn(2).setResizable(false);
			cm.getColumn(2).setMinWidth(70);
			cm.getColumn(2).setPreferredWidth(80);

			cm.getColumn(3).setResizable(false);
			cm.getColumn(3).setMinWidth(70);
			cm.getColumn(3).setPreferredWidth(80);

			cm.getColumn(4).setResizable(false);
			cm.getColumn(4).setMinWidth(50);
			cm.getColumn(4).setPreferredWidth(50);

			cm.getColumn(5).setResizable(false);
			cm.getColumn(5).setMinWidth(50);
			cm.getColumn(5).setPreferredWidth(50);
		
		}
	}

	public SDOTrack getSDOTrack() {
		return sdoTrack;
	}

	public void setSDOTrack(SDOTrack sdoTrack) {
		if (this.sdoTrack == sdoTrack)
			return;

		if (this.sdoTrack != null)
			this.sdoTrack.removePropertyChangeListener(this.propertyListener);

		this.sdoTrack = sdoTrack;

		if (this.sdoTrack != null)
			this.sdoTrack.addPropertyChangeListener(this.propertyListener);

		this.setTableColors(this.sdoTrack);
		this.updateTableData();
	}

	public String getElevationUnit() {
		return this.elevationUnit;
	}

	public void setElevationUnit(String unit) {
		this.elevationUnit = unit;
	}

	public String getAngleFormat() {
		return this.angleFormat;
	}

	public void setAngleFormat(String format) {
		this.angleFormat = format;
	}

	public void updateTableData() {
		((AbstractTableModel) this.getModel()).fireTableDataChanged();
	}

	public void updateTableRow(int row) {
		((AbstractTableModel) this.getModel()).fireTableRowsUpdated(row, row);
	}

	protected Color getTableColorForTrack(SDOTrack track) {
		if (track == null)
			return null;

		Color color = track.getColor();

		float[] hsbComponents = new float[3];
		Color.RGBtoHSB(color.getRed(), color.getGreen(), color.getBlue(), hsbComponents);
		float hue = hsbComponents[0];
		float saturation = hsbComponents[1];
		float brightness = hsbComponents[2];

		saturation *= 0.2f;

		int rgbInt = Color.HSBtoRGB(hue, saturation, brightness);

		return new Color(rgbInt);
	}

	protected void setTableColors(SDOTrack track) {
		Color tableBackground = this.getTableColorForTrack(track);
		Color selectionBg = (tableBackground != null) ? Color.DARK_GRAY : null;
		Color selectionFg = (tableBackground != null) ? Color.WHITE : null;

		this.setBackground(tableBackground);
		this.setSelectionForeground(selectionFg);
		this.setSelectionBackground(selectionBg);
		this.setOpaque(true);

		Container c = this.getParent();
		if (c != null) {
			c.setBackground(tableBackground);
			if (c instanceof JComponent)
				((JComponent) c).setOpaque(true);
		}
	}

	private class LLATableModel extends AbstractTableModel {

		private static final long serialVersionUID = 1L;

		String[] columnNames = new String[] { "#", "Type", "Latitude", "Longitude", "Altitude", "Duration", "Radius" };

		Class<?>[] columnTypes = new Class[] { Integer.class, PositionType.class, String.class, String.class, Double.class, Double.class, Double.class };

		boolean[] columnEditable = new boolean[] { false, false, true, true, true, true, true };

		private PositionTable table;

		public LLATableModel(PositionTable table) {
			this.table = table;
		}

		@Override
		public Class<?> getColumnClass(int columnIndex) {
			return this.columnTypes[columnIndex];
		}

		@Override
		public boolean isCellEditable(int rowIndex, int columnIndex) {
			return this.columnEditable[columnIndex];
		}

		@Override
		public int getRowCount() {
			return sdoTrack != null ? sdoTrack.size() : 0;
		}

		@Override
		public String getColumnName(int columnIndex) {
			return this.columnNames[columnIndex];
		}

		@Override
		public int getColumnCount() {
			return this.columnNames.length;
		}

		@Override
		public Object getValueAt(int row, int col) {
			if (sdoTrack == null)
				return null;

			switch (col) {
			case ITEM_NUM_COLUMN:
				return row;
			case ITEM_TYPE_COLUMN:
				return sdoTrack.get(row).getPositionType();
			case LATITUDE_COLUMN:
				return sdoTrack.get(row).getLatitudeDegrees();
			case LONGITUDE_COLUMN:
				return sdoTrack.get(row).getLongitudeDegrees();
			case ALTITUDE_COLUMN:
				return sdoTrack.get(row).getElevation();
			case DURATION_COLUMN:
				return sdoTrack.get(row).getDuration();
			case RADIUS_COLUMN:
				return sdoTrack.get(row).getRadius();
			}

			return null;
		}

		@Override
		public void setValueAt(Object object, int row, int col) {
			if (sdoTrack == null)
				return;

			SDOPosition curPos = sdoTrack.get(row);
			SDOPosition newPos;
			Angle newAngle;

			switch (col) {
			case LATITUDE_COLUMN:
				if (!(object instanceof String))
					return;
				if ((newAngle = table.toAngle((String) object)) == null)
					return;
				newPos = new SDOPosition(curPos.getPositionType(), newAngle.getDegrees(), curPos.getLongitudeDegrees(), curPos.getElevation(), curPos.getDuration(), curPos.getRadius());
				break;
			case LONGITUDE_COLUMN:
				if (!(object instanceof String))
					return;
				if ((newAngle = table.toAngle((String) object)) == null)
					return;
				newPos = new SDOPosition(curPos.getPositionType(), curPos.getLatitudeDegrees(), newAngle.getDegrees(), curPos.getElevation(), curPos.getDuration(), curPos.getRadius());
				break;
			case ALTITUDE_COLUMN:
				// The value stored in a SARPosition's elevation will always be in meters.
				// So when the altitude is displayed in feet, we will convert the incoming
				// value back to meters. This allows the user entring a value to operate in
				// whatever units are being displayed without thinking about conversion.
				if (!(object instanceof Double))
					return;
				double newVal = (Double) object;
				if (WorldWindVisualization.UNIT_IMPERIAL.equals(elevationUnit))
					newVal = ConversionUtils.feetToMeters(newVal);
				newPos = new SDOPosition(curPos.getPositionType(), curPos.getLatitudeDegrees(), curPos.getLongitudeDegrees(), newVal, curPos.getDuration(), curPos.getRadius());
				break;
			case DURATION_COLUMN:
				if (!(object instanceof Double))
					return;
				double durVal = (Double) object;
				newPos = new SDOPosition(curPos.getPositionType(), curPos.getLatitudeDegrees(), curPos.getLongitudeDegrees(), curPos.getElevation(), durVal, curPos.getRadius());
				break;
			case RADIUS_COLUMN:
				if (!(object instanceof Double))
					return;
				double radVal = (Double) object;
				newPos = new SDOPosition(curPos.getPositionType(), curPos.getLatitudeDegrees(), curPos.getLongitudeDegrees(), curPos.getElevation(), curPos.getDuration(), radVal);
				break;
			default:
				return;
			}

			sdoTrack.set(row, newPos);
		}
	}

	private Angle toAngle(String string) {
		if (Angle.ANGLE_FORMAT_DMS.equals(this.angleFormat)) {
			try {
				return Angle.fromDMS(string);
			} catch (Exception ignore) {
				return null;
			}
		}
		try {
			Number number = NumberFormat.getInstance().parse(string.trim());
			return Angle.fromDegrees(number.doubleValue());
		} catch (Exception ignore) {
			return null;
		}
	}

	public static String formatAngle(String format, Angle angle) {
		String s;
		if (Angle.ANGLE_FORMAT_DMS.equals(format))
			s = angle.toDMSString();
		else
			s = String.format("%7.4f\u00B0", angle.degrees);
		return s;
	}

	private String makeAngleDescription(double degrees) {
		return PositionTable.formatAngle(this.angleFormat, Angle.fromDegrees(degrees));
	}

	private String makeDurationDescription(double durationMillis) {
		String s;
		s = NumberFormat.getInstance().format(durationMillis);
		return s;
	}

	private String makeElevationDescription(double metersElevation) {
		String s;
		if (WorldWindVisualization.UNIT_IMPERIAL.equals(this.elevationUnit))
			s = NumberFormat.getInstance().format(ConversionUtils.metersToFeet(metersElevation));
		else // Default to metric units.
			s = NumberFormat.getInstance().format(metersElevation);
		return s;
	}

	private String makeRadiusDescription(double radiusMillis) {
		String s;
		s = NumberFormat.getInstance().format(radiusMillis);
		return s;
	}


	private static class DurationHeaderRenderer implements TableCellRenderer {
		private TableCellRenderer delegate;

		/**
		 * @param table TODO consider whether we actually need this 
		 */
		public DurationHeaderRenderer(TableCellRenderer delegate, PositionTable table) {
			this.delegate = delegate;
		}

		@Override
		public Component getTableCellRendererComponent(JTable tbl, Object value, boolean isSelected, boolean hasFocus,
				int row, int column) {
			if (this.delegate == null)
				return null;

			Component c = this.delegate.getTableCellRendererComponent(tbl, value, isSelected, hasFocus, row, column);
			if (c == null || !(c instanceof JLabel))
				return c;

			JLabel label = (JLabel) c;
			if (label.getText() == null)
				return c;

			label.setText(label.getText() + " (ms)");
			return label;
		}
	}
	
	
	private static class RadiusHeaderRenderer implements TableCellRenderer {
		private TableCellRenderer delegate;

		/**
		 * @param table TODO consider whether we actually need this 
		 */
		public RadiusHeaderRenderer(TableCellRenderer delegate, PositionTable table) {
			this.delegate = delegate;
		}

		@Override
		public Component getTableCellRendererComponent(JTable tbl, Object value, boolean isSelected, boolean hasFocus,
				int row, int column) {
			if (this.delegate == null)
				return null;

			Component c = this.delegate.getTableCellRendererComponent(tbl, value, isSelected, hasFocus, row, column);
			if (c == null || !(c instanceof JLabel))
				return c;

			JLabel label = (JLabel) c;
			if (label.getText() == null)
				return c;

			label.setText(label.getText() + " (m)");
			return label;
		}
	}

	
	private static class AltitudeHeaderRenderer implements TableCellRenderer {
		private TableCellRenderer delegate;
		private PositionTable table;

		public AltitudeHeaderRenderer(TableCellRenderer delegate, PositionTable table) {
			this.delegate = delegate;
			this.table = table;
		}

		@Override
		public Component getTableCellRendererComponent(JTable tbl, Object value, boolean isSelected, boolean hasFocus,
				int row, int column) {
			if (this.delegate == null)
				return null;

			Component c = this.delegate.getTableCellRendererComponent(tbl, value, isSelected, hasFocus, row, column);
			if (c == null || !(c instanceof JLabel))
				return c;

			JLabel label = (JLabel) c;
			if (label.getText() == null)
				return c;

			if (WorldWindVisualization.UNIT_IMPERIAL.equals(this.table.elevationUnit))
				label.setText(label.getText() + " (ft)");
			else // Default to metric units.
				label.setText(label.getText() + " (m)");
			return label;
		}
	}
	
	private static class AngleHeaderRenderer implements TableCellRenderer {
		private TableCellRenderer delegate;
		private PositionTable table;

		public AngleHeaderRenderer(TableCellRenderer delegate, PositionTable table) {
			this.delegate = delegate;
			this.table = table;
		}

		@Override
		public Component getTableCellRendererComponent(JTable tbl, Object value, boolean isSelected, boolean hasFocus,
				int row, int column) {
			if (this.delegate == null)
				return null;

			Component c = this.delegate.getTableCellRendererComponent(tbl, value, isSelected, hasFocus, row, column);
			if (c == null || !(c instanceof JLabel))
				return c;

			JLabel label = (JLabel) c;
			if (label.getText() == null)
				return c;

			if (Angle.ANGLE_FORMAT_DMS.equals(this.table.angleFormat))
				label.setText(label.getText() + " (dms)");
			else // Default to decimal degrees.
				label.setText(label.getText() + " (dd)");
			return label;
		}
	}

	private static class AngleCellRenderer extends DefaultTableCellRenderer {

		private static final long serialVersionUID = 1L;
		private PositionTable table;

		private AngleCellRenderer(PositionTable table) {
			this.table = table;
			setHorizontalAlignment(SwingConstants.RIGHT);
		}

		@Override
		public void setValue(Object value) {
			setText(value != null ? this.table.makeAngleDescription((Double) value) : "");
		}
	}

	private static class AltitudeCellRenderer extends DefaultTableCellRenderer {
		private static final long serialVersionUID = 1L;
		private PositionTable table;

		private AltitudeCellRenderer(PositionTable table) {
			this.table = table;
			setHorizontalAlignment(SwingConstants.RIGHT);
		}

		@Override
		protected void setValue(Object value) {
			setText(this.table.makeElevationDescription((Double) value));
		}
	}
	
	private static class DurationCellRenderer extends DefaultTableCellRenderer {
		private static final long serialVersionUID = 1L;
		private PositionTable table;

		private DurationCellRenderer(PositionTable table) {
			this.table = table;
			setHorizontalAlignment(SwingConstants.RIGHT);
		}

		@Override
		protected void setValue(Object value) {
			setText(this.table.makeDurationDescription((Double) value));
		}
	}	

	
	private static class RadiusCellRenderer extends DefaultTableCellRenderer {
		private static final long serialVersionUID = 1L;
		private PositionTable table;

		private RadiusCellRenderer(PositionTable table) {
			this.table = table;
			setHorizontalAlignment(SwingConstants.RIGHT);
		}

		@Override
		protected void setValue(Object value) {
			setText(this.table.makeRadiusDescription((Double) value));
		}
	}	

	
	private static class GeneralCellEditor extends DefaultCellEditor {
		private static final long serialVersionUID = 1L;
		private PositionTable table;
		private Object value;

		public GeneralCellEditor(JTextField textField, PositionTable table) {
			super(textField);
			this.table = table;
		}

		public PositionTable getTable() {
			return table;
		}

		@Override
		public Object getCellEditorValue() {
			return this.value;
		}

		@Override
		public boolean stopCellEditing() {
			String s = (String) super.getCellEditorValue();
			try {
				this.value = this.validateEditorText(s);
			} catch (Exception e) {
				((JComponent) getComponent()).setBorder(new LineBorder(Color.red));
				return false;
			}
			return super.stopCellEditing();
		}

		@Override
		public Component getTableCellEditorComponent(JTable tbl, Object val, boolean isSelected, int row,
				int column) {
			((JComponent) getComponent()).setBorder(new LineBorder(Color.black));
			this.value = null;
			try {
				this.value = this.createEditorText(val);
			} catch (Exception e) {
				return null;
			}
			return super.getTableCellEditorComponent(tbl, this.value, isSelected, row, column);
		}

		protected Object validateEditorText(String text) throws Exception {
			return text;
		}

		protected String createEditorText(Object val) throws Exception {
			return val.toString();
		}
	}

	private class AngleCellEditor extends GeneralCellEditor {

		private static final long serialVersionUID = 1L;
		double min, max;

		public AngleCellEditor(PositionTable table, double min, double max) {
			super(new JTextField(), table);
			this.min = min;
			this.max = max;
			((JTextField) getComponent()).setHorizontalAlignment(SwingConstants.RIGHT);
		}

		@Override
		protected Object validateEditorText(String text) throws Exception {
			Angle angle = this.getTable().toAngle(text);
			if (angle == null)
				throw new IllegalArgumentException(text);
			if (angle.degrees < min || angle.degrees > max)
				throw new IllegalArgumentException(text);
			return text;
		}

		@Override
		protected String createEditorText(Object value) throws Exception {
			String text = this.getTable().makeAngleDescription((Double) value);
			text = text.replaceAll("[D|d|\u00B0|'|\u2019|\"|\u201d]", " ").replaceAll("\\s+", " ");
			return text;
		}
	}

	private static class AltitudeCellEditor extends GeneralCellEditor {

		private static final long serialVersionUID = 1L;

		public AltitudeCellEditor(PositionTable table) {
			super(new JTextField(), table);
			((JTextField) getComponent()).setHorizontalAlignment(SwingConstants.RIGHT);
		}

		@Override
		protected Object validateEditorText(String text) throws Exception {
			Number number = NumberFormat.getInstance().parse(text);
			return number.doubleValue();
		}

		@Override
		protected String createEditorText(Object value) throws Exception {
			return this.getTable().makeElevationDescription((Double) value);
		}
	}

	private static class DurationCellEditor extends GeneralCellEditor {

		private static final long serialVersionUID = 1L;

		public DurationCellEditor(PositionTable table) {
			super(new JTextField(), table);
			((JTextField) getComponent()).setHorizontalAlignment(SwingConstants.RIGHT);
		}

		@Override
		protected Object validateEditorText(String text) throws Exception {
			Number number = NumberFormat.getInstance().parse(text);
			return number.doubleValue();
		}

		@Override
		protected String createEditorText(Object value) throws Exception {
			return this.getTable().makeDurationDescription((Double) value);
		}
	}

	
	private static class RadiusCellEditor extends GeneralCellEditor {

		private static final long serialVersionUID = 1L;

		public RadiusCellEditor(PositionTable table) {
			super(new JTextField(), table);
			((JTextField) getComponent()).setHorizontalAlignment(SwingConstants.RIGHT);
		}

		@Override
		protected Object validateEditorText(String text) throws Exception {
			Number number = NumberFormat.getInstance().parse(text);
			return number.doubleValue();
		}

		@Override
		protected String createEditorText(Object value) throws Exception {
			return this.getTable().makeRadiusDescription((Double) value);
		}
	}
	
}
