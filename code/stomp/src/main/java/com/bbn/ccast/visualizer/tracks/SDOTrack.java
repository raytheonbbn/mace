//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tracks;

import com.fasterxml.jackson.annotation.JsonIgnore;
import gov.nasa.worldwind.WWObjectImpl;
import org.apache.log4j.Logger;

import java.awt.*;
import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;
import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.UUID;

//@JsonTypeInfo(use=JsonTypeInfo.Id.NAME, include=JsonTypeInfo.As.PROPERTY, property="@type")
public class SDOTrack extends WWObjectImpl implements Iterable<SDOPosition> {
	private static class FormatInfo {
		private TrackReader reader;
		@SuppressWarnings("unused")
		private int format;  // use this if we make more readers for tracks (e.g., mavlink plan - json)

		private FormatInfo(TrackReader reader, int format) {
			this.reader = reader;
			this.format = format;
		}
	}

	public static final int FORMAT_MAVLINK = 1;

	private static final Logger logger = Logger.getLogger(SDOTrack.class.getName());

	
	@JsonIgnore
	private static int nextColor = 0;
	@JsonIgnore
	private static Color[] colors = new Color[] { Color.RED, Color.GREEN, Color.BLUE, Color.MAGENTA, Color.CYAN,
			Color.ORANGE, Color.PINK, Color.YELLOW };
	
	public static SDOTrack fromFile(String filePath) {
		if (filePath == null) {
			String message = "nullValue.FilePathIsNull";
			logger.error(message);
			throw new IllegalArgumentException(message);
		}

		File file = new File(filePath);

		if (!file.exists()) {
			String message = "generic.FileNotFound: " + filePath;
			logger.error(message);
			throw new IllegalArgumentException(message);
		}

		SDOTrack track = null;

		FormatInfo[] formatInfoArray = new FormatInfo[] { new FormatInfo(new MAVLinkTrackReader(), FORMAT_MAVLINK), };

		int formatIndex;
		for (formatIndex = 0; formatIndex < formatInfoArray.length; formatIndex++) {
			track = readTrack(filePath, formatInfoArray[formatIndex]);
			if (track != null)
				break;
		}

		if (track != null) {
			track.setFile(file);
			track.setFormat(formatInfoArray[formatIndex].format);
			track.setName(file.getName());
			track.setUid(UUID.randomUUID().toString());
		}

		return track;
	}

	public static SDOTrack fromString(String trackString) {

		SDOTrack track = null;

		FormatInfo[] formatInfoArray = new FormatInfo[] { new FormatInfo(new MAVLinkTrackReader(), FORMAT_MAVLINK), };

		int formatIndex;
		for (formatIndex = 0; formatIndex < formatInfoArray.length; formatIndex++) {
			track = readTrackString(trackString, formatInfoArray[formatIndex]);
			if (track != null)
				break;
		}

		if (track != null) {
			track.setFormat(formatInfoArray[formatIndex].format);
//			track.setName(file.getName());
			track.setUid(UUID.randomUUID().toString());
		}

		return track;
	}

	private static Color nextColor() {
		return colors[nextColor++ % colors.length];
	}

	private static SDOTrack readTrackString(String trackString, FormatInfo format) {
		SDOTrack[] tracks = null;
		InputStream in = new ByteArrayInputStream(trackString.getBytes());
		try {
			tracks = format.reader.read(in);
		} catch (Exception e) {
			logger.error("Exception attempting to read SDOTrack string: ", e);
		}

		if (tracks == null || tracks.length == 0) {
			return null;
		}

		if (tracks.length > 1) {
			String message = "Error. Not currently set up to process multiple tracks in a single file!";
			logger.error(message);
		}
		return tracks[0];
	}

	private static SDOTrack readTrack(String filePath, FormatInfo format) {
		if (!format.reader.canRead(filePath))
			return null;

		SDOTrack[] tracks = null;
		try {
			tracks = format.reader.read(filePath);
		} catch (Exception e) {
			String message = "generic.ExceptionAttemptingToReadFile: " + filePath;
			logger.error(message);
		}

		if (tracks == null || tracks.length == 0) {
			return null;
		}

		if (tracks.length > 1) {
			String message = "Error. Not currently set up to process multiple tracks in a single file!";
			logger.error(message);
		}
		return tracks[0];
	}

	public static void toFile(SDOTrack track, String filePath, int format) throws IOException {
		if (track == null)
			throw new IllegalArgumentException("track is null");
		if (filePath == null)
			throw new IllegalArgumentException("filePath is null");

		if (format == FORMAT_MAVLINK)
			writeMAVLink(track, filePath);
		// If no format is specified, then do nothing.
	}

	public static String writeToString(SDOTrack track) throws UnsupportedEncodingException {
		ByteArrayOutputStream baos = new ByteArrayOutputStream();
		BufferedOutputStream out = new BufferedOutputStream(baos);
		writeMAVLink(track, out);
		return baos.toString();
	}
	
	private static void writeMAVLink(SDOTrack track, String filePath) throws IOException {
		MAVLinkWriter writer = new MAVLinkWriter(filePath);
		writer.writeTrack(track);
		writer.close();
	}

	private static void writeMAVLink(SDOTrack track, BufferedOutputStream out) throws UnsupportedEncodingException {
		MAVLinkWriter writer = new MAVLinkWriter(out);
		writer.writeTrack(track);
		writer.close();
	}

	// Meta-track properties.
	@JsonIgnore
	private transient File file = null;

	private String name = null;
	private String uid = null;

	@JsonIgnore
	private transient int format = 0;
	@JsonIgnore
	private transient long lastSaveTime = 0L;

	@JsonIgnore
	private transient long lastModifiedTime = 0L;
	

	// Track properties.
	@JsonIgnore
	private transient double offset = 0;

	@JsonIgnore
	private Color color = nextColor();

	private ArrayList<SDOPosition> positions;

	private boolean isUnordered;

	@JsonIgnore
	private transient PropertyChangeSupport propChangeSupport = new PropertyChangeSupport(this);

	public SDOTrack(String uid, String name) {
		this.uid = uid;
		this.name = name;
		this.positions = new ArrayList<SDOPosition>();
		this.isUnordered = false;
	}

	public SDOTrack(String name) {
		this.uid = UUID.randomUUID().toString();
		this.name = name;
		this.positions = new ArrayList<SDOPosition>();
		this.isUnordered = false;
	} 
	
	
	public void add(int index, SDOPosition position) {
		if (position == null)
			return;

		if (index >= this.positions.size())
			this.positions.add(position);
		else
			this.positions.add(index, position);

		this.markDirty();
		this.firePropertyChange(TrackController.TRACK_MODIFY, null, this);
	}

	@Override
	public synchronized void addPropertyChangeListener(PropertyChangeListener listener) {
		if(propChangeSupport == null) {propChangeSupport = new PropertyChangeSupport(this);}
		this.propChangeSupport.addPropertyChangeListener(listener);
	}

	@Override
	public synchronized void addPropertyChangeListener(String propertyName, PropertyChangeListener listener) {
		if(propChangeSupport == null) {propChangeSupport = new PropertyChangeSupport(this);}
		this.propChangeSupport.addPropertyChangeListener(propertyName, listener);
	}

	public void appendPosition(SDOPosition position) {
		if (position == null)
			return;

		this.positions.add(position);
		this.markDirty();
		this.firePropertyChange(TrackController.TRACK_MODIFY, null, this);
	}

	public void clearDirtyBit() {
		long time = System.currentTimeMillis();
		this.lastSaveTime = time;
		this.lastModifiedTime = time;
		this.firePropertyChange(TrackController.TRACK_DIRTY_BIT, null, this);
	}

	@Override
	public void firePropertyChange(String propertyName, Object oldValue, Object newValue) {
		if(propChangeSupport == null) {propChangeSupport = new PropertyChangeSupport(this);}
		this.propChangeSupport.firePropertyChange(propertyName, oldValue, newValue);
	}

	public SDOPosition get(int index) {
		return this.positions.size() > index ? this.positions.get(index) : null;
	}

	public Color getColor() {
		return color;
	}

	public File getFile() {
		return this.file;
	}

	public int getFormat() {
		return format;
	}

	public long getLastModifiedTime() {
		return this.lastModifiedTime;
	}

	public long getLastSaveTime() {
		return this.lastSaveTime;
	}

	public String getName() {
		return this.name;
	}

	public String getUid() {
		return this.uid;
	}

	public double getOffset() {
		return offset;
	}

	public ArrayList<SDOPosition> getPositions() {
		return this.positions;
	}

	public void insertPosition(int index, SDOPosition position) {
		if (position == null || index < 0)
			return;

		this.positions.add(index, position);
		this.markDirty();
		this.firePropertyChange(TrackController.TRACK_MODIFY, null, this);
	}

	@JsonIgnore
	public boolean isDirty() {
		return this.lastModifiedTime == 0L || this.lastSaveTime == 0L || (this.lastModifiedTime > this.lastSaveTime);
	}

	@Override
	public Iterator<SDOPosition> iterator() {
		return new Iterator<SDOPosition>() {
			private Iterator<SDOPosition> iter = SDOTrack.this.positions.iterator();

			@Override
			public boolean hasNext() {
				return this.iter.hasNext();
			}

			@Override
			public SDOPosition next() {
				return this.iter.next();
			}

			@Override
			public void remove() {
				throw new UnsupportedOperationException("Remove operation not supported for SARTrack iterator");
			}
		};
	}

	public void markDirty() {
		this.lastModifiedTime = System.currentTimeMillis();
		this.firePropertyChange(TrackController.TRACK_DIRTY_BIT, null, this);
	}

	public void removePosition(int index) {
		if (index < 0 || index >= this.positions.size())
			return;

		this.positions.remove(index);
		this.markDirty();
		this.firePropertyChange(TrackController.TRACK_MODIFY, null, this);
	}

	public void removePositions(int[] positionNumbers) {
		Arrays.sort(positionNumbers);
		for (int i = positionNumbers.length - 1; i >= 0; i--) {
			if (positionNumbers[i] < 0 || positionNumbers[i] >= this.positions.size())
				continue;

			this.positions.remove(positionNumbers[i]);
		}

		this.markDirty();
		this.firePropertyChange(TrackController.TRACK_MODIFY, null, this);
	}

	@Override
	public synchronized void removePropertyChangeListener(PropertyChangeListener listener) {
		if(propChangeSupport == null) {return;}
		this.propChangeSupport.removePropertyChangeListener(listener);
	}

	public void set(int index, SDOPosition position) {
		if (position == null)
			return;

		if (index >= this.positions.size())
			this.positions.add(position);
		else
			this.positions.set(index, position);

		this.markDirty();
		this.firePropertyChange(TrackController.TRACK_MODIFY, null, index);
	}

	public void setColor(Color color) {
		this.color = color;
	}

	public void setFile(File file) {
		this.file = file;
	}

	public void setFormat(int format) {
		this.format = format;
	}

	public void setName(String name) {
		this.name = name;
		this.firePropertyChange(TrackController.TRACK_NAME, null, this);
	}

	public void setUid(String uid) {
		this.uid = uid;
		this.firePropertyChange(TrackController.TRACK_UID, null, this);
	}

	public void setOffset(double offset) {
		double oldOffset = this.offset;
		this.offset = offset;

		this.firePropertyChange(TrackController.TRACK_OFFSET, oldOffset, this.offset);
	}

	public void setPosition(int index, SDOPosition position) {
		if (position == null || index < 0)
			return;

		this.positions.set(index, position);
		this.markDirty();
		this.firePropertyChange(TrackController.TRACK_MODIFY, null, index);
	}

	/**
	 * @return the isUnordered
	 */
	public boolean isUnordered() {
		return isUnordered;
	}

	/**
	 * @param isUnordered the isUnordered to set
	 */
	public void setUnordered(boolean isUnordered) {
		if (this.isUnordered != isUnordered) {
			this.isUnordered = isUnordered;
			//String vis = isUnordered ? TrackController.TRACK_UNORDERED : TrackController.TRACK_ORDERED;
			//firePropertyChange(vis, null, this);	
		}
	}

	public int size() {
		return this.positions.size();
	}

	@Override
	public String toString() {
		return this.name;
	}
}
