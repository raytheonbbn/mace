//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tracks;

import com.MAVLink.enums.MAV_CMD;
import org.apache.log4j.Logger;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

public class MAVLinkReader extends AbstractTrackReader {

	private static final Logger logger = Logger.getLogger(MAVLinkReader.class.getName());

	public static final String MAVLINK_HEADER_STRING = "QGC WPL";
	
	private List<SDOTrack> tracks;
	private String name;

	public MAVLinkReader() {
		this.tracks = new ArrayList<SDOTrack>();
	}

	@Override
	public String getDescription() {
		return "MAVLink (*.mavlink)";
	}

	public String getName() {
		return this.name;
	}

	@Override
	protected boolean acceptFilePath(String filePath) {
		String lowerCasePath = filePath.toLowerCase();
		return lowerCasePath.endsWith(".mavlink") || lowerCasePath.endsWith(".txt");
	}

	/**
	 * @param path
	 * @throws IllegalArgumentException
	 *             if <code>path</code> is null
	 * @throws IOException
	 */
	public void readFile(String path) throws IOException {
		if (path == null) {
			String msg="nullValue.PathIsNull";
			logger.error(msg);
			throw new IllegalArgumentException(msg);
		}

		java.io.File file = new java.io.File(path);
		if (!file.exists()) {
			String msg = "generic.FileNotFound. " + path;
			logger.error(msg);
			throw new java.io.FileNotFoundException(path);
		}

		this.name = file.getName();
		try(java.io.FileInputStream fis = new java.io.FileInputStream(file)) {
			this.doRead(fis);
		} catch (Exception e) {
			throw e;
		}

	}

	/**
	 * @param stream
	 * @param streamName
	 * @throws IllegalArgumentException
	 *             if <code>stream</code> is null
	 * @throws IOException
	 */
	public void readStream(InputStream stream, String streamName) throws IOException {
		if (stream == null) {
			String msg = "nullValue.InputStreamIsNull";
			logger.error(msg);
			throw new IllegalArgumentException(msg);
		}

		this.name = streamName != null ? streamName : "Un-named stream";
		this.doRead(stream);
	}

	@Override
	protected SDOTrack[] doRead(InputStream stream) throws IOException {
		String row = null;
        BufferedReader reader = new BufferedReader(new InputStreamReader(stream));

		String header = reader.readLine();
		if (!(header.trim().startsWith(MAVLINK_HEADER_STRING))) {
			String msg = "Bad header string: " + header;
			logger.error(msg);
			throw new IllegalArgumentException(msg);
		}

		SDOTrack track = new SDOTrack(this.name);
		int lineNumber = 0;

		try {
			do {
				row = reader.readLine();
				if (row != null) {
					SDOPosition sdoPos = this.parseSentence(row);
					// skip the first row/position, assuming it's the start position.
					if (lineNumber > 0) {
						track.add(lineNumber, sdoPos);
					}
					lineNumber++;
				}
			} while (row != null);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		tracks.clear();
		tracks.add(track);

		SDOTrack[] trackArray = new SDOTrack[1];
		trackArray[0] = track;
		
		return trackArray;
	}

	
	public static PositionType convertMavLinkCommandToPositionType(int cmdTypeInt) {

		switch (cmdTypeInt) {
		case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
			return PositionType.GOTO;
		case MAV_CMD.MAV_CMD_DO_SET_ROI :
			return PositionType.LOOKPOINT;
		case MAV_CMD.MAV_CMD_NAV_LOITER_TIME :
			return PositionType.LOITER;
		case MAV_CMD.MAV_CMD_NAV_LAND :
			return PositionType.LAND;
		}
		return null;
	}

	
	private SDOPosition parseSentence(String row) {
		String[] words = row.split("\t");
		
		if (words.length < 12) {
			String msg = "Invalid MavLink Row Data: " + row;
			logger.error(msg);
			return null;
		}

		PositionType type = convertMavLinkCommandToPositionType(Integer.parseInt(words[3]));
		double radius = Double.parseDouble(words[6]);
		double duration = Double.parseDouble(words[4]) * 1000.0; // now in ms
		double latitude = Double.parseDouble(words[8]);
		double longitude = Double.parseDouble(words[9]);
		double elevation = Double.parseDouble(words[10]);

		SDOPosition sdoPos = new SDOPosition(type, latitude, longitude, elevation, duration, radius);
		
		return sdoPos;
	}

	public List<SDOTrack> getTracks() {
		return this.tracks;
	}
	
}