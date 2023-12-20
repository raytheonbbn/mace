//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tracks;

import com.MAVLink.enums.MAV_CMD;
import gov.nasa.worldwind.util.Logging;

import java.io.BufferedOutputStream;
import java.io.UnsupportedEncodingException;

public class MAVLinkWriter {

	public static final int MAVLINK_UNCERTAINTY_RADIUS = 2; //meters
	public static final int MAVLINK_COORDINATE_FRAME = 3;
	public static final int MAVLINK_AUTOCONTINUE = 1;
	
	
	private static final String DEFAULT_ENCODING = "US-ASCII";
	
	
	public static int convertPositionTypeToMavLinkCommand(PositionType type) {

		switch (type) {
		case GOTO:
			return MAV_CMD.MAV_CMD_NAV_WAYPOINT;
		case LOOKPOINT :
			return MAV_CMD.MAV_CMD_DO_SET_ROI;
		case LOITER:
			return MAV_CMD.MAV_CMD_NAV_LOITER_TIME;
		case LAND:
			return MAV_CMD.MAV_CMD_NAV_LAND;
		}
		return 0;
	}
	
	private final java.io.PrintStream printStream;
	private final String encoding;
	
	private int lineNumber = 0;
	private int wpNumber = 1;

	public MAVLinkWriter(java.io.OutputStream stream) throws java.io.IOException {
		this(stream, DEFAULT_ENCODING);
	}

	public MAVLinkWriter(java.io.OutputStream stream, String encoding) throws java.io.IOException {
		if (stream == null) {
			String msg = Logging.getMessage("nullValue.InputStreamIsNull");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}
		if (encoding == null) {
			String msg = Logging.getMessage("nullValue.StringIsNull");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}

		this.encoding = encoding;
		this.printStream = new java.io.PrintStream(new BufferedOutputStream(stream), false, // Disable
																									// autoflush.
				this.encoding); // Character mapping from 16-bit UTF characters to bytes.
	}

	public MAVLinkWriter(String path) throws java.io.IOException {
		this(path, DEFAULT_ENCODING);
	}

	public MAVLinkWriter(BufferedOutputStream out) throws UnsupportedEncodingException {
		this(out, DEFAULT_ENCODING);
	}

	public MAVLinkWriter(BufferedOutputStream out, String encoding) throws UnsupportedEncodingException {
		if (encoding == null) {
			String msg = Logging.getMessage("nullValue.StringIsNull");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}

		this.encoding = encoding;
		this.printStream = new java.io.PrintStream(out,
				false, // Disable autoflush.
				this.encoding); // Character mapping from 16-bit UTF characters to bytes.
	}

	public MAVLinkWriter(String path, String encoding) throws java.io.IOException {
		if (path == null) {
			String msg = Logging.getMessage("nullValue.PathIsNull");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}
		if (encoding == null) {
			String msg = Logging.getMessage("nullValue.StringIsNull");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}

		this.encoding = encoding;
		this.printStream = new java.io.PrintStream(new BufferedOutputStream(new java.io.FileOutputStream(path)),
				false, // Disable autoflush.
				this.encoding); // Character mapping from 16-bit UTF characters to bytes.
	}

	public void close() {
		doFlush();
		this.printStream.close();
	}

	private void doFlush() {
		this.printStream.flush();
	}

	private void doWriteTrack(SDOTrack track, java.io.PrintStream out) {
		if (track != null && track.getPositions() != null) {
			out.print("QGC WPL 110\r\n");
			// write out home location, which we'll say is the first location in the track.
			doWriteTrackPos(track.getPositions().get(0), out);
			this.lineNumber++;			
			if (this.wpNumber > 0) {
				this.wpNumber = 0;
			}

			for (SDOPosition sdoPos : track.getPositions()) {
				doWriteTrackPos(sdoPos, out);
			
				this.lineNumber++;			
				if (this.wpNumber > 0) {
					this.wpNumber = 0;
				}
//				if (sdoPos.getPositionType() == PositionType.GOTO) {
//					this.wpNumber++;
//				}
			}
		}
	}

	private void doWriteTrackPos(SDOPosition sdoPos, java.io.PrintStream out) {
		if (sdoPos != null) {

			StringBuilder sb = new StringBuilder();
			// index
			sb.append(this.lineNumber);
			sb.append("\t");
			// current wp
			sb.append(wpNumber);
			sb.append("\t");
			// coordinate frame
			sb.append(MAVLinkWriter.MAVLINK_COORDINATE_FRAME);
			sb.append("\t");
			// command
			sb.append(convertPositionTypeToMavLinkCommand(sdoPos.getPositionType()));
			sb.append("\t");
			// param 1 - time in seconds at location
			sb.append(formatDuration(sdoPos.getDuration()));
			sb.append("\t");
			// param 2 - uncertainty radius in meters
			sb.append(MAVLinkWriter.MAVLINK_UNCERTAINTY_RADIUS);
			sb.append("\t");
			// param 3 radius, pass or loiter
			sb.append(sdoPos.getRadius());
			sb.append("\t");
			// param 4 - heading in degrees
			sb.append("0\t");
			// latitude
			sb.append(sdoPos.getLatitudeDegrees());
			sb.append("\t");
			// longitude
			sb.append(sdoPos.getLongitudeDegrees());
			sb.append("\t");
			// altitude
			sb.append(sdoPos.getAltitude());
			sb.append("\t");
			// autocontinue
			sb.append(MAVLinkWriter.MAVLINK_AUTOCONTINUE);

			out.print(sb.toString());
			out.print("\r\n");
			doFlush();
		}
	}

	public final String getEncoding() {
		return this.encoding;
	}

	public void writeTrack(SDOTrack track) {
		if (track == null) {
			String msg = Logging.getMessage("nullValue.TrackIsNull");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}

		doWriteTrack(track, this.printStream);
		doFlush();
	}


    private String formatDuration(double duration)
    {
        return Double.toString(duration / 1000.0); // duration in seconds
    }


}
