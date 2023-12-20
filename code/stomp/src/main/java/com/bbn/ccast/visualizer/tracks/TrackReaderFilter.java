//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */
package com.bbn.ccast.visualizer.tracks;

import gov.nasa.worldwind.util.Logging;

/**
 * @author dcollins
 * @version $Id: TrackReaderFilter.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class TrackReaderFilter extends javax.swing.filechooser.FileFilter implements java.io.FileFilter {
	protected final TrackReader trackReader;

	public TrackReaderFilter(TrackReader trackReader) {
		this.trackReader = trackReader;
	}

	public final TrackReader getTrackReader() {
		return this.trackReader;
	}

	@Override
	public String getDescription() {
		return this.trackReader.getDescription();
	}

	@Override
	public boolean accept(java.io.File file) {
		if (file == null) {
			String message = Logging.getMessage("nullValue.FileIsNull");
			Logging.logger().severe(message);
			throw new IllegalArgumentException(message);
		}

		return file.isDirectory() || this.trackReader.canRead(file);
	}
}
