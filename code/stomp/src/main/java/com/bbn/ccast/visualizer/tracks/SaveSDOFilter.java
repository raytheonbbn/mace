//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */
package com.bbn.ccast.visualizer.tracks;

import gov.nasa.worldwind.util.Logging;
import gov.nasa.worldwind.util.WWIO;

import java.io.File;

/**
 * @author dcollins
 * @version $Id: SaveTrackFilter.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class SaveSDOFilter extends javax.swing.filechooser.FileFilter implements java.io.FileFilter {
	private final int format;
	private final String description;
	private final String[] suffixes;

	public SaveSDOFilter(int format, String description, String[] suffixes) {
		this.format = format;
		this.description = description;
		this.suffixes = new String[suffixes.length];
		System.arraycopy(suffixes, 0, this.suffixes, 0, suffixes.length);
	}

	public int getFormat() {
		return this.format;
	}

	@Override
	public String getDescription() {
		return this.description;
	}

	public String[] getSuffixes() {
		String[] copy = new String[this.suffixes.length];
		System.arraycopy(this.suffixes, 0, copy, 0, this.suffixes.length);
		return copy;
	}

	@Override
	public boolean accept(File file) {
		return true;
	}

	public File appendSuffix(File file) {
		if (file == null) {
			String message = Logging.getMessage("nullValue.FileIsNull");
			Logging.logger().severe(message);
			throw new IllegalArgumentException(message);
		}

		String path = file.getPath();

		String lowerCasePath = path.toLowerCase();
		for (String suffix : this.suffixes) {
			if (lowerCasePath.endsWith(suffix))
				return file;
		}

		return new File(WWIO.replaceSuffix(path, this.suffixes[0]));
	}
}