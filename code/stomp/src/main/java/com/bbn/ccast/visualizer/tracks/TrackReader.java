//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */
package com.bbn.ccast.visualizer.tracks;


/**
 * @author dcollins
 * @version $Id: TrackReader.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public interface TrackReader {
	String getDescription();

	boolean canRead(Object source);

	SDOTrack[] read(Object source);
}
