//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */
package com.bbn.ccast.visualizer.actions;

import gov.nasa.worldwind.WorldWindow;

import javax.swing.*;

/**
 * @author dcollins
 * @version $Id: SARScreenShotAction.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class CCASTScreenShotAction extends ScreenShotAction {

	private static final long serialVersionUID = 1L;

	public CCASTScreenShotAction(WorldWindow wwd, Icon icon) {
		super(wwd);
		this.putValue(Action.NAME, "Screen Shot...");
		this.putValue(Action.SHORT_DESCRIPTION, "Save a screen shot");
		this.putValue(Action.SMALL_ICON, icon);
	}
}
