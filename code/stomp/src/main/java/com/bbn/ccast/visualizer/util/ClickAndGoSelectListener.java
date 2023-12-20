//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */
package com.bbn.ccast.visualizer.util;

import gov.nasa.worldwind.View;
import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.event.SelectEvent;
import gov.nasa.worldwind.event.SelectListener;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.util.Logging;

/**
 * Handles view 'fly to' on left clicked picked objects with a position.
 *
 * @author Patrick Murris
 * @version $Id: ClickAndGoSelectListener.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class ClickAndGoSelectListener implements SelectListener {

    private final WorldWindow wwd;
    private final Class<?> pickedObjClass;    // Which picked object class do we handle
    private final double elevationOffset;  // Meters above the target position

    public ClickAndGoSelectListener(WorldWindow wwd, Class<?> pickedObjClass) {
        if (wwd == null) {
            String msg = Logging.getMessage("nullValue.WorldWindow");
            Logging.logger().severe(msg);
            throw new IllegalArgumentException(msg);
        }

        this.wwd = wwd;
        this.pickedObjClass = pickedObjClass;
        this.elevationOffset = 0d;
    }

    public ClickAndGoSelectListener(WorldWindow wwd, Class<?> pickedObjClass, double elevationOffset) {
        if (wwd == null) {
            String msg = Logging.getMessage("nullValue.WorldWindow");
            Logging.logger().severe(msg);
            throw new IllegalArgumentException(msg);
        }
        if (pickedObjClass == null) {
            String msg = Logging.getMessage("nullValue.ClassIsNull");
            Logging.logger().severe(msg);
            throw new IllegalArgumentException(msg);
        }

        this.wwd = wwd;
        this.pickedObjClass = pickedObjClass;
        this.elevationOffset = elevationOffset;
    }

    /**
     * Select Listener implementation.
     *
     * @param event the SelectEvent
     */
    @Override
	public void selected(SelectEvent event) {
        if (event.getEventAction().equals(SelectEvent.LEFT_CLICK)) {
            // This is a left click
            if (event.hasObjects() && event.getTopPickedObject().hasPosition()) {
                // There is a picked object with a position
                if (event.getTopObject().getClass().equals(pickedObjClass)) {
                    // This object class we handle and we have an orbit view
                    Position targetPos = event.getTopPickedObject().getPosition();
                    View view = this.wwd.getView();
                    // Use a PanToIterator to iterate view to target position
                    if (view != null) {
                        // The elevation component of 'targetPos' here is not the surface elevation,
                        // so we ignore it when specifying the view center position.
                        view.goTo(new Position(targetPos, 0),
                                targetPos.getElevation() + this.elevationOffset);
                    }
                }
            }
        }
    }

}
