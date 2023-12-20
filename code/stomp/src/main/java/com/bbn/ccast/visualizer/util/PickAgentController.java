//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */

package com.bbn.ccast.visualizer.util;

import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.event.SelectEvent;
import gov.nasa.worldwind.event.SelectListener;
import gov.nasa.worldwind.symbology.TacticalSymbol;

import java.util.HashSet;
import java.util.Set;

public class PickAgentController implements SelectListener {
    protected WorldWindow wwd;
    protected Object highlightEventType = SelectEvent.LEFT_CLICK;
    public Set<String> selectedUids = new HashSet<>();
    private OnItemPicked onPickCallback = null;

    public interface OnItemPicked {
        void onItemPicked(TacticalSymbol object, int x, int y);
    }

    /**
     * Creates a controller for a specified World Window.
     *
     * @param wwd                the World Window to monitor.
     * @param highlightEventType the type of {@link SelectEvent} to highlight in response to. The default is {@link
     *                           SelectEvent#ROLLOVER}.
     */
    public PickAgentController(WorldWindow wwd, Object highlightEventType) {
        this.wwd = wwd;
        this.highlightEventType = highlightEventType;
        this.wwd.addSelectListener(this);
    }

    public PickAgentController setOnPickCallback(OnItemPicked onPickCallback) {
        this.onPickCallback = onPickCallback;
        return this;
    }

    public void dispose() {
        this.wwd.removeSelectListener(this);
    }

    @Override
	public void selected(SelectEvent event) {
        try {
            if (this.highlightEventType != null && event.getEventAction().equals(this.highlightEventType)) {
                int x = event.getMouseEvent().getX();
                int y = event.getMouseEvent().getY();
                select(event.getTopObject(), x, y);
            }
        } catch (Exception e) {
            // Wrap the handler in a try/catch to keep exceptions from bubbling up
            Util.getlogger().warn(e.getMessage() != null ? e.getMessage() : e.toString());
        }
    }


    protected void select(Object o, int x, int y) {
        if(o instanceof TacticalSymbol) {
            TacticalSymbol symbol = (TacticalSymbol) o;
            if(symbol != null && this.onPickCallback != null) {
                this.onPickCallback.onItemPicked(symbol, x, y);
            }
        }
    }

}
