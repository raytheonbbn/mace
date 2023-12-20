//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */

package com.bbn.ccast.visualizer.util;


import com.bbn.ccast.AgentTelemPackage;
import com.bbn.ccast.Utils;
import com.bbn.ccast.visualizer.WorldWindAppFrame;
import com.bbn.ccast.visualizer.WorldWindAppPanel;
import com.bbn.ccast.visualizer.WorldWindVisualization;
import gov.nasa.worldwind.Disposable;
import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.avlist.AVList;
import gov.nasa.worldwind.event.SelectEvent;
import gov.nasa.worldwind.event.SelectListener;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.layers.AnnotationLayer;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.symbology.SymbologyConstants;
import gov.nasa.worldwind.symbology.milstd2525.MilStd2525TacticalSymbol;
import gov.nasa.worldwind.util.Logging;

/**
 * Controls display of tool tips on picked objects. Any shape implementing {@link AVList} can participate. Shapes
 * provide tool tip text in their AVList for either or both of hover and rollover events. The keys associated with the
 * text are specified to the constructor.
 *
 * @author tag
 * @version $Id: ToolTipController.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class ToolTipController implements SelectListener, Disposable {
    protected WorldWindow wwd;
    protected String hoverKey = AVKey.HOVER_TEXT;
    protected String rolloverKey = AVKey.ROLLOVER_TEXT;
    protected Object lastRolloverObject;
    protected Object lastHoverObject;
    protected AnnotationLayer layer;
    protected ToolTipAnnotation annotation;
    private String lastPathAgentUid;



    /**
     * Create a controller for a specified {@link WorldWindow} that displays tool tips on hover and/or rollover.
     *
     * @param wwd         the World Window to monitor.
     * @param rolloverKey the key to use when looking up tool tip text from the shape's AVList when a rollover event
     *                    occurs. May be null, in which case a tool tip is not displayed for rollover events.
     * @param hoverKey    the key to use when looking up tool tip text from the shape's AVList when a hover event
     *                    occurs. May be null, in which case a tool tip is not displayed for hover events.
     */
    public ToolTipController(WorldWindow wwd, String rolloverKey, String hoverKey) {
        this.wwd = wwd;
        this.hoverKey = hoverKey;
        this.rolloverKey = rolloverKey;

        this.wwd.addSelectListener(this);
    }

    /**
     * Create a controller for a specified {@link WorldWindow} that displays "DISPLAY_NAME" on rollover.
     *
     * @param wwd the World Window to monitor.
     */
    public ToolTipController(WorldWindow wwd) {
        this.wwd = wwd;
        this.rolloverKey = AVKey.DISPLAY_NAME;

        this.wwd.addSelectListener(this);
    }



    @Override
	public void dispose() {
        this.wwd.removeSelectListener(this);
    }

    protected String getHoverText(SelectEvent event) {
        return event.getTopObject() != null && event.getTopObject() instanceof AVList ?
                ((AVList) event.getTopObject()).getStringValue(this.hoverKey) : null;
    }

    private AgentTelemPackage getAgentTelemPackageFromSelectEvent(SelectEvent event) {
        if(event.getTopObject() instanceof MilStd2525TacticalSymbol) {
            String uid = WorldWindAppPanel.getUidFromSymbol((MilStd2525TacticalSymbol)event.getTopObject());
            AgentTelemPackage uidPkg = null;
//            AgentTelemPackage uidPkg = WorldWindAppFrame.getInstance().getVisualization().getDevice().getAgentUidToTelemPackageMap().get(uid);
            return uidPkg;
        }
        return null;
    }
    
    protected String getRolloverText(SelectEvent event) {
    	AgentTelemPackage uidPkg = getAgentTelemPackageFromSelectEvent(event);
    	if (uidPkg != null) {
    		//Object represents and agent
    		StringBuilder builder = new StringBuilder();

    		builder.append("Battery: " + uidPkg.getBatteryLevel() + System.lineSeparator());
    		Long timestampOfLastMessage = uidPkg.getTimestamp();
    		Long timeStampDelta = System.currentTimeMillis() - timestampOfLastMessage;
    		String commsMessage = "";
    		if(timeStampDelta <= 550) {
    			commsMessage = "< 500ms";
    		} else if (timeStampDelta < 1050) {
    			commsMessage = "< 1000ms";
    		} else if (timeStampDelta < 2050) {
    			commsMessage = "< 2000ms";
    		} else {
    			commsMessage = timeStampDelta.toString();
    		}
    		builder.append("Last pkg " + commsMessage + System.lineSeparator());

            double elev = Utils.EARTH.getElevation(Angle.fromDegreesLatitude(uidPkg.getLatitude()),
                    Angle.fromDegreesLongitude(uidPkg.getLongitude()));
    		if(uidPkg.getAltitude() - elev > 0) {
    			builder.append("Alt: " + uidPkg.getAltitude() + System.lineSeparator());
    		}

//    		boolean firstTactic = true;
//    		if(uidPkg.getTactics() != null && !uidPkg.getTactics().isEmpty()) {
//    			builder.append("Tactics: ");
//    			for(String s : uidPkg.getTactics()) {
////    				Tactic.TacticDataStruct t = Tactic.TacticDataStruct.fromString(s);
////    				if(t.status.getState().equals(TacticStatus.TacticState.ACTIVE)) {
////    					if(!firstTactic) builder.append(", ");
////    					builder.append(t.name);
////    					firstTactic = false;
////    				}
//    			}
//    		}
			return builder.toString();    		
    	} else if (event.getTopObject() instanceof MilStd2525TacticalSymbol) {
    		// object is not an agent, but is a Milstd2525 symbol, so likely this is an AprilTag
    		StringBuilder builder = new StringBuilder();
    		MilStd2525TacticalSymbol symbol = (MilStd2525TacticalSymbol) event.getTopObject();
    		builder.append(symbol.getModifier(SymbologyConstants.ADDITIONAL_INFORMATION).toString() + System.lineSeparator());
    		builder.append("Id: " + symbol.getModifier(SymbologyConstants.UNIQUE_DESIGNATION).toString());
    		return builder.toString();
    	}
    	return event.getTopObject() != null && event.getTopObject() instanceof AVList ?
    			((AVList) event.getTopObject()).getStringValue(this.rolloverKey) : null;
    }

    @Override
	public void selected(SelectEvent event) {
        try {
            if (event.isRollover() && this.rolloverKey != null)
                this.handleRollover(event);
            else if (event.isHover() && this.hoverKey != null)
                this.handleHover(event);
        } catch (Exception e) {
            // Wrap the handler in a try/catch to keep exceptions from bubbling up
            Logging.logger().warning(e.getMessage() != null ? e.getMessage() : e.toString());
        }
    }

    protected void handleRollover(SelectEvent event) {
        if (this.lastRolloverObject != null && this.lastRolloverObject != event.getTopObject()) {
            this.hideToolTip();
            hideAgentPath();
            this.lastRolloverObject = null;
            this.wwd.redraw();
        }

        String rolloverText = getRolloverText(event);
        if (rolloverText != null) {
            this.lastRolloverObject = event.getTopObject();
            this.showToolTip(event, rolloverText.replace("\\n", "\n"));
            this.wwd.redraw();
        }
        
        if (getAgentTelemPackageFromSelectEvent(event) != null) {
            this.lastRolloverObject = event.getTopObject();
            showAgentPath(event);
            this.wwd.redraw();	
        }
    }

    protected void handleHover(SelectEvent event) {
        if (this.lastHoverObject != null) {
            if (this.lastHoverObject == event.getTopObject())
                return;

            this.hideToolTip();
            this.lastHoverObject = null;
            this.wwd.redraw();
        }

        if (getHoverText(event) != null) {
            this.lastHoverObject = event.getTopObject();
            this.showToolTip(event, getHoverText(event).replace("\\n", "\n"));
            this.wwd.redraw();
        }
    }

    private void showAgentPath(SelectEvent event) {
        if(WorldWindVisualization.showAgentPathOnHover && event.getTopObject() != null && event.getTopObject() instanceof MilStd2525TacticalSymbol) {
            String uid = WorldWindAppPanel.getUidFromSymbol((MilStd2525TacticalSymbol) event.getTopObject());
            WorldWindAppFrame.getInstance().getVisualization().showAgentPath(uid);
            lastPathAgentUid = uid;
        }
    }

    private void hideAgentPath() {
        if(lastRolloverObject instanceof MilStd2525TacticalSymbol && lastPathAgentUid != null) {
            WorldWindAppFrame.getInstance().getVisualization().hideAgentPath(lastPathAgentUid);
            lastPathAgentUid = null;
        }
    }

    protected void showToolTip(SelectEvent event, String text) {
        if (annotation != null) {
            annotation.setText(text);
            annotation.setScreenPoint(event.getPickPoint());
        } else {
            annotation = new ToolTipAnnotation(text);
        }

        if (layer == null) {
            layer = new AnnotationLayer();
            layer.setPickEnabled(false);
        }

        layer.removeAllAnnotations();
        layer.addAnnotation(annotation);
        this.addLayer(layer);
    }

    protected void hideToolTip() {
        if (this.layer != null) {
            this.layer.removeAllAnnotations();
            this.removeLayer(this.layer);
            this.layer.dispose();
            this.layer = null;
        }

        if (this.annotation != null) {
            this.annotation.dispose();
            this.annotation = null;
        }
    }

    protected void addLayer(Layer newLayer) {
        if (!this.wwd.getModel().getLayers().contains(newLayer))
            WorldWindVisualization.insertBeforeCompass(this.wwd, newLayer);
    }

    protected void removeLayer(Layer layerToRemove) {
        this.wwd.getModel().getLayers().remove(layerToRemove);
    }
}
