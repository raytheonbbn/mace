//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.awt.AWTInputHandler;
import gov.nasa.worldwind.event.InputHandler;
import gov.nasa.worldwind.geom.Position;
import org.apache.log4j.Logger;

import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

public class MapClickListener {
    private static final Logger logger = Logger.getLogger(MapClickListener.class.getName());

    private final WorldWindow wwd;
    private int modifierBitmask;
    private OnMapClick onMapClickCallback = null;

    public interface OnMapClick {
        void onMapClick(int x, int y, Position position, MouseEvent e);
    }

    public MapClickListener setOnMapClickCallback(OnMapClick callback) {
        this.onMapClickCallback = callback;
        return this;
    }

    /**
     * Override me if you want a different filter behavior
     * @param e
     * @return
     */
    public boolean shouldPreventCallbackFromFiring(MouseEvent e) {
        if(e.getModifiers() != modifierBitmask)  // TODO: check the bitmask rather than an equals call
            return true;

//        PickedObjectList objectsAtCurrentPosition = wwd.getObjectsAtCurrentPosition();
//        logger.trace("Objects intersecting: " + objectsAtCurrentPosition);
//        if(objectsAtCurrentPosition.size() > 1) return true;

        return false;
    }

    public void setModifierBitmask(int modifierBitmask) {
        this.modifierBitmask = modifierBitmask;
    }

    public MapClickListener(WorldWindow wwd, int modifierBitmask) {

        this.wwd = wwd;
        this.modifierBitmask = modifierBitmask;

        ((Component) this.wwd).addMouseListener(new MouseListener() {
            @Override
            public void mouseClicked(MouseEvent e) {

                if(e != null) {

                    if(shouldPreventCallbackFromFiring(e))
                        return;

                    int x = e.getX(), y = e.getY();
                    Position position = wwd.getView().computePositionFromScreenPoint(x, y);
                    logger.trace("Mouse click at " + position + " w " + e.getModifiers());

                    if(onMapClickCallback != null) {
                        onMapClickCallback.onMapClick(x, y, position, e);
                    }
                }
            }

            @Override
            public void mousePressed(MouseEvent e) {

            }

            @Override
            public void mouseReleased(MouseEvent e) {

            }

            @Override
            public void mouseEntered(MouseEvent e) {

            }

            @Override
            public void mouseExited(MouseEvent e) {

            }
        });

    }

}
