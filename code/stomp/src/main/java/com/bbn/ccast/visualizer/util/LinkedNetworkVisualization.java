//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.TargetTelemPackage;
import com.bbn.ccast.visualizer.WorldWindVisualization;
import com.bbn.ccast.visualizer.tracks.TrackController;
import gov.nasa.worldwind.WorldWind;
import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.BasicShapeAttributes;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.render.ShapeAttributes;
import scala.concurrent.impl.FutureConvertersImpl;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.StreamSupport;

public class LinkedNetworkVisualization {
    private ConcurrentHashMap<String, TargetTelemPackage> telems;
    private Color color;
    private final String networkName;
    private final WorldWindVisualization wwv;
    private final WorldWindow wwd;
    private Path links;
    private final com.bbn.ccast.config.Configuration configuration;

    private static ShapeAttributes LINKS_SHAPE_ATTRIBUTES = null;

    static {
        LINKS_SHAPE_ATTRIBUTES = new BasicShapeAttributes();
        LINKS_SHAPE_ATTRIBUTES.setOutlineStippleFactor(5);
        LINKS_SHAPE_ATTRIBUTES.setOutlineStipplePattern((short) 0xAAAA);
        LINKS_SHAPE_ATTRIBUTES.setOutlineWidth(2);
    }


    public LinkedNetworkVisualization(String networkName,
                                      ConcurrentHashMap<String, TargetTelemPackage> targetTelems,
                                      Color color, WorldWindVisualization wwv, WorldWindow wwd) {
        this.networkName = networkName;
        this.telems = targetTelems;
        this.color = color;
        this.wwv = wwv;
        this.wwd = wwd;
        // Add the configuration file to check if white or blue force
        this.configuration = wwv.configuration;

        createPath();
    }

    public String getNetworkName() {
        return this.networkName;
    }

    public Color getColor() {
        return color;
    }

    private void createPath() {
        List<Position> positions = new ArrayList<>();
        List<Position> completed = new ArrayList<>();
        for (TargetTelemPackage telem : telems.values()) {
            Position pos = new Position(LatLon.fromDegrees(telem.getLatitude(), telem.getLongitude()),
                    telem.getAltitude());
            positions.add(pos);
            completed.add(pos);
            for (TargetTelemPackage telem2 : telems.values()) {
                if (telem != telem2) {
                    Position pos2 = new Position(LatLon.fromDegrees(telem2.getLatitude(), telem2.getLongitude()), telem2.getAltitude());
                    if (!completed.contains(pos2)) {
                        positions.add(pos2);
                        positions.add(pos);
                    }
                }
            }
        }

        Color[] colors = new Color[positions.size()];
        Arrays.fill(colors, this.color);
        links = new Path(positions);
        links.setPathType(AVKey.RHUMB_LINE);
        if (wwv.getConfiguration().isInSim()) {
            links.setAltitudeMode(WorldWind.CLAMP_TO_GROUND);
        } else {
            links.setAltitudeMode(WorldWind.ABSOLUTE);
        }
        links.setPositionColors(new TrackController.SimplePositionColors(colors, positions.size()));
        links.setFollowTerrain(true);
        links.setAttributes(LINKS_SHAPE_ATTRIBUTES);

        updateVisibility();
        wwv.addVisualization(links, WorldWindVisualization.MISSION_PLAN_LAYER_NAME);
    }

    public Color update(ConcurrentHashMap<String, TargetTelemPackage> targets, boolean checkTelems) {
        if (checkTelems) {
            ConcurrentHashMap<String, TargetTelemPackage> tempTelems = new ConcurrentHashMap<>();

            for (String uid : targets.keySet()) {
                if (!targets.get(uid).getNetworksCaptured().isEmpty()) {
                    if (targets.get(uid).getNetworksCaptured().containsKey(networkName)) {
                        tempTelems.put(uid, targets.get(uid));
                    }
                }
            }

            if (tempTelems.isEmpty()) {
                wwv.removeVisualization(links, WorldWindVisualization.MISSION_PLAN_LAYER_NAME);
                return color;
            }

            telems = tempTelems;
        }

        updatePosition();
        updateVisibility();
        return null;
    }

    public void updateVisibility() {
        for (TargetTelemPackage telem : telems.values()) {
            if (!telem.getDiscovered() && !this.configuration.getForceType().equals("white")) {
                links.setVisible(false);
                return;
            }
        }
        links.setVisible(true);
    }

    public void updatePosition() {
        List<Position> updatedPositions = new ArrayList<>();
        List<Position> completed = new ArrayList<>();
        for (TargetTelemPackage telem : telems.values()) {
            Position pos = new Position(LatLon.fromDegrees(telem.getLatitude(), telem.getLongitude()),
                    telem.getAltitude());
            updatedPositions.add(pos);
            completed.add(pos);
            for (TargetTelemPackage telem2 : telems.values()) {
                if (telem != telem2) {
                    Position pos2 = new Position(LatLon.fromDegrees(telem2.getLatitude(), telem2.getLongitude()), telem2.getAltitude());
                    if (!completed.contains(pos2)) {
                        updatedPositions.add(pos2);
                        updatedPositions.add(pos);
                    }
                }
            }
        }

        links.setPositions(updatedPositions);
        wwd.redraw();
    }

    public void setColor(Color color) {
        this.color = color;

        int count = (int) StreamSupport.stream(links.getPositions().spliterator(), false).count();
        Color[] colors = new Color[count];
        Arrays.fill(colors, this.color);
        links.setPositionColors(new TrackController.SimplePositionColors(colors, count));

        wwd.redraw();
    }
}
