//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */
package com.bbn.ccast.visualizer.tracks;

import com.bbn.ccast.Utils;
import com.bbn.ccast.visualizer.WorldWindVisualization;
import com.bbn.ccast.visualizer.util.SDOListPanel;
import com.bbn.ccast.visualizer.util.SDOPanel;
import gov.nasa.worldwind.WorldWind;
import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.event.SelectEvent;
import gov.nasa.worldwind.event.SelectListener;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.pick.PickedObject;
import gov.nasa.worldwind.render.*;
import gov.nasa.worldwind.render.airspaces.AirspaceAttributes;
import gov.nasa.worldwind.render.airspaces.BasicAirspaceAttributes;
import gov.nasa.worldwind.render.airspaces.SphereAirspace;
import gov.nasa.worldwind.util.WWUtil;
import org.jboss.netty.util.ThreadRenamingRunnable;

import javax.vecmath.Vector3d;
import java.awt.*;
import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.util.List;
import java.util.*;
import java.util.stream.StreamSupport;

/**
 * @author tag
 * @version $Id: TrackController.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class TrackController {
    public static final double LOOKPOINT_FRUSTUM_HFOV = 54.0;
    public static final double LOOKPOINT_FRUSTUM_VFOV = 46.0;
    public static final double LOOKPOINT_FRUSTUM_MAX_DISTANCE = 10.0; //meters

    public static final String TRACK_ADD = "TrackController.TrackAdded";
    public static final String TRACK_CURRENT = "TrackController.TrackCurrent";
    public static final String TRACK_DIRTY_BIT = "TrackController.TrackDirtyBit";
    public static final String TRACK_DISABLE = "TrackController.TrackDisabled";
    public static final String TRACK_ENABLE = "TrackController.TrackEnabled";
    public static final String TRACK_UNORDERED = "TrackController.TrackUnordered";
    public static final String TRACK_ORDERED = "TrackController.TrackOrdered";
    public static final String TRACK_MODIFY = "TrackController.TrackModified";
    public static final String TRACK_NAME = "TrackController.TrackName";
    public static final String TRACK_UID = "TrackController.TrackUid";
    public static final String TRACK_OFFSET = "TrackController.TrackOffset";
    public static final String TRACK_REMOVE = "TrackController.TrackRemoved";

    public static final String BEGIN_TRACK_POINT_ENTRY = "TrackController.BeginTrackPointEntry";
    public static final String END_TRACK_POINT_ENTRY = "TrackController.EndTrackPointEntry";
    public static final String MOVE_TO_NEXT_POINT = "TrackController.MoveToNextPoint";
    public static final String REMOVE_LAST_POINT = "TrackController.RemoveLastPoint";

    public static final String EXTENSION_PLANE = "TrackController.ExtensionPlane";
    public static final String EXTENSION_CURSOR_GROUND = "TrackController.ExtensionMouseGround";
    public static final String EXTENSION_CURSOR_AIR = "TrackController.ExtensionMouseAir";

    private static ShapeAttributes TRACK_AIR_SHAPE_ATTRIBUTES = null;

    static {
        TRACK_AIR_SHAPE_ATTRIBUTES = new BasicShapeAttributes();
    }

    private static ShapeAttributes TRACK_GROUND_SHAPE_ATTRIBUTES = null;

    static {
        TRACK_GROUND_SHAPE_ATTRIBUTES = new BasicShapeAttributes();
        TRACK_GROUND_SHAPE_ATTRIBUTES.setOutlineStippleFactor(5);
        TRACK_GROUND_SHAPE_ATTRIBUTES.setOutlineStipplePattern((short) 0xAAAA);
    }

    public static double SPHERE_AIRSPACE_RADIUS = 0.5;

    private WorldWindow wwd;
    private WorldWindVisualization wwv;
    private boolean visualizeTrackPlan = false;
    private SDOListPanel sdoListPanel;
    private SDOPanel sdoPanel;
    private HashMap<String, List<Renderable>> trackRenderables = new HashMap<String, List<Renderable>>();
    private HashMap<String, SDOTrack> trackNameMap = new HashMap<>();
    private boolean isRoute = true;

    private final SelectListener selectListener = event -> {
        if (event == null)
            return;

        onSelected(event);
    };

    public TrackController(WorldWindVisualization visualization) {
        this.wwv = visualization;
        this.visualizeTrackPlan = this.wwv.getConfiguration().visualizeTrackPlan();
    }

    public WorldWindow getWwd() {
        return wwd;
    }

    public void setWwd(WorldWindow wwd) {
        if (wwd == this.wwd)
            return;

        if (this.wwd != null) {
            this.wwd.removeSelectListener(this.selectListener);
        }

        this.wwd = wwd;

        if (this.wwd != null) {
            this.wwd.addSelectListener(this.selectListener);
        }

    }

    public boolean getIsRoute() {
        return isRoute;
    }

    public void setIsRoute(boolean isRoute) {
        this.isRoute = isRoute;
    }

    public SDOListPanel getSDOListPanel() {
        return sdoListPanel;
    }

    public void setSDOListPanel(SDOListPanel tracksPanel) {
        this.sdoListPanel = tracksPanel;
    }

    public SDOPanel getSDOPanel() {
        return sdoPanel;
    }

    public void setSDOPanel(SDOPanel sdoPanel) {
        this.sdoPanel = sdoPanel;
    }

    public void addTrack(SDOTrack track) {
        addTrack(track, true);
    }

    public void addTrack(SDOTrack track, boolean modifySDOPanel) {
        if (track == null)
            return;

        if (trackNameMap.containsKey(track.getUid())) {
            return;
        }
        trackNameMap.put(track.getUid(), track);

        if (isRoute) {
            this.createPolylineTrackRepresentation(track);
        } else {
            this.createSphereTrackRepresentation(track);
        }

        track.addPropertyChangeListener(propertyChangeEvent -> {
            if (propertyChangeEvent.getPropertyName() == TrackController.TRACK_REMOVE)
                removeTrack((SDOTrack) propertyChangeEvent.getSource());
            else if (propertyChangeEvent.getPropertyName() == TrackController.TRACK_MODIFY)
                updateTrack((SDOTrack) propertyChangeEvent.getSource());
            else if (propertyChangeEvent.getPropertyName() == TrackController.TRACK_ENABLE)
                enableTrack((SDOTrack) propertyChangeEvent.getSource());
            else if (propertyChangeEvent.getPropertyName() == TrackController.TRACK_DISABLE)
                disableTrack((SDOTrack) propertyChangeEvent.getSource());
            else if (propertyChangeEvent.getPropertyName() == TrackController.TRACK_UNORDERED)
                unorderedTrack((SDOTrack) propertyChangeEvent.getSource());
            else if (propertyChangeEvent.getPropertyName() == TrackController.TRACK_ORDERED)
                orderedTrack((SDOTrack) propertyChangeEvent.getSource());
            else if (propertyChangeEvent.getPropertyName() == TrackController.TRACK_CURRENT)
                trackCurrent((SDOTrack) propertyChangeEvent.getSource());
            else if (propertyChangeEvent.getPropertyName() == TrackController.TRACK_NAME)
                trackName((SDOTrack) propertyChangeEvent.getSource());
            else if (propertyChangeEvent.getPropertyName() == TrackController.TRACK_UID)
                trackUid((SDOTrack) propertyChangeEvent.getSource());
            else if (propertyChangeEvent.getPropertyName() == TrackController.TRACK_DIRTY_BIT)
                trackDirtyBit((SDOTrack) propertyChangeEvent.getSource());
        });

        if (modifySDOPanel) {
            this.sdoListPanel.addTrack(track);
            this.sdoPanel.setTrack(track);
        }

        // this.moveToTrack(track);
    }


    public void appendPositionTypeToCurrentTrack(PositionType positionType, Position position) {
        SDOTrack currentTrack = getCurrentTrack();
        if (currentTrack != null) {
            SDOPosition sdoPos = new SDOPosition(positionType, position.getLatitude(),
                    position.getLongitude(),
                    position.getElevation(), 0.0, 0.0);
            currentTrack.appendPosition(sdoPos);
        }
    }


    public void insertPositionTypeToCurrentTrack(PositionType positionType, Position position,
                                                 boolean isAbove) {
        SDOTrack currentTrack = getCurrentTrack();
        if (currentTrack != null) {
            SDOPosition sdoPos = new SDOPosition(positionType, position.getLatitude(),
                    position.getLongitude(),
                    position.getElevation(), 0.0, 0.0);

            int index = this.getSDOPanel().getPositionTable().getSelectionModel().getMinSelectionIndex();
            if (!isAbove) {
                index = this.getSDOPanel().getPositionTable().getSelectionModel().getMaxSelectionIndex() + 1;
            }

            if (index < 0) {
                currentTrack.appendPosition(sdoPos);
            } else {
                currentTrack.add(index, sdoPos);
            }
        }
    }

    public void insertLookAtToCurrentTrack(Position position, boolean isAbove) {
        SDOTrack currentTrack = getCurrentTrack();
        if (currentTrack != null) {
            SDOPosition sdoPos = new SDOPosition(PositionType.LOOKPOINT, position.getLatitude(),
                    position.getLongitude(), position.getElevation(), 0.0, 0.0);

            int index = this.getSDOPanel().getPositionTable().getSelectionModel().getMinSelectionIndex();
            if (!isAbove) {
                index = this.getSDOPanel().getPositionTable().getSelectionModel().getMaxSelectionIndex() + 1;
            }

            if (index < 0) {
                currentTrack.appendPosition(sdoPos);
            } else {
                currentTrack.add(index, sdoPos);
            }
        }
    }

    public void insertLoiterToCurrentTrack(Position position, boolean isAbove) {
        SDOTrack currentTrack = getCurrentTrack();
        if (currentTrack != null) {
            SDOPosition sdoPos = new SDOPosition(PositionType.LOITER, position.getLatitude(),
                    position.getLongitude(), position.getElevation(), 0.0, 0.0);

            int index = this.getSDOPanel().getPositionTable().getSelectionModel().getMinSelectionIndex();
            if (!isAbove) {
                index = this.getSDOPanel().getPositionTable().getSelectionModel().getMaxSelectionIndex() + 1;
            }

            if (index < 0) {
                currentTrack.appendPosition(sdoPos);
            } else {
                currentTrack.add(index, sdoPos);
            }
        }
    }


    public void insertLandToCurrentTrack(Position position, boolean isAbove) {
        SDOTrack currentTrack = getCurrentTrack();
        if (currentTrack != null) {
            SDOPosition sdoPos = new SDOPosition(PositionType.LAND, position.getLatitude(),
                    position.getLongitude(), position.getElevation(), 0.0, 0.0);

            int index = this.getSDOPanel().getPositionTable().getSelectionModel().getMinSelectionIndex();
            if (!isAbove) {
                index = this.getSDOPanel().getPositionTable().getSelectionModel().getMaxSelectionIndex() + 1;
            }

            if (index < 0) {
                currentTrack.appendPosition(sdoPos);
            } else {
                currentTrack.add(index, sdoPos);
            }
        }
    }


    public SDOTrack getCurrentTrack() {
        return this.sdoListPanel.getCurrentTrack();
    }

    public void refreshCurrentTrack() {
        trackCurrent(getCurrentTrack());
    }

    public static AirspaceAttributes getRoutePositionAttributes(SDOTrack track) {
        AirspaceAttributes attributes = new BasicAirspaceAttributes();
        attributes.setInteriorMaterial(
                new Material(track.getColor(), track.getColor(), track.getColor(), track.getColor(), 0.0f));
        attributes.setOutlineMaterial(
                new Material(track.getColor(), track.getColor(), track.getColor(), track.getColor(), 0.0f));
        attributes.setDrawOutline(true);
        attributes.setInteriorOpacity(0.95);
        attributes.setOutlineOpacity(.95);
        attributes.setOutlineWidth(2);
        return attributes;
    }

    public static AirspaceAttributes getLookPointAttributes(SDOTrack track) {
        AirspaceAttributes attributes = new BasicAirspaceAttributes();
        attributes.setInteriorMaterial(
                new Material(Color.DARK_GRAY, Color.DARK_GRAY, Color.DARK_GRAY, track.getColor(), 0.0f));
        attributes.setOutlineMaterial(
                new Material(Color.DARK_GRAY, Color.DARK_GRAY, Color.DARK_GRAY, track.getColor(), 0.0f));
        attributes.setDrawOutline(true);
        attributes.setInteriorOpacity(0.95);
        attributes.setOutlineOpacity(.95);
        attributes.setOutlineWidth(2);
        return attributes;
    }

    public static List<Position> createFrustumPointPath(List<Vector3d> frustumPoints,
                                                        Position goalTargetPosition) {
        //double alt = frustumPoints.get(0).getZ() >= 0.0 ? frustumPoints.get(0).getZ() : 0.0;
        double alt = frustumPoints.get(0).getZ();
        Position lr = Position.fromDegrees(frustumPoints.get(0).getX(), frustumPoints.get(0).getY(), alt);
        alt = frustumPoints.get(1).getZ();
        Position ll = Position.fromDegrees(frustumPoints.get(1).getX(), frustumPoints.get(1).getY(), alt);
        alt = frustumPoints.get(2).getZ();
        Position ur = Position.fromDegrees(frustumPoints.get(2).getX(), frustumPoints.get(2).getY(), alt);
        alt = frustumPoints.get(3).getZ();
        Position ul = Position.fromDegrees(frustumPoints.get(3).getX(), frustumPoints.get(3).getY(), alt);

        ArrayList<Position> retList = new ArrayList<Position>();

        retList.add(ul);
        retList.add(ur);
        retList.add(lr);
        retList.add(ll);
        retList.add(ul);
        retList.add(goalTargetPosition);
        retList.add(ll);
        retList.add(lr);
        retList.add(goalTargetPosition);
        retList.add(ur);

        return retList;
    }


    private void createPolylineTrackRepresentation(SDOTrack track) {
        if (!visualizeTrackPlan) {
            return;
        }

        List<Renderable> renderables = new ArrayList<Renderable>();

        List<Position> positions = createPositionListFromSDOTrack(track);

        if (WorldWindVisualization.areAllPositionsZeroAltitude(positions)) {
            SurfacePolyline path = new SurfacePolyline(positions);
            path.setPathType(AVKey.RHUMB_LINE);
            AirspaceAttributes attr = getRoutePositionAttributes(track);
            path.setAttributes(attr);
            if (track.isUnordered()) {
                path.setVisible(false);
            }
            renderables.add(path);
            wwv.addVisualization(path, WorldWindVisualization.MISSION_PLAN_LAYER_NAME);

        } else {
            Path airPath = new Path(positions);
            airPath.setPathType(AVKey.RHUMB_LINE);
            airPath.setAltitudeMode(WorldWind.RELATIVE_TO_GROUND);
            Color[] colors = new Color[positions.size()];
            Arrays.fill(colors, track.getColor());
            airPath.setPositionColors(new SimplePositionColors(colors, positions.size()));
            airPath.setAttributes(TRACK_AIR_SHAPE_ATTRIBUTES);
            //airPath.setShowPositions(true);
            //airPath.setShowPositionsScale(10.5);

            Path groundPath = new Path(positions);
            groundPath.setPathType(AVKey.RHUMB_LINE);
            groundPath.setAltitudeMode(WorldWind.CLAMP_TO_GROUND);
            groundPath.setPositionColors(new SimplePositionColors(colors, positions.size()));
            groundPath.setFollowTerrain(true);
            groundPath.setAttributes(TRACK_GROUND_SHAPE_ATTRIBUTES);
            if (track.isUnordered()) {
                airPath.setVisible(false);
                groundPath.setVisible(false);
            }
            renderables.add(airPath);
            wwv.addVisualization(airPath, WorldWindVisualization.MISSION_PLAN_LAYER_NAME);
            renderables.add(groundPath);
            wwv.addVisualization(groundPath, WorldWindVisualization.MISSION_PLAN_LAYER_NAME);
        }

        SDOPosition lastGoToPos = null;
        for (SDOPosition sdoPos : track.getPositions()) {
            if (sdoPos.getPositionType() == PositionType.GOTO) {
                lastGoToPos = sdoPos;
            }

            AirspaceAttributes attr = getRoutePositionAttributes(track);
            if (sdoPos.getPositionType() == PositionType.LOOKPOINT) {
                attr = getLookPointAttributes(track);

                if (lastGoToPos != null) {
                    Vector3d lookPoint = new Vector3d(sdoPos.getLatitudeDegrees(),
                            sdoPos.getLongitudeDegrees(),
                            sdoPos.getAltitude());
                    Vector3d goalTargetPoint = new Vector3d(lastGoToPos.getLatitudeDegrees(),
                            lastGoToPos.getLongitudeDegrees(), lastGoToPos.getAltitude());
                    ArrayList<Vector3d> fpl = Utils.getFrustumPoints(goalTargetPoint, lookPoint,
                            LOOKPOINT_FRUSTUM_HFOV,
                            LOOKPOINT_FRUSTUM_VFOV, true, LOOKPOINT_FRUSTUM_MAX_DISTANCE, false);
                    List<Position> frustumPointList = createFrustumPointPath(fpl, Position
                            .fromDegrees(goalTargetPoint.getX(), goalTargetPoint.getY(),
                                    goalTargetPoint.getZ()));

                    Path frustum = new LookFrustumPath(frustumPointList);
                    frustum.setPathType(AVKey.RHUMB_LINE);
                    frustum.setAltitudeMode(WorldWind.RELATIVE_TO_GROUND);
                    Color[] colors = new Color[positions.size()];
                    Arrays.fill(colors, track.getColor());
                    frustum.setPositionColors(new SimplePositionColors(colors, positions.size()));
                    frustum.setAttributes(attr);
                    renderables.add(frustum);
                    wwv.addVisualization(frustum, WorldWindVisualization.MISSION_PLAN_LAYER_NAME);
                }
            } else {
                // Create sphere at goto track position
                SphereAirspace sphere = new SphereAirspace(
                        new Position(sdoPos.getLatitude(), sdoPos.getLongitude(), sdoPos.getElevation()),
                        TrackController.SPHERE_AIRSPACE_RADIUS);
                sphere.setAttributes(getRoutePositionAttributes(track));
                sphere.setAltitudeDatum(AVKey.ABOVE_GROUND_LEVEL, AVKey.ABOVE_GROUND_LEVEL);
                sphere.setAltitude(sdoPos.getElevation());
                renderables.add(sphere);
                wwv.addVisualization(sphere, WorldWindVisualization.MISSION_PLAN_LAYER_NAME);
            }
        }

        if (this.wwd != null) {
            this.wwd.redraw();
        }

        Point mouse = MouseInfo.getPointerInfo().getLocation();

        PositionSizeThread posSize = new PositionSizeThread(wwv, wwd, wwd.getView().computePositionFromScreenPoint(mouse.getX(), mouse.getY()));
        Thread updateSizeThread = new Thread(posSize);
        updateSizeThread.start();
        

        this.trackRenderables.put(track.getUid(), renderables);
    }

    private void createSphereTrackRepresentation(SDOTrack track) {
        if (!visualizeTrackPlan) {
            return;
        }

        List<Renderable> renderables = new ArrayList<Renderable>();
		/*List<Position> positions = createPositionListFromSDOTrack(track);

		Path airPath = new Path(positions);
		airPath.setPathType(AVKey.SHAPE_CIRCLE);
		airPath.setAltitudeMode(WorldWind.CLAMP_TO_GROUND);
		Color[] colors = new Color[positions.size()];
		Arrays.fill(colors, track.getColor());
		airPath.setPositionColors(new SimplePositionColors(colors, positions.size()));
		airPath.setAttributes(getRoutePositionAttributes(track));
		airPath.setShowPositions(true);
		airPath.setShowPositionsScale(10.5);
		airPath.setDrawVerticals(false);
		airPath.setExtrude(false);

		renderables.add(airPath);
		wwv.addVisualization(airPath, WorldWindVisualization.MISSION_PLAN_LAYER_NAME);*/

        for (SDOPosition sdoPos : track.getPositions()) {
            SphereAirspace sphere = new SphereAirspace(
                    new Position(sdoPos.getLatitude(), sdoPos.getLongitude(), sdoPos.getElevation()), 2);
            sphere.setAttributes(getRoutePositionAttributes(track));
            sphere.setAltitudeDatum(AVKey.ABOVE_GROUND_LEVEL, AVKey.ABOVE_GROUND_LEVEL);
            sphere.setAltitude(sdoPos.getElevation());

            renderables.add(sphere);
            wwv.addVisualization(sphere, WorldWindVisualization.MISSION_PLAN_LAYER_NAME);
        }

        if (this.wwd != null) {
            this.wwd.redraw();
        }

        Point mouse = MouseInfo.getPointerInfo().getLocation();

        PositionSizeThread posSize = new PositionSizeThread(wwv, wwd, wwd.getView().computePositionFromScreenPoint(mouse.getX(), mouse.getY()));
        Thread updateSizeThread = new Thread(posSize);
        updateSizeThread.start();

        this.trackRenderables.put(track.getUid(), renderables);
    }

    protected List<Position> createPositionListFromSDOTrack(SDOTrack track) {
        List<Position> positions = new ArrayList<Position>();
        for (SDOPosition sdoPos : track.getPositions()) {
            if (sdoPos.getPositionType() == PositionType.GOTO) {
                positions.add(new Position(sdoPos.getLatitude(), sdoPos.getLongitude(),
                        sdoPos.getElevation()));
            }
        }
        return positions;
    }

    public Set<SDOTrack> getTracks() {
        return new HashSet<SDOTrack>(this.trackNameMap.values());
    }

    public List<SDOTrack> getTracks(String uid) {
        List<SDOTrack> tracks = new ArrayList<SDOTrack>();
        SDOTrack sdo = this.trackNameMap.get(uid);
        if (sdo != null) {
            tracks.add(sdo);
        }
        return tracks;
    }


    private void removeTrack(SDOTrack track) {
        if (this.trackNameMap.containsKey(track.getUid())) {
            this.trackNameMap.remove(track.getUid());

            List<Renderable> renderables = this.trackRenderables.get(track.getUid());

            if (renderables == null) {
                return;
            }

            for (Renderable renderable : renderables) {
                wwv.removeVisualization(renderable, WorldWindVisualization.MISSION_PLAN_LAYER_NAME);
            }
            this.trackRenderables.remove(track.getUid());
        }

        // is this necessary?
        if (this.wwd != null)
            this.wwd.redraw();
    }

    private void enableTrack(SDOTrack track) {
        if (!this.trackRenderables.containsKey(track.getUid())) {
            if (isRoute) {
                createPolylineTrackRepresentation(track);
            } else {
                createSphereTrackRepresentation(track);
            }
        }
        // is this necessary?
        if (this.wwd != null) {
            this.wwd.redraw();
        }
    }

    private void disableTrack(SDOTrack track) {
        if (this.trackRenderables.containsKey(track.getUid())) {
            for (Renderable renderable : this.trackRenderables.get(track.getUid())) {
                wwv.removeVisualization(renderable, WorldWindVisualization.MISSION_PLAN_LAYER_NAME);
            }
            this.trackRenderables.remove(track.getUid());
        }

        // is this necessary?
        if (this.wwd != null) {
            this.wwd.redraw();
        }
    }

    private void unorderedTrack(SDOTrack track) {
        track.setUnordered(true);
        List<Renderable> renderables = this.trackRenderables.get(track.getUid());
        if (renderables != null) {
            for (Renderable renderable : renderables) {
                if (renderable instanceof Path && !(renderable instanceof LookFrustumPath)) {
                    ((Path) renderable).setVisible(false);
                }
                if (renderable instanceof SurfacePolyline) {
                    ((SurfacePolyline) renderable).setVisible(false);
                }
            }
        }

        // is this necessary?
        if (this.wwd != null) {
            this.wwd.redraw();
        }
    }

    private void orderedTrack(SDOTrack track) {
        track.setUnordered(false);
        List<Renderable> renderables = this.trackRenderables.get(track.getUid());
        if (renderables != null) {
            for (Renderable renderable : renderables) {
                if (renderable instanceof Path && !(renderable instanceof LookFrustumPath)) {
                    ((Path) renderable).setVisible(true);
                }
                if (renderable instanceof SurfacePolyline) {
                    ((SurfacePolyline) renderable).setVisible(true);
                }
            }
        }

        // is this necessary?
        if (this.wwd != null) {
            this.wwd.redraw();
        }
    }

    private void updateTrack(SDOTrack track) {
        this.removeTrack(track);

        this.trackNameMap.put(track.getUid(), track);

        if (isRoute) {
            this.createPolylineTrackRepresentation(track);
        } else {
            this.createSphereTrackRepresentation(track);
        }

        // is this necessary?
        if (this.wwd != null) {
            this.wwd.redraw();
        }
    }

    private void trackCurrent(SDOTrack track) {

        // Adjust track line width
        for (SDOTrack st : this.trackNameMap.values()) {
            if (st != track) {
                this.setTrackLayerLineWidth(st, 1);
                this.setSphereRadius(st, 1);
            }
        }
        this.setTrackLayerLineWidth(track, 2);
        this.setSphereRadius(track, 2);

        this.wwd.firePropertyChange(TRACK_CURRENT, null, track); // broadcast track change via wwd
    }

    private void trackName(@SuppressWarnings("unused") SDOTrack track) {
        // Intentionally left blank, as a placeholder for future functionality.
    }

    private void trackUid(@SuppressWarnings("unused") SDOTrack track) {
        // Intentionally left blank, as a placeholder for future functionality.
    }

    private void trackDirtyBit(@SuppressWarnings("unused") SDOTrack track) {
        // Intentionally left blank, as a placeholder for future functionality.
    }

    protected void onSelected(SelectEvent event) {
        SDOTrack track = this.getPickedTrack(event.getTopPickedObject());

        if (event.getEventAction().equals(SelectEvent.LEFT_CLICK)) {
            if (track != null)
                this.onTrackClicked(track);
        } else if (event.getEventAction().equals(SelectEvent.ROLLOVER)) {
            this.onTrackRollover(track);
        } else if (event.getEventAction().equals(SelectEvent.HOVER)) {
            this.onTrackHover(track);
        }
    }

    protected SDOTrack getPickedTrack(PickedObject pickedObject) {
        if (pickedObject == null)
            return null;

        Object obj = pickedObject.getObject();
        Set<String> trackGuids = this.trackRenderables.keySet();
        for (String guid : trackGuids) {
            List<Renderable> renderables = this.trackRenderables.get(guid);
            if (renderables.contains(obj)) {
                return this.trackNameMap.get(guid);
            }
        }

        return null;
    }

    protected void onTrackClicked(SDOTrack track) {
        this.sdoListPanel.setCurrentTrack(track);
    }

    protected void onTrackRollover(SDOTrack track) {

        for (SDOTrack st : this.trackNameMap.values()) {
            if (st != track) {
                this.setTrackLayerColor(st, st.getColor());
            }
        }

        if (track != null) {
            Color rolloverColor = WWUtil.makeColorDarker(track.getColor());
            this.setTrackLayerColor(track, rolloverColor);
        }
    }

    protected void onTrackHover(@SuppressWarnings("unused") SDOTrack track) {
        // TODO: show tool tip with track name
    }

    private void setTrackLayerColor(SDOTrack track, Color color) {

        if (this.trackRenderables.get(track.getUid()) == null) {
            return;
        }

        for (Renderable r : this.trackRenderables.get(track.getUid())) {
            if (r instanceof Path) {
                Path line = (Path) r;
                int count = (int) StreamSupport.stream(line.getPositions().spliterator(), false).count();
                Color[] colors = new Color[count];
                Arrays.fill(colors, color);
                line.setPositionColors(new SimplePositionColors(colors, count));
            } else if (r instanceof SphereAirspace) {
                SphereAirspace sphere = (SphereAirspace) r;
                AirspaceAttributes attr = sphere.getAttributes();
                attr.setInteriorMaterial(new Material(color, color, color, color, 0.0f));
                attr.setOutlineMaterial(new Material(color, color, color, color, 0.0f));
                sphere.setAttributes(attr);
            }
        }

        if (this.wwd != null)
            this.wwd.redraw();
    }

    private void setTrackLayerLineWidth(SDOTrack track, double width) {

        List<Renderable> renderables = this.trackRenderables.get(track.getUid());
        if (renderables != null && !renderables.isEmpty()) {
            for (Renderable r : renderables) {
                if (r instanceof Path) {
                    Path line = (Path) r;

                    ShapeAttributes sa = new BasicShapeAttributes(TRACK_AIR_SHAPE_ATTRIBUTES);
                    if (line.getAltitudeMode() == WorldWind.CLAMP_TO_GROUND) {
                        sa = new BasicShapeAttributes(TRACK_GROUND_SHAPE_ATTRIBUTES);
                    }
                    sa.setOutlineWidth(width);
                    line.setAttributes(sa);
                }
            }

            if (this.wwd != null) {
                this.wwd.redraw();
            }
        }
    }

    private void setSphereRadius(SDOTrack track, double radius) {
        List<Renderable> renderables = this.trackRenderables.get(track.getUid());
        if (renderables != null && !renderables.isEmpty()) {
            for (Renderable rr : renderables) {
                if (rr instanceof Path) {
                    Path sphere = (Path) rr;
                    ShapeAttributes sa = new BasicShapeAttributes(TRACK_AIR_SHAPE_ATTRIBUTES);
                    sa.setOutlineWidth(radius);
                    sphere.setAttributes(sa);
                }
            }

            if (this.wwd != null) {
                this.wwd.redraw();
            }
        }
    }

    public static class SimplePositionColors implements Path.PositionColors {
        protected Color[] colors;
        protected int pathLength;

        public SimplePositionColors(Color[] colors, int pathLength) {
            this.colors = colors;
            this.pathLength = pathLength;
        }

        @Override
        public Color getColor(Position position, int ordinal) {
            if (this.pathLength == 0) {
                return null;
            }
            int index = colors.length * ordinal / this.pathLength;
            if (index > this.colors.length - 1) {
                index = this.colors.length - 1;
            }
            return this.colors[index];
        }
    }

}
