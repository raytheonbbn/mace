//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tactics;

import com.bbn.ccast.nullsim.SimVehicle;
import com.bbn.ccast.util.LatLonAlt;
import com.bbn.ccast.visualizer.WorldWindAppFrame;
import com.bbn.ccast.visualizer.tracks.PositionType;
import com.bbn.ccast.visualizer.tracks.SDOPosition;
import com.bbn.ccast.visualizer.tracks.SDOTrack;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import org.apache.log4j.Logger;

import java.awt.*;
import java.util.EnumSet;
import java.util.Map;

public class LookAtTacticsDialog extends TacticsDialog{

    private static final Logger logger = Logger.getLogger(LookAtTacticsDialog.class.getName());

    public LookAtTacticsDialog(WorldWindAppFrame parent, Dialog.ModalityType modalityType,
                               EditingActivity editingActivity){
        super(parent, "LookAt", modalityType, editingActivity, "LookAt",
                EnumSet.of(DataSources.TRACKS, DataSources.POSITIONS),
                EnumSet.noneOf(OnCompletionActivities.class));

    }


    @Override
    protected boolean executeCommand(String[] agentIDs) {
        return false;
    }

    @Override
    protected boolean executeCommand(String[] agentIDs, SDOTrack sdoPositionTrack) {
        if (!worldWindAppFrame.getVisualization().getConfiguration().isInSim()){
            return false;
        }
        Map<String, SimVehicle> vehicleMap = worldWindAppFrame.getVisualization().getNullSim().getAllVehicles();
        java.util.List<SDOPosition> positions = sdoPositionTrack.getPositions();
        if (positions.size() == 0){
            return false;
        }

        if (this.onCompletionActivities.contains(OnCompletionActivities.RTL) && this.rtlButton.isSelected()) {
            SDOPosition last = positions.get(positions.size()-1);
            SDOPosition rtl = new SDOPosition(PositionType.RTL, last.getLatitude(), last.getLongitude(), last.getAltitude(),
                    last.getDuration(), last.getRadius());
            positions.add(rtl);
        } else if (this.onCompletionActivities.contains(OnCompletionActivities.LAND) && this.landButton.isSelected()) {
            SDOPosition last = positions.get(positions.size()-1);
            SDOPosition land = new SDOPosition(PositionType.LAND, last.getLatitude(), last.getLongitude(), last.getAltitude(),
                    last.getDuration(), last.getRadius());
            positions.add(land);
        }

        if (positions.size() < 1){
            logger.info("LookAt requires 1 position.");
            return false;
        }

        SDOPosition pos = positions.get(0);
        Position lookPos = new Position(Angle.fromDegrees(pos.getLatitudeDegrees()),
                Angle.fromDegrees(pos.getLongitudeDegrees()),
                pos.getAltitude());


        for (String uid : agentIDs) {
            SimVehicle vehicle = vehicleMap.get(uid);
            if (vehicle != null){
                if (!vehicle.isArmed()){
                    vehicle.armDisarm(true);
                }
                LatLonAlt curLla = vehicle.getPosition();
                Position curPos = new Position(Angle.fromDegrees(curLla.getLat()), Angle.fromDegrees(curLla.getLon()), curLla.getAlt());
                double targetHeading = LatLon.greatCircleAzimuth(curPos, lookPos).getDegrees();
                if (targetHeading < 0) {
                    targetHeading += 360;
                }
                vehicle.setAbsoluteYawTarget((float) targetHeading);
            }
        }
        return true;
    }
}
