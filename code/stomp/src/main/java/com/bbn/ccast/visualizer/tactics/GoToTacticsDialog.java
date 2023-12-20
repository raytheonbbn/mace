//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tactics;

import com.bbn.ccast.nullsim.SimVehicle;
import com.bbn.ccast.util.LatLonAlt;
import com.bbn.ccast.visualizer.WorldWindAppFrame;
import com.bbn.ccast.visualizer.tracks.PositionType;
import com.bbn.ccast.visualizer.tracks.SDOPosition;
import com.bbn.ccast.visualizer.tracks.SDOTrack;

import javax.swing.*;
import java.awt.*;
import java.util.EnumSet;
import java.util.Map;

public class GoToTacticsDialog extends TacticsDialog {
    public GoToTacticsDialog(WorldWindAppFrame parent, Dialog.ModalityType modalityType,
                             EditingActivity editingActivity) {
        super(parent, "GoTo", modalityType, editingActivity, "GoTo",
                EnumSet.of(DataSources.POSITIONS),
                EnumSet.of(OnCompletionActivities.LOITER, OnCompletionActivities.LAND,
                        OnCompletionActivities.RTL));

    }


    @Override
    protected boolean executeCommand(String[] agentIDs) {
        return false;
    }

    @Override
    protected boolean executeCommand(String[] agentIDs, SDOTrack sdoPositionTrack) {
        // If the user hasn't selected any payloads, warn them.
        if (agentIDs.length == 0){
            JOptionPane.showMessageDialog(worldWindAppFrame, "No payloads selected!", "Warning",
                    JOptionPane.ERROR_MESSAGE);
            return false;
        }

        if (!worldWindAppFrame.getVisualization().getConfiguration().isInSim()){
            return false;
        }

        Map<String, SimVehicle> vehicleMap =
                worldWindAppFrame.getVisualization().getNullSim().getAllVehicles();
        java.util.List<SDOPosition> positions = sdoPositionTrack.getPositions();
        if (positions.size() == 0) {
            return false;
        }

        if (this.onCompletionActivities.contains(OnCompletionActivities.RTL) && this.rtlButton.isSelected()) {
            SDOPosition last = positions.get(positions.size() - 1);
            SDOPosition rtl = new SDOPosition(PositionType.RTL, last.getLatitude(), last.getLongitude(),
                    last.getAltitude(),
                    last.getDuration(), last.getRadius());
            positions.add(rtl);
        } else if (this.onCompletionActivities.contains(OnCompletionActivities.LAND) && this.landButton.isSelected()) {
            SDOPosition last = positions.get(positions.size() - 1);
            SDOPosition land = new SDOPosition(PositionType.LAND, last.getLatitude(), last.getLongitude(),
                    last.getAltitude(),
                    last.getDuration(), last.getRadius());
            positions.add(land);
        }

        for (String uid : agentIDs) {
            SimVehicle vehicle = vehicleMap.get(uid);
            if (vehicle != null) {
                if (!vehicle.isArmed()) {
                    vehicle.armDisarm(true);
                }
                vehicle.setTargetList(positions);
            }
        }
        return true;
    }
}
