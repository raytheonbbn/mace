//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tactics;

import com.bbn.ccast.nullsim.SimVehicle;
import com.bbn.ccast.visualizer.WorldWindAppFrame;
import com.bbn.ccast.visualizer.tracks.SDOTrack;

import java.awt.*;
import java.util.EnumSet;
import java.util.Map;

public class StopTacticsDialog extends TacticsDialog{
    public StopTacticsDialog(WorldWindAppFrame parent, Dialog.ModalityType modalityType,
                             EditingActivity editingActivity){
        super(parent, "Stop", modalityType, editingActivity, "Stop",
                EnumSet.noneOf(DataSources.class),
                EnumSet.noneOf(OnCompletionActivities.class));

    }

    private void execute(String[] agentIDs){
        if (!worldWindAppFrame.getVisualization().getConfiguration().isInSim()){
            return;
        }
        Map<String, SimVehicle> vehicleMap = worldWindAppFrame.getVisualization().getNullSim().getAllVehicles();
        for (String uid : agentIDs) {
            SimVehicle vehicle = vehicleMap.get(uid);
            if (vehicle != null){
                vehicle.setTarget(vehicle.getPosition());
            }
        }
    }

    @Override
    protected boolean executeCommand(String[] agentIDs) {
        execute(agentIDs);
        return true;
    }

    @Override
    protected boolean executeCommand(String[] agentIDs, SDOTrack sdoPositionTrack) {
        execute(agentIDs);
        return true;
    }
}
