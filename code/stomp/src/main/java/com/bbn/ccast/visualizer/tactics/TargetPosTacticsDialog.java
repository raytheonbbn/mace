//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tactics;

import com.bbn.ccast.visualizer.WorldWindAppFrame;
import com.bbn.ccast.visualizer.tracks.PositionSizeThread;
import com.bbn.ccast.visualizer.tracks.SDOPosition;
import com.bbn.ccast.visualizer.tracks.SDOTrack;

import com.bbn.ccast.visualizer.util.SDOPanel;
import gov.nasa.worldwind.geom.Position;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.ListIterator;

public class TargetPosTacticsDialog extends TacticsDialog {
    public TargetPosTacticsDialog(WorldWindAppFrame parent, Dialog.ModalityType modalityType,
                                  EditingActivity editingActivity) {
        super(parent, "Target Positions", modalityType, editingActivity, "Target Positions",
                EnumSet.of(DataSources.POSITIONS), EnumSet.noneOf(OnCompletionActivities.class), false);

        parent.getWwjPanel().setMapClickModifier(4);
        parent.getWwjPanel().setMapClickAction(false);
        parent.getTrackController().setIsRoute(false);

        setAgentPanelVisibility(false);
        setTargetPanelVisibility(true);
        dialog.setSize(new Dimension(700, 300));
    }

    @Override
    protected boolean executeTactic() {
        String[] targetCheckBoxLabels = targetCheckBoxGroup.getSelected();
        String[] targetIDs = getUidsFromCheckBoxGroupLabels(targetCheckBoxLabels);
        return executeCommand(targetIDs, sdoPositionTrack);
    }

    @Override
    protected boolean executeCommand(String[] targetIDs) {
        return false;
    }

    @Override
    protected boolean executeCommand(String[] targetCheckBoxLabels, SDOTrack sdoPositionTrack) {
        String[] targetIDs = getUidsFromCheckBoxGroupLabels(targetCheckBoxLabels);
        if (targetIDs.length == 0){
            JOptionPane.showMessageDialog(worldWindAppFrame, "No targets selected!", "Warning",
                    JOptionPane.ERROR_MESSAGE);
            return false;
        }

        if (!worldWindAppFrame.getVisualization().getConfiguration().isInSim()){
            return false;
        }

        java.util.List<SDOPosition> sdoPositions = sdoPositionTrack.getPositions();
        java.util.List<Position> positions = new ArrayList<>();

        for (SDOPosition sdoPosition : sdoPositions) {
            //double altitude = Math.max(sdoPosition.getAltitude(), 0.0);
            Position pos = new Position(sdoPosition.getLatitude(), sdoPosition.getLongitude(),
                    sdoPosition.getElevation());
            positions.add(pos);
        }

        ListIterator<Position> ii = positions.listIterator();
        for (String uid : targetIDs)
            if (ii.hasNext()) {
                
                worldWindAppFrame.getVisualization().getNullSim().setSimTargetPosition(uid, ii.next());
            } 

        worldWindAppFrame.getTrackController().setIsRoute(true);
        return true;
    }
}
