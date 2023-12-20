//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tracks;

import com.bbn.ccast.visualizer.WorldWindAppFrame;
import com.bbn.ccast.visualizer.WorldWindVisualization;
import com.google.common.collect.Iterables;
import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.layers.TerrainProfileLayer;
import gov.nasa.worldwind.render.Renderable;
import gov.nasa.worldwind.render.airspaces.SphereAirspace;

public class PositionSizeThread implements Runnable {
    WorldWindVisualization wwv;
    WorldWindow wwd;
    Position clickPos;

    public PositionSizeThread(WorldWindVisualization wwv, WorldWindow wwd, Position clickPos) {
        this.wwv = wwv;
        this.wwd = wwd;
        this.clickPos = clickPos;
    }

    @Override
    public void run() {
        while (Iterables.size(wwv.ensureLayer(WorldWindVisualization.MISSION_PLAN_LAYER_NAME).getRenderables()) > 0) {
            Position eye = wwd.getView().getCurrentEyePosition();
            double test = wwd.getView().getGlobe().getElevation(clickPos.getLatitude(), clickPos.getLongitude());
            Iterable<Renderable> spheres =
                    wwv.ensureLayer(WorldWindVisualization.MISSION_PLAN_LAYER_NAME).getRenderables();
            for (Renderable ss : spheres) {
                if (ss instanceof SphereAirspace) {
                    SphereAirspace sphere = (SphereAirspace) ss;
                    sphere.setRadius(wwd.getView().computePixelSizeAtDistance(eye.getElevation() - test) * 8);
                }
            }
            wwv.triggerRedraw();
            try {
                Thread.sleep(33);
            } catch (InterruptedException ignored) {
            }
        }
    }
}
