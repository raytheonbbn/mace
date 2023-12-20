//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.TargetTelemPackage;
import com.bbn.ccast.nullsim.blenodes.SimTargetType;
import com.bbn.ccast.visualizer.WorldWindAppFrame;
import com.bbn.ccast.visualizer.WorldWindVisualization;
import gov.nasa.worldwind.WorldWindow;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;

public class NetworkVisualizationManager implements Runnable {
    private final ConcurrentHashMap<String, LinkedNetworkVisualization> networks;
    private final ConcurrentHashMap<String, Boolean> captureStates;
    private final List<Color> freeColors;
    private final List<Color> usedColors;
    private final WorldWindVisualization wwv;
    private final WorldWindow wwd;
    private boolean checkTelems = false;
    private ConcurrentHashMap<String, TargetTelemPackage> targetTelems;

    public NetworkVisualizationManager(WorldWindVisualization wwv, WorldWindow wwd) {
        freeColors = new ArrayList<>(Arrays.asList(Color.RED, Color.ORANGE, Color.YELLOW, Color.GREEN,
                Color.CYAN, Color.BLUE,
                Color.PINK, Color.MAGENTA, Color.WHITE));
        usedColors = new ArrayList<>();
        networks = new ConcurrentHashMap<>();
        captureStates = new ConcurrentHashMap<>();
        this.wwv = wwv;
        this.wwd = wwd;
        targetTelems = wwv.getTargetTelemMap();
    }

    public void newNetwork(String name, ConcurrentHashMap<String, TargetTelemPackage> targets) {
        checkTelems = false;
        Color color = Color.RED;
        if (!freeColors.isEmpty()) {
            color = freeColors.get(0);
            usedColors.add(color);
            freeColors.remove(0);
        }

        LinkedNetworkVisualization lnv = new LinkedNetworkVisualization(name, targets, color, wwv, wwd);
        networks.put(name, lnv);
        captureStates.put(name, false);

        while(!checkTelems) {
            ConcurrentHashMap<String, TargetTelemPackage> telems = wwv.getTargetTelemMap();
            int count = 0;
            for (String key : targets.keySet()) {
                if (telems.containsKey(key)) {
                    if (!telems.get(key).getNetworksCaptured().isEmpty()) {
                        if (telems.get(key).getNetworksCaptured().containsKey(name)) {
                            count++;
                        }
                    }
                }
            }

            if (count == targets.size())
                checkTelems = true;
        }
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            checkTelems = true;
            ConcurrentHashMap<String, TargetTelemPackage> telemMap = wwv.getTargetTelemMap();
            ConcurrentHashMap<String, TargetTelemPackage> newNetwork = new ConcurrentHashMap<>();
            String newNetName = null;

            for (TargetTelemPackage telem : telemMap.values()) {
                if (!networks.isEmpty() && !telem.getNetworksCaptured().isEmpty()) {
                    for (String name : telem.getNetworksCaptured().keySet()) {
                        if (!networks.containsKey(name) && (newNetName == null || newNetName.equals(name))) {
                            newNetName = name;
                            newNetwork.put(telem.getUid(), telem);
                        }
                    }
                } else if (!telem.getNetworksCaptured().isEmpty()) {
                    for (String name : telem.getNetworksCaptured().keySet()) {
                        if (newNetName == null || newNetName.equals(name)) {
                            newNetName = name;
                            newNetwork.put(telem.getUid(), telem);
                        }
                    }
                }
            }

            if (newNetName != null) {
                newNetwork(newNetName, newNetwork);
                checkTelems = false;
            }

            if (checkTelems && !networks.isEmpty() && !captureStates.isEmpty()) {
                for (String key : networks.keySet()) {
                    Color color = networks.get(key).update(telemMap, checkTelems);
                    if (color != null) {
                        if (!freeColors.contains(color) && color != Color.BLACK) {
                            usedColors.remove(color);
                            freeColors.add(color);
                        }
                        networks.remove(key);
                        captureStates.remove(key);
                    }
                }

                captureStates.forEach((kk, vv) -> {
                    for (TargetTelemPackage telem : telemMap.values()) {
                        if (!telem.getNetworksCaptured().isEmpty()) {
                            if (telem.getNetworksCaptured().containsKey(kk)) {
                                vv = telem.getNetworksCaptured().get(kk);
                            }
                        }
                    }
                    if (Boolean.TRUE.equals(vv)) {
                        if (networks.get(kk).getColor() != Color.BLACK) {
                            usedColors.remove(networks.get(kk).getColor());
                            freeColors.add(networks.get(kk).getColor());
                            networks.get(kk).setColor(Color.BLACK);
                        }
                    } else {
                        if (networks.get(kk).getColor() == Color.BLACK) {
                            if (!freeColors.isEmpty())
                                networks.get(kk).setColor(freeColors.get(0));
                            else
                                networks.get(kk).setColor(Color.RED);
                        }
                    }
                });

                targetTelems = telemMap;

                try {
                    Thread.sleep(500);
                } catch (InterruptedException ignored) {
                }
            }
        }
    }
}
