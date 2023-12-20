//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer;

import com.bbn.ccast.Utils;
import com.bbn.ccast.nullsim.NullSim;
import com.bbn.ccast.nullsim.SimVehicle;
import com.bbn.ccast.util.LatLonAlt;
import com.bbn.ccast.visualizer.tactics.TacticsDialog;
import com.bbn.ccast.visualizer.tracks.PositionType;
import com.bbn.ccast.visualizer.tracks.SDOTrack;
import com.bbn.ccast.visualizer.tracks.TrackController;
import com.bbn.ccast.visualizer.util.HighlightController;
import com.bbn.ccast.visualizer.util.MapClickListener;
import com.bbn.ccast.visualizer.util.PickAgentController;
import com.jogamp.newt.event.InputEvent;
import gov.nasa.worldwind.Model;
import gov.nasa.worldwind.WorldWind;
import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.awt.WorldWindowGLCanvas;
import gov.nasa.worldwind.event.SelectEvent;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.layers.RenderableLayer;
import gov.nasa.worldwind.render.Renderable;
import gov.nasa.worldwind.symbology.SymbologyConstants;
import gov.nasa.worldwind.symbology.TacticalSymbol;
import gov.nasa.worldwind.util.StatusBar;
import org.apache.log4j.Logger;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.*;

import static com.bbn.ccast.visualizer.WorldWindVisualization.AGENT_LAYER_NAME;

public class WorldWindAppPanel extends JPanel {
    private static final Logger logger = Logger.getLogger(WorldWindAppPanel.class.getName());
    private static final long serialVersionUID = 8394322655971367590L;
    private HighlightController highlightController;
//    final PickAgentController pickAgentController;
    MapClickListener mapClickListener;
    final WorldWindAppFrame frame;
    private WorldWindow wwd;
    private StatusBar statusBar;
    private JPopupMenu itemPopupMenu = null;
    private TrackController trackController;
    private String selectedSimTargetId;

    public WorldWindAppPanel(WorldWindAppFrame frame, final Dimension canvasSize,
                             final boolean includeStatusBar) {

        super(new BorderLayout());

        this.frame = frame;
        this.wwd = this.createWorldWindow();
        ((Component) this.wwd).setPreferredSize(canvasSize);

        // Create the default model as described in the current worldwind properties.
        Model m = (Model) WorldWind.createConfigurationComponent(AVKey.MODEL_CLASS_NAME);
        this.wwd.setModel(m);

        // Setup a select listener for the worldmap click-and-go feature
        // this.wwd.addSelectListener(new ClickAndGoSelectListener(this.getWwd(),
        // WorldMapLayer.class));

        this.add((Component) this.wwd, BorderLayout.CENTER);
        if (includeStatusBar) {
            this.statusBar = new StatusBar();
            this.add(statusBar, BorderLayout.PAGE_END);
            this.statusBar.setEventSource(wwd);
        }

        // Add controllers to manage highlighting and tool tips.
        // this.toolTipController =
        // new ToolTipController(this.getWwd(), AVKey.DISPLAY_NAME, null);

        this.highlightController = new HighlightController(this.getWwd(), SelectEvent.ROLLOVER);

        // this.pickAgentController = new PickAgentController(this.getWwd(), SelectEvent.RIGHT_CLICK)
        //         .setOnPickCallback(new PickAgentController.OnItemPicked() {
        //             @Override
        //             public void onItemPicked(TacticalSymbol symbol, int x, int y) {
        //                 openItemClickMenu(WorldWindAppPanel.this, symbol, x, y);
        //             }
        //         });

        this.mapClickListener = new MapClickListener(this.getWwd(), 4);
        setMapClickAction(true);

        // this.wwd.addSelectListener(new SelectListener() {
        // @Override
        // public void selected(SelectEvent event) {
        // if (event.isRightClick()) {
        // logger.info("Right clicked object: " + event.getMouseEvent().getPoint());
        // }
        // }
        // });

    }

    // What happens when you right click on the map
    public void setMapClickAction(boolean agent) {
        // This is what happens when you right click on the map after opening the Goto dialog
        if (agent) {
            // mapClickListener.setOnMapClickCallback((x, y, position, e) -> {
            //     logger.info("Selected positions for Payload:");
            //     if (!(itemPopupMenu != null && itemPopupMenu.isShowing())) {
            //         Position pos = getWwd().getCurrentPosition();
            //         Position noAltPos = new Position(pos.getLatitude(), pos.getLongitude(), 0.0);
            //         openClickMenu(WorldWindAppPanel.this, x, y, noAltPos);
            //     }
            // });
            mapClickListener.setOnMapClickCallback((x, y, position, e) -> {
                logger.trace("Adding position to track.");
                Position pos = getWwd().getCurrentPosition();
                // Set default altitude to 1.0 meters to avoid scraping the ground
                Position noAltPos = new Position(pos.getLatitude(), pos.getLongitude(), 1.0);
                logger.info("Selected Payload Position: " + noAltPos);

                TacticsDialog tacticsDialog = frame.getActiveTacticsDialog();
                if (tacticsDialog == null) {
                    trackController.appendPositionTypeToCurrentTrack(PositionType.GOTO, noAltPos);
                } else {
                    tacticsDialog.appendPositionTypeToTrack(PositionType.GOTO, noAltPos, 0.0, 10.0);
                }
            });
        } else {
            mapClickListener.setOnMapClickCallback((x, y, position, e) -> {
                logger.trace("Adding position to track.");
                Position pos = getWwd().getCurrentPosition();
                Position noAltPos = new Position(pos.getLatitude(), pos.getLongitude(),0.0);
                logger.info("Selected Position: " + noAltPos);

                TacticsDialog tacticsDialog = frame.getActiveTacticsDialog();
                if (tacticsDialog == null) {
                    trackController.appendPositionTypeToCurrentTrack(PositionType.GOTO, noAltPos);
                } else {
                    tacticsDialog.appendPositionTypeToTrack(PositionType.GOTO, noAltPos, 0.0, 0.0);
                }
            });
        }
    }

    /**
     * Sets which button is used to activate certain actions.
     *
     * @param modifierMask 4 for left click and 16 for right click
     */
    public void setMapClickModifier(int modifierMask) {
        mapClickListener.setModifierBitmask(modifierMask);
    }

    public HighlightController getHighlightController() {
        return this.highlightController;
    }

    public void setTrackController(TrackController trackController) {
        this.trackController = trackController;
    }

    private void openItemClickMenu(WorldWindAppPanel panel, TacticalSymbol symbol, int x, int y) {
        // itemPopupMenu = initializeItemPopupMenu(symbol);
        // itemPopupMenu.show(panel, x, y);
    }

    Set<String> getSelectedUids() {
        Set<String> retVal = new HashSet<String>();
        Layer layer = wwd.getModel().getLayers().getLayerByName(AGENT_LAYER_NAME);
        if (layer instanceof RenderableLayer) {
            RenderableLayer rl = (RenderableLayer) layer;
            Iterable<Renderable> objects = rl.getRenderables();
            for (Object obj : objects) {
                if (obj instanceof TacticalSymbol) {
                    TacticalSymbol symbol = (TacticalSymbol) obj;
                    if (symbol.isHighlighted()) {
                        Object uniqueDesignation = symbol.getModifier(SymbologyConstants.UNIQUE_DESIGNATION);
                        if (uniqueDesignation != null) {
                            String uid = uniqueDesignation.toString();
                            retVal.add(uid);
                        }
                    }

                }
            }
        }
        logger.trace("Selected agents: " + retVal);
        return retVal;
    }

    public static final String AGENT_LIST_DELIMITER = ",";


    public String getSelectedAgentListString() {
        StringJoiner sj = new StringJoiner(",");
        Set<String> uids = getSelectedUids();
        if (uids != null && !uids.isEmpty()) {
            for (String uid : uids) {
                sj.add(uid);
            }
        }
        return sj.toString();
    }

    public String[] getSelectedAgents() {
        return getSelectedAgentListString().split(AGENT_LIST_DELIMITER);
    }

    public static final String getUidFromSymbol(TacticalSymbol symbol) {
        Object uniqueDesignation = symbol.getModifier(SymbologyConstants.UNIQUE_DESIGNATION);
        String uid = null;
        if (uniqueDesignation != null) {
            uid = uniqueDesignation.toString();
        }
        return uid;
    }

    private JPopupMenu initializeItemPopupMenu(TacticalSymbol symbol) {
        JPopupMenu ret = new JPopupMenu();

        final String targetUidFinal = getUidFromSymbol(symbol);
        logger.trace("Target UID: " + targetUidFinal);
        final String[] agentListFinal = getSelectedAgentListString().split(",");
        logger.trace("Tasking selected agents: " + getSelectedAgentListString());


        JMenuItem item = new JMenuItem("GoTo " + targetUidFinal);
        item.addActionListener(e -> {
            logger.trace("GoTo " + targetUidFinal);
            logger.info("GoTo");
//			try {
//				MoveGtpListOrTargetIdListCommandType goToWithinRegionCmd = new
//				MoveGtpListOrTargetIdListCommandType();
//				CommandXmlSerializationTools.setDefaultsForCommandFields(GoToWithinRegion.getCommandName(),
//				Arrays.asList(agentListFinal), goToWithinRegionCmd);
//				MoveGtpListOrTargetIdListMovementArgsType movementArgsType = new
//				MoveGtpListOrTargetIdListMovementArgsType();
//				MovementTargetsType movementTargetsType = new MovementTargetsType();
//				TargetIdListType targetIdListType = new TargetIdListType();
//				targetIdListType.getTargetIds().add(targetUidFinal);
//				movementTargetsType.setTargetIdList(targetIdListType);
//				movementArgsType.setMovementTargets(movementTargetsType);
//				movementArgsType.setMovementType(MovementType.PLANNER_GOTO);
//				goToWithinRegionCmd.setMovementArgs(movementArgsType);
//				sendDispatcherCommand(goToWithinRegionCmd);
//			} catch (IOException e1) {
//				e1.printStackTrace();
//			}
        });
        ret.add(item);

        // ret.addSeparator();

        item = new JMenuItem("Stop");
        item.addActionListener(e -> {
            logger.trace("Stop " + agentListFinal);
            logger.info("Stop " + agentListFinal);
//			try {
//				DispatcherCommand dc = new DispatcherCommand.Builder(Stop.getCommandName(), agentListFinal)
//						.build();
//				sendDispatcherCommand(dc.toJson());
//			} catch (IOException e1) {
//				e1.printStackTrace();
//			}
        });
        ret.add(item);

        return ret;
    }


    JPopupMenu popupMenu = null;

    private void openClickMenu(Component panel, int x, int y, Position position) {
        popupMenu = initializePopupMenu(position);
        popupMenu.show(panel, x, y);
    }

    private JPopupMenu initializePopupMenu(final Position position) {
        JPopupMenu ret = new JPopupMenu();
        ret.setLightWeightPopupEnabled(false);


        final String[] agentListFinal = getSelectedAgentListString().split(AGENT_LIST_DELIMITER);
        logger.info("Tasking selected agents: " + Arrays.toString(agentListFinal));

        JMenuItem item;
//        /*** Guided Goto ***/
//        JMenuItem item = new JMenuItem("GoTo");
//        item.addActionListener(new ActionListener() {
//            @Override
//            public void actionPerformed(ActionEvent e) {
//
//                if (!frame.getVisualization().getConfiguration().isInSim()){
//                    return;
//                }
//
//                String s = (String) JOptionPane.showInputDialog(frame, "Altitude:", "Altitude",
//                        JOptionPane.PLAIN_MESSAGE, null, null, "0.0");
//
//                if (s != null && !s.isEmpty()) {
//                    logger.trace("Guided GoTo to " + position);
//                    double lat = position.getLatitude().getDegrees();
//                    double lon = position.getLongitude().getDegrees();
//                    try {
//                        double alt = Double.parseDouble(s);
//                        logger.info("GoTo " + Arrays.toString(agentListFinal));
//                        logger.info("GoTo pos: " + position);
//                        Map<String, SimVehicle> vehicleMap =
//                                frame.getVisualization().getNullSim().getAllVehicles();
//                        for (String uid : agentListFinal) {
//                            SimVehicle vehicle = vehicleMap.get(uid);
//                            if (vehicle != null) {
//                                if (!vehicle.isArmed()) {
//                                    vehicle.armDisarm(true);
//                                }
//                                vehicle.setTarget(new LatLonAlt(lat, lon, alt));
//                            }
//                        }
//
//
////						try {
////							DispatcherCommand dc = new DispatcherCommand.Builder(GoToWithinRegion
////							.getCommandName(), agentListFinal)
////									.atLLA(new LatLonAlt[] {new LatLonAlt(lat, lon, alt)})
////									.withMoveType(GoToWithinRegion.GUIDED_GOTO)
////									.build();
////							sendDispatcherCommand(dc.toJson());
////						} catch (IOException e1) {
////							e1.printStackTrace();
////						}
//                    } catch (NumberFormatException ex) {
//                        JOptionPane.showMessageDialog(frame, "Invalid altitude: " + s, "Invalid Input",
//                                JOptionPane.ERROR_MESSAGE);
//                    }
//                }
//            }
//        });
//        ret.add(item);


//        /*** LookAt ***/
//        item = new JMenuItem("LookAt");
//        item.addActionListener(new ActionListener() {
//            @Override
//            public void actionPerformed(ActionEvent e) {
//
//                if (!frame.getVisualization().getConfiguration().isInSim()){
//                    return;
//                }
//
//                String s = (String) JOptionPane.showInputDialog(frame, "Altitude:", "Altitude",
//                        JOptionPane.PLAIN_MESSAGE, null, null, "0.0");
//
//                if (s != null && !s.isEmpty()) {
//                    logger.trace("LookAt " + position);
//                    try {
//                        logger.info("LookAt " + position);
//
//                        Map<String, SimVehicle> vehicleMap =
//                                frame.getVisualization().getNullSim().getAllVehicles();
//                        for (String uid : agentListFinal) {
//                            SimVehicle vehicle = vehicleMap.get(uid);
//                            if (vehicle != null) {
//                                if (!vehicle.isArmed()) {
//                                    vehicle.armDisarm(true);
//                                }
//                                LatLonAlt curLla = vehicle.getPosition();
//                                Position curPos = new Position(Angle.fromDegrees(curLla.getLat()),
//                                        Angle.fromDegrees(curLla.getLon()), curLla.getAlt());
//                                double targetHeading =
//                                        LatLon.greatCircleAzimuth(curPos, position).getDegrees();
//                                if (targetHeading < 0) {
//                                    targetHeading += 360;
//                                }
//                                vehicle.setAbsoluteYawTarget((float) targetHeading);
//                            }
//                        }
//
////						try {
////							DispatcherCommand dc = new DispatcherCommand.Builder(LookAt.getCommandName(),
////							agentListFinal)
////									.atLLA(new LatLonAlt[] {new LatLonAlt(lat,lon,alt)})
////									.build();
////							sendDispatcherCommand(dc.toJson());
////						} catch (IOException e1) {
////							e1.printStackTrace();
////						}
//
//                    } catch (NumberFormatException ex) {
//                        JOptionPane.showMessageDialog(frame, "Invalid altitude: " + s, "Invalid Input",
//                                JOptionPane.ERROR_MESSAGE);
//                    }
//                }
//            }
//        });
//        ret.add(item);

//        ret.addSeparator();

        /*** Append Position to Track ***/
        item = new JMenuItem("Add GoTo Position");
        item.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                logger.trace("Adding position to track.");

                double altitude = Math.max(position.getElevation(), 1.0);
                Position pos = new Position(position.getLatitude(), position.getLongitude(), altitude);
                logger.info("Selected Position: " + pos);
                TacticsDialog tacticsDialog = frame.getActiveTacticsDialog();
                if (tacticsDialog == null) {
                    trackController.appendPositionTypeToCurrentTrack(PositionType.GOTO, pos);
                } else {
                    tacticsDialog.appendPositionTypeToTrack(PositionType.GOTO, pos, 0.0, 0.0);
                }
            }
        });
        int row = trackController.getSDOListPanel().getTable().getSelectedRow();
        // ret.add(item);

//        /*** Insert Position to Track - Above ***/
//        item = new JMenuItem("Insert Position to Track (Above)");
//        item.addActionListener(new ActionListener() {
//            @Override
//            public void actionPerformed(ActionEvent e) {
//                logger.trace("Insert position to track, above current selection location.");
//                Position pos = new Position(position.getLatitude(), position.getLongitude(),
//                        position.getElevation());
//                if (position.getAltitude() < 0.0) {
//                    pos = new Position(position.getLatitude(), position.getLongitude(), 0.0);
//                }
//                logger.info("Selected Position: " + pos);
//
//
//                TacticsDialog td = frame.getActiveTacticsDialog();
//                if (td == null) {
//                    trackController.insertPositionTypeToCurrentTrack(PositionType.GOTO, pos, true);
//                } else {
//                    td.insertPositionTypeToTrack(PositionType.GOTO, pos, 0.0, 0.0, true);
//                }
//            }
//        });
////		int index = trackController.getSDOPanel().getPositionTable().getSelectionModel()
////		.getMinSelectionIndex();
////		if ((index >= 0 && tacticsDialog == null) || ((!(tacticsDialog instanceof LookAtTacticsDialog))
////				&& getTacticsDialogPositionMinSelectionIndex() >= 0)) {
////			item.setEnabled(true);
////		} else {
////			item.setEnabled(false);
////		}
//
//        ret.add(item);
//
//        /*** Insert Position to Track - Below ***/
//        item = new JMenuItem("Insert Position to Track (Below)");
//        item.addActionListener(new ActionListener() {
//            @Override
//            public void actionPerformed(ActionEvent e) {
//                logger.trace("Insert position to track, below current selection location.");
//                Position pos = new Position(position.getLatitude(), position.getLongitude(),
//                        position.getElevation());
//                if (position.getAltitude() < 0.0) {
//                    pos = new Position(position.getLatitude(), position.getLongitude(), 0.0);
//                }
//                logger.info("Selected Position: " + pos);
//
//
//                TacticsDialog td = frame.getActiveTacticsDialog();
//                if (td == null) {
//                    trackController.insertPositionTypeToCurrentTrack(PositionType.GOTO, pos, false);
//                } else {
//                    td.insertPositionTypeToTrack(PositionType.GOTO, pos, 0.0, 0.0, false);
//                }
//            }
//        });
////		index = trackController.getSDOPanel().getPositionTable().getSelectionModel().getMaxSelectionIndex();
////		if ((index >= 0 && tacticsDialog == null) || ((!(tacticsDialog instanceof LookAtTacticsDialog))
////				&& (getTacticsDialogPositionMaxSelectionIndex() >= 0))) {
////			item.setEnabled(true);
////		} else {
////			item.setEnabled(false);
////		}
//
//        ret.add(item);
//
//        ret.addSeparator();
//
//        /*** Append Look At to Track ***/
//        item = new JMenuItem("Add Look Point to Track");
//        item.addActionListener(new ActionListener() {
//            @Override
//            public void actionPerformed(ActionEvent e) {
//                logger.trace("Adding look point to track.");
//                Position pos = new Position(position.getLatitude(), position.getLongitude(),
//                        position.getElevation());
//                if (position.getAltitude() < 0.0) {
//                    pos = new Position(position.getLatitude(), position.getLongitude(), 0.0);
//                }
//                logger.info("Selected Position: " + pos);
////
////				TacticsDialog td = frame.getActiveTacticsDialog();
////				if (td == null) {
////					trackController.appendPositionTypeToCurrentTrack(PositionType.LOOKPOINT, pos);
////				} else {
////					td.appendPositionTypeToTrack(PositionType.LOOKPOINT, pos);
////				}
//
//            }
//        });
////		index = trackController.getSDOPanel().getPositionTable().getSelectionModel().getMinSelectionIndex();
////		if ((index > 0 && tacticsDialog == null)
////				|| (tacticsDialog instanceof LookAtTacticsDialog && isTacticsDialogAcceptingPositionData())
////				|| (!(tacticsDialog instanceof LookAtTacticsDialog)
////						&& getTacticsDialogPositionMinSelectionIndex() >= 0)) {
////			item.setEnabled(true);
////		} else {
////			item.setEnabled(false);
////		}
//
//        ret.add(item);
//
//        /*** Insert Look At to Track - Above ***/
//        item = new JMenuItem("Insert Look Point to Track (Above)");
//        item.addActionListener(new ActionListener() {
//            @Override
//            public void actionPerformed(ActionEvent e) {
//                logger.trace("Insert look point to track, above current selection location.");
//                Position pos = new Position(position.getLatitude(), position.getLongitude(),
//                        position.getElevation());
//                if (position.getAltitude() < 0.0) {
//                    pos = new Position(position.getLatitude(), position.getLongitude(), 0.0);
//                }
//                logger.info("Selected Position: " + pos);
////
////
////				TacticsDialog td = frame.getActiveTacticsDialog();
////				if (td == null) {
////					trackController.insertPositionTypeToCurrentTrack(PositionType.LOOKPOINT, pos, true);
////				} else {
////					td.insertPositionTypeToTrack(PositionType.LOOKPOINT, pos, true);
////				}
//            }
//        });
////		index = trackController.getSDOPanel().getPositionTable().getSelectionModel().getMinSelectionIndex();
////		if ((index > 0 && tacticsDialog == null)
////				|| (tacticsDialog instanceof LookAtTacticsDialog &&
////				getTacticsDialogPositionMinSelectionIndex() >= 0)
////				|| (!(tacticsDialog instanceof LookAtTacticsDialog)
////						&& getTacticsDialogPositionMinSelectionIndex() > 0)) {
////			item.setEnabled(true);
////		} else {
////			item.setEnabled(false);
////		}
//
//        ret.add(item);
//
//        /*** Insert Look At to Track - Below ***/
//        item = new JMenuItem("Insert Look Point to Track (Below)");
//        item.addActionListener(new ActionListener() {
//            @Override
//            public void actionPerformed(ActionEvent e) {
//                logger.trace("Insert look point to track, below current selection location.");
//                Position pos = new Position(position.getLatitude(), position.getLongitude(),
//                        position.getElevation());
//                if (position.getAltitude() < 0.0) {
//                    pos = new Position(position.getLatitude(), position.getLongitude(), 0.0);
//                }
//                logger.info("Selected Position: " + pos);
////
////				TacticsDialog td = frame.getActiveTacticsDialog();
////				if (td == null) {
////					trackController.insertLookAtToCurrentTrack(pos, false);
////				} else {
////					td.insertPositionTypeToTrack(PositionType.LOOKPOINT, pos, false);
////				}
//            }
//        });
//
////		index = trackController.getSDOPanel().getPositionTable().getSelectionModel().getMaxSelectionIndex();
////		if ((index >= 0 && tacticsDialog == null) || getTacticsDialogPositionMaxSelectionIndex() >= 0) {
////			item.setEnabled(true);
////		} else {
////			item.setEnabled(false);
////		}
        // ret.add(item);

        // ret.addSeparator();


//		/*** Change World Region ***/
//		item = new JMenuItem("Change World Region");
//		item.addActionListener(new ActionListener() {
//			@Override
//			public void actionPerformed(ActionEvent e) {
//				WorldRegions worldRegions = frame.visualization.getCcastWorldDatabase().getWorldRegions();
//				Object[] choices = worldRegions.getRegions().values().toArray();
//
//
//				JList<Object> list = new JList<Object>(choices);
//				JScrollPane jscrollpane=new JScrollPane();
//				jscrollpane.setViewportView(list);
//
//				//JOptionPane.showMessageDialog(frame, jscrollpane, "World Region", JOptionPane
//				.PLAIN_MESSAGE);
//				int result = JOptionPane.showOptionDialog(frame, jscrollpane, "World Region", JOptionPane
//				.OK_CANCEL_OPTION, JOptionPane.PLAIN_MESSAGE, null, null, null);
//
//
//				//Object region = JOptionPane.showInputDialog(frame, "World Region:", "World Region",
//				//		JOptionPane.PLAIN_MESSAGE, null, choices, choices[0]);
//
//				if (result != JOptionPane.CLOSED_OPTION && result != JOptionPane.CANCEL_OPTION && list
//				.getSelectedIndex() != -1) {
//					frame.visualization.clearVisualization(WorldWindVisualization.OBSTACLE_LAYER_NAME);
//					int[] indices = list.getSelectedIndices();
//					Region largestRegion = null;
//					double largestArea = 0.0;
//					for (int i=0; i < indices.length; i++) {
//						Region region = (Region) list.getModel().getElementAt(indices[i]);
//						frame.visualization.getCcastWorldDatabase().setCurrentRegionId(region.getId());
//						frame.visualization.displayRegionData();
//						if (frame.visualization.getCcastWorldDatabase().getCurrentDB().getArea() >
//						largestArea) {
//							largestRegion = region;
//							largestArea = frame.visualization.getCcastWorldDatabase().getCurrentDB()
//							.getArea();
//						}
//					}
//					if (largestRegion != null) {
//						frame.visualization.getCcastWorldDatabase().setCurrentRegionId((largestRegion).getId
//						());
//					}
//				}
//			}
//		});
//		ret.add(item);
//
//		ret.addSeparator();

//        /*** Return to Launch ***/
//        item = new JMenuItem("Return to Launch");
//        item.addActionListener(new ActionListener() {
//            @Override
//            public void actionPerformed(ActionEvent e) {
//
//                if (!frame.getVisualization().getConfiguration().isInSim()){
//                    return;
//                }
//
//                logger.info("Return To Launch " + Arrays.toString(agentListFinal));
//                logger.info("RTL selected.");
//                Map<String, SimVehicle> vehicleMap = frame.getVisualization().getNullSim().getAllVehicles();
//                for (String uid : agentListFinal) {
//                    SimVehicle vehicle = vehicleMap.get(uid);
//                    if (vehicle != null) {
//                        vehicle.returnToLaunch();
//                    }
//                }
////				try {
////					DispatcherCommand dc = new DispatcherCommand.Builder(ReturnToLaunchMultiRegion
////					.getCommandName(), agentListFinal)
////							.build();
////					sendDispatcherCommand(dc.toJson());
////				} catch (IOException e1) {
////					e1.printStackTrace();
////				}
//            }
//        });
//        ret.add(item);


//        /*** Stop All ***/
//        item = new JMenuItem("Stop All");
//        item.addActionListener(new ActionListener() {
//            @Override
//            public void actionPerformed(ActionEvent e) {
//
//                if (!frame.getVisualization().getConfiguration().isInSim()){
//                    return;
//                }
//
//                logger.info("Stopping " + agentListFinal);
//                Map<String, SimVehicle> vehicleMap = frame.getVisualization().getNullSim().getAllVehicles();
//                for (String uid : agentListFinal) {
//                    SimVehicle vehicle = vehicleMap.get(uid);
//                    if (vehicle != null) {
//                        vehicle.setTarget(vehicle.getPosition());
//                    }
//                }
////				try {
////					DispatcherCommand dc = new DispatcherCommand.Builder(Stop.getCommandName(), new String[]
////					{"all"})
////							.build();
////					sendDispatcherCommand(dc.toJson());
////				} catch (IOException e1) {
////					e1.printStackTrace();
////				}
//            }
//        });
//        ret.add(item);

        // /*** Select Position for Sim Target ***/
        // item = new JMenuItem("Select Position for Sim Target");
        // item.addActionListener(new ActionListener() {
        //     @Override
        //     public void actionPerformed(ActionEvent e) {

        //         if (!frame.getVisualization().getConfiguration().isInSim()){
        //             return;
        //         }

        //         logger.trace("Selecting position for target.");
        //         Position pos = new Position(position.getLatitude(), position.getLongitude(),
        //                 position.getElevation());
        //         if (position.getAltitude() < 0.0) {
        //             pos = new Position(position.getLatitude(), position.getLongitude(), 0.0);
        //         }

        //         logger.info("Selected Position: " + pos);

        //         String targetId;
        //         if (selectedSimTargetId == null) {
        //             targetId = (String) JOptionPane.showInputDialog(frame, "Enter the uid of the target " +
        //                             "whose position is being updated.", "Sim Target UID",
        //                     JOptionPane.PLAIN_MESSAGE, null, null, "");
        //         } else {
        //             targetId = selectedSimTargetId;
        //             selectedSimTargetId = null;
        //             frame.getTargetPanel().clearSelection();
        //         }

        //         frame.getVisualization().getNullSim().setSimTargetPosition(targetId, position);
        //     }
        // });
        // ret.add(item);

//        /*** Add Sim Target ***/
//        item = new JMenuItem("Add Sim Target");
//        item.addActionListener(new ActionListener() {
//            @Override
//            public void actionPerformed(ActionEvent e) {
//                if (!frame.getVisualization().getConfiguration().isInSim()){
//                    return;
//                }
//
//                logger.info("Adding sim target at " + position);
//                NullSim nullSim = frame.getVisualization().getNullSim();
//                nullSim.addSimTarget(position);
//            }
//        });
//        //TODO: Uncomment when we can spinup a new python target from here: ret.add(item);
//
//        /*** Remove Sim Targets ***/
//        item = new JMenuItem("Remove Sim Target");
//        item.addActionListener(new ActionListener() {
//            @Override
//            public void actionPerformed(ActionEvent e) {
//                if (!frame.getVisualization().getConfiguration().isInSim()){
//                    return;
//                }
//
//                String targetId = (String) JOptionPane.showInputDialog(frame, "Target ID to remove",
//                        "Target ID",
//                        JOptionPane.PLAIN_MESSAGE, null, null, "0");
//                if (targetId != null && !targetId.isEmpty()) {
//                    logger.info("Removing sim target " + targetId);
//                    NullSim nullSim = frame.getVisualization().getNullSim();
//                    nullSim.removeSimTarget(targetId);
//                }
//            }
//        });
//        ret.add(item);

        return ret;
    }

//	private boolean isTacticsDialogAcceptingPositionData() {
//		TacticsDialog tacticsDialog = frame.getActiveTacticsDialog();
//		if (tacticsDialog != null && tacticsDialog.isPositionDataSelected()) {
//			return true;
//		}
//		return false;
//	}
//
//	private int getTacticsDialogPositionMaxSelectionIndex() {
//		TacticsDialog tacticsDialog = frame.getActiveTacticsDialog();
//		if (tacticsDialog != null && tacticsDialog.isPositionDataSelected()) {
//			return tacticsDialog.getPositionMaxSelectionIndex();
//		}
//		return -1;
//	}
//
//	private int getTacticsDialogPositionMinSelectionIndex() {
//		TacticsDialog tacticsDialog = frame.getActiveTacticsDialog();
//		if (tacticsDialog != null && tacticsDialog.isPositionDataSelected()) {
//			return tacticsDialog.getPositionMinSelectionIndex();
//		}
//		return -1;
//	}

//	private void sendDispatcherCommand(CommandType cmd) throws IOException {
//		String xml = CommandXmlSerializationTools.getInstance().cmdToXml(cmd);
//		sendDispatcherCommand(xml);
//	}

//	private void sendDispatcherCommand(String dispatcherCommand) throws IOException {
//		logger.trace(dispatcherCommand);
//
//		String host = this.frame.visualization.getConfiguration().getStringProperty
//		(DISPATCHER_HOST_ADDR_PROPERTY);
//		Integer port = this.frame.visualization.getConfiguration().getIntProperty(DISPATCHER_PORT_PROPERTY);
//
//		sendDispatcherCommand(dispatcherCommand, host, port);
//	}

//	public static void sendDispatcherCommand(final String dispatcherCommand, final String host, final int
//	port)
//			throws IOException {
//		try (CloseableHttpClient httpClient = HttpClientBuilder.create().build()) {
//			HttpPost request = new HttpPost("http://" + host + ":" + port + DispatcherHttpListener
//			.DISPATCHER_COMMAND_URI_PATH);
//			StringEntity params = new StringEntity(dispatcherCommand);
//			request.setEntity(params);
//			request.addHeader("content-type", "text/xml");
//			try (CloseableHttpResponse response = httpClient.execute(request)) {
//				logger.trace(response.getStatusLine().getStatusCode());
//			}
//		}
//	}


    public void addTrack(SDOTrack track) {
        trackController.addTrack(track);
    }

    public void addTracks(Collection<SDOTrack> tracks) {
        for (SDOTrack track : tracks) {
            addTrack(track);
        }
    }


    protected WorldWindow createWorldWindow() {
        return new WorldWindowGLCanvas();
    }

    public WorldWindow getWwd() {
        return wwd;
    }

    public void setSelectedTarget(String uid) {
        this.selectedSimTargetId = uid;
    }

    // public StatusBar getStatusBar() {
    // return statusBar;
    // }
}
