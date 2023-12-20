//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.AgentTelemPackage;
import com.bbn.ccast.config.Configuration;
import com.bbn.ccast.nullsim.SimVehicle;
import com.bbn.ccast.nullsim.blenodes.PayloadEventHandler;
import com.bbn.ccast.visualizer.WorldWindAppFrame;
import com.bbn.ccast.visualizer.tactics.IntentTacticsDialog;
import com.bbn.ccast.visualizer.tactics.StopTacticsDialog;
import com.bbn.ccast.visualizer.tactics.TacticsDialog;
import org.apache.log4j.Logger;

import javax.swing.*;
import java.awt.*;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.*;

import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;

public class ExecutionToolBar extends AbstractToolBar {

	private static final Logger logger = Logger.getLogger(ExecutionToolBar.class.getName());
	private String DEFAULT_TRACK_PATH = "/tracks/productionTracks";

    public ExecutionToolBar(WorldWindAppFrame wwaf) {
        super(wwaf);
        worldWindAppFrame.getVisualization().getConfiguration();
		String tmp = worldWindAppFrame.getVisualization().getConfiguration().getStringProperty
                (Configuration.EXECUTION_TOOLBAR_PATH_PROPERTY);
        trackPath = tmp != null ? tmp : DEFAULT_TRACK_PATH;
    }

    @Override
    protected void addCustomButtonsToToolBar(JToolBar toolBar){
		toolBar.addSeparator();
		addCollisionToggle(toolBar, worldWindAppFrame);
		toolBar.addSeparator();
		addViewAnalyticsButton(toolBar, worldWindAppFrame);
		addResetButton(toolBar, worldWindAppFrame);
		toolBar.addSeparator();
		addCaptureToggle(toolBar, worldWindAppFrame);
    }
	


	// == Enable Capture == //
	private void addCollisionToggle(JToolBar toolBar, WorldWindAppFrame worldWindAppFrame1) {

		// Get the configuration
		com.bbn.ccast.config.Configuration configuration = worldWindAppFrame.getVisualization().getConfiguration();

		// create a collision check box
		JToggleButton collisionCheckBox = new JCheckBox("Enable Collision Warnings", false);
		collisionCheckBox.setToolTipText("Enable/Disable collision warnings");

		// ItemListener is notified whenever you click on the Button
		// itemStateChanged() method is invoked automatically when checkbox is ticked/unticked
		ItemListener itemListener = e -> {
			// Event if bbox is ticked
			if (e.getStateChange() == 1) {
				System.out.println("Collision Warnings Enabled");
				worldWindAppFrame.getVisualization().getNullSim().setCollisionEnabled(true);
			} else {
				// Event box is unticked
				System.out.println("Collision Warnings Disabled");
				worldWindAppFrame.getVisualization().getNullSim().setCollisionEnabled(false);
			}

		};

		// Attach Listeners
		collisionCheckBox.addItemListener(itemListener);
		toolBar.add(collisionCheckBox);

	}


	// == Enable Capture == //
	private void addCaptureToggle(JToolBar toolBar, WorldWindAppFrame worldWindAppFrame1) {
		// Do not display this button if user is blue force
		com.bbn.ccast.config.Configuration configuration = worldWindAppFrame.getVisualization().getConfiguration();
		if (configuration.getForceType().equals("blue")){
			return;
		}
		// create a ToggleButton
		JToggleButton toggleButton = makeToolBarToggleButton("Enable/Disable target capture", "Enable Capture");

		// ItemListener is notified whenever you click on the Button
		// itemStateChanged() method is invoked automatically
		// whenever you click or unclick on the Button.
		ItemListener itemListener = e -> {
			boolean sim = false;
			Map<String, SimVehicle> vehicleMap = new HashMap<>();
			if (configuration.isInSim()) {
				sim = true;
				vehicleMap = worldWindAppFrame.getVisualization().getNullSim().getAllVehicles();
			}
			Map<String, AgentTelemPackage> telem = worldWindAppFrame.getVisualization().getAgentTelemMap();
			Collection<SimVehicle.Intent> payloadIntent = new ArrayList<SimVehicle.Intent>();
			PayloadEventHandler eventHandler = new PayloadEventHandler(worldWindAppFrame.getVisualization().maceMessageTransport);

			// Event if button is toggled on
			if (e.getStateChange() == ItemEvent.SELECTED) {
				System.out.println("Selected");

				payloadIntent.add(SimVehicle.Intent.MASS);
				payloadIntent.add(SimVehicle.Intent.PERI);
				payloadIntent.add(SimVehicle.Intent.LINK);

				for (AgentTelemPackage pkg : telem.values()) {
					if (sim) {
						SimVehicle vehicle = vehicleMap.get(pkg.getUid());
						if (vehicle != null) {
							vehicle.clearIntent();
							vehicle.setIntent(payloadIntent);
						}
					}
					eventHandler.sendPayloadConfig(pkg.getUid(), payloadIntent);
				}
			} else {
			// Event if toggled off
				System.out.println("Deselected");

				payloadIntent.add(SimVehicle.Intent.IDLE);

				for (AgentTelemPackage pkg : telem.values()) {
					if (sim) {
						SimVehicle vehicle = vehicleMap.get(pkg.getUid());
						if (vehicle != null) {
							vehicle.clearIntent();
							vehicle.setIntent(payloadIntent);
						}
					}
					eventHandler.sendPayloadConfig(pkg.getUid(), payloadIntent);
				}
			}
		};

		// Attach Listeners
		toggleButton.addItemListener(itemListener);
		toolBar.add(toggleButton);

	}

	// == Reset == //
	private void addResetButton(JToolBar toolBar, WorldWindAppFrame worldWindAppFrame1){
		JButton button; 

		// create a ToggleButton
		button = makeToolBarButton(null, "Reset", "Reset");
		button.addActionListener(e -> worldWindAppFrame1.resetSimulation());

		// Attach Listeners
		toolBar.add(button);
	}

	private void addViewAnalyticsButton(JToolBar toolBar, WorldWindAppFrame worldWindAppFrame1) {
		// == Reset == //
		JButton button; 

		// create a ToggleButton
		button = makeToolBarButton(null, "View analytics", "View Analytics");
		button.addActionListener(e -> worldWindAppFrame1.viewAnalytics());

		// Attach Listeners
		toolBar.add(button);
	}
}


// Unused buttons

//	private void addStopCommands(JToolBar toolBar, WorldWindAppFrame worldWindAppFrame1) {
//		JButton button;
//		// == Stop ==
//		button = makeToolBarButton(null, "Stop", "Stop");
//		button.addActionListener(e -> {
//			logger.info("Stop.");
//			TacticsDialog tacticsDialog = new StopTacticsDialog(worldWindAppFrame1, Dialog.ModalityType.MODELESS,
//					TacticsDialog.EditingActivity.ADD);
//			worldWindAppFrame1.setActiveTacticsDialog(tacticsDialog);
//			tacticsDialog.getDialog().addWindowListener(new WindowAdapter() {
//				@Override
//				public void windowClosed(WindowEvent we) {
//					worldWindAppFrame1.setActiveTacticsDialog(null);
//				}
//			});
//		});
//
//		toolBar.add(button);

//		// == Stop All ==
//		button = makeToolBarButton("24x24-stop.gif", "Stop All Tactics", "Stop All");
//		button.addActionListener(e -> worldWindAppFrame1.stopAllTactics());
//		toolBar.add(button);
//	}

//		// == About ==
//		button = makeToolBarButton("24x24-about.gif", "About the CCAST Swarm C2 application", "About");
//		button.addActionListener(e -> worldWindAppFrame1.showAbout());
//		toolBar.add(button);

//	private void addHelpAndAbout(JToolBar toolBar, WorldWindAppFrame worldWindAppFrame1) {
//		// == Help ==
//		JButton button = makeToolBarButton("24x24-help.gif", "Help", "Help");
//		button.addActionListener(e -> worldWindAppFrame1.showHelp());
//		toolBar.add(button);
//
//
//	}