//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.AgentTelemPackage;
import com.bbn.ccast.nullsim.SimVehicle;
import com.bbn.ccast.nullsim.blenodes.PayloadEventHandler;
import com.bbn.ccast.visualizer.WorldWindAppFrame;
import com.bbn.ccast.visualizer.tactics.*;
import com.bbn.ccast.visualizer.tracks.SDOTrack;

import com.bbn.ccast.visualizer.tracks.TrackController;
import gov.nasa.worldwind.util.WWIO;
import org.apache.log4j.Logger;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;

public abstract class AbstractToolBar {

	protected WorldWindAppFrame worldWindAppFrame;
	protected String trackPath = "";
	private static final Logger logger = Logger.getLogger(AbstractToolBar.class.getName());

	private GoToTacticsDialog previousGoToTacticsDialog;
	private TargetPosTacticsDialog previoustargetPosDialog;
	
	private SDOTrack sdoPositionTrack;


	public AbstractToolBar(WorldWindAppFrame wwaf) {
		worldWindAppFrame = wwaf;
	}

	public JPanel setupToolBar() {
		JToolBar toolBar = new JToolBar();
		toolBar.setLayout(new FlowLayout());
		toolBar.setFloatable(false);
		toolBar.setRollover(true);
		toolBar.setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5)); // top, left, bottom, right

		// Add our buttons


		// Now the tactics consistent across multiple toolbars
		addGotoButtons(toolBar);
		toolBar.addSeparator();
		addConfigButtons(toolBar);
		toolBar.addSeparator();
		addMapLocationButton(toolBar);

		// Add in customButtons from ExecutionToolBar.java
		addCustomButtonsToToolBar(toolBar);

		JPanel panel = new JPanel(new BorderLayout(0, 0)); // hgap, vgap
		panel.add(toolBar);
		return panel;
	}



	public static JButton makeToolBarButton(String imageName, String toolTipText, String altText) {
		JButton button = new JButton(altText);
		button.setToolTipText(toolTipText);

		return button;
	}

	public static JToggleButton makeToolBarToggleButton(String toolTipText, String altText) {
		JToggleButton button = new JToggleButton(altText);
		button.setToolTipText(toolTipText);

		return button;
	}

	private static ImageIcon getIcon(String imageName) {
		String imagePath = "resources/images/" + imageName;
		Object o = WWIO.getFileOrResourceAsStream(imagePath, ExecutionToolBar.class);
		if (!(o instanceof InputStream))
			return null;

		try {
			BufferedImage icon = ImageIO.read((InputStream) o);
			return new ImageIcon(icon);
		} catch (Exception e) {
			return null;
		}
	}

	protected void addMapLocationButton(JToolBar toolBar) {
		JButton button;


		// == Set Map Location ==
		button = makeToolBarButton(null, "Set map location", "Map Location");
		button.addActionListener(e -> {
			ChooseMapDialog mapDialog = new ChooseMapDialog(worldWindAppFrame);
			mapDialog.setup();
		});
		toolBar.add(button);
	}

	protected void addGotoButtons(JToolBar toolBar) {
		JButton button;

		// == GoTo ==
		button = makeToolBarButton(null, "Move sim payloads", "GoTo");
		button.addActionListener(e -> {
			logger.info("GoTo button clicked.");
			// Prevent additional dialogs from opening if button clicked with one open
			boolean showDialog = true;		
			if (previousGoToTacticsDialog != null){
				if (previousGoToTacticsDialog.getDialog().isVisible() == true){
					showDialog = false;
				}
			}
			if (showDialog == true) {
				GoToTacticsDialog tacticsDialog = new GoToTacticsDialog(worldWindAppFrame, Dialog.ModalityType.MODELESS, TacticsDialog.EditingActivity.ADD);
				previousGoToTacticsDialog = tacticsDialog;
				worldWindAppFrame.setActiveTacticsDialog(tacticsDialog);
			}
		});
		toolBar.add(button);
	}


	protected void addConfigButtons(JToolBar toolBar) {
		JButton button;

		// == Set Target Position ==
		button = makeToolBarButton(null, "Set target position(s)", "Target Positions");
		button.addActionListener(e -> {
			logger.info("TargetPos button clicked.");
			// Prevent additional dialogs from opening if button clicked with one open
			boolean showDialog = true;		
			if (previoustargetPosDialog != null){
				if (previoustargetPosDialog.getDialog().isVisible() == true){
					showDialog = false;
				}
			}
			if (showDialog == true) {
				TargetPosTacticsDialog targetPosDialog = new TargetPosTacticsDialog(worldWindAppFrame, Dialog.ModalityType.MODELESS,
						TacticsDialog.EditingActivity.ADD);
				previoustargetPosDialog = targetPosDialog;
				worldWindAppFrame.setActiveTacticsDialog(targetPosDialog);
				targetPosDialog.getDialog().addWindowListener(new WindowAdapter() {
					@Override
					public void windowClosed(WindowEvent we) {
						if (sdoPositionTrack != null) {
							sdoPositionTrack.firePropertyChange(TrackController.TRACK_REMOVE, null, sdoPositionTrack);
						}
						worldWindAppFrame.getWwjPanel().setMapClickModifier(4);
						worldWindAppFrame.getWwjPanel().setMapClickAction(true);
						worldWindAppFrame.setActiveTacticsDialog(null);

					}
				});
			}	
		});
		toolBar.add(button);

		// == Set Payload Intent/Target Type ==
		button = makeToolBarButton(null, "Set target type and payload intent", "Configuration");
		button.addActionListener(e -> {
			logger.info("Intent button clicked.");
			IntentTacticsDialog intentDialog = new IntentTacticsDialog(worldWindAppFrame, Dialog.ModalityType.MODELESS, TacticsDialog.EditingActivity.ADD);
			worldWindAppFrame.setActiveTacticsDialog(intentDialog);
			intentDialog.getDialog().addWindowListener(new WindowAdapter() {
				@Override
				public void windowClosed(WindowEvent we) {
					worldWindAppFrame.setActiveTacticsDialog(null);
				}
			});
		});
		toolBar.add(button);

		// == Set Discovered ==
		button = makeToolBarButton(null, "Set whether a target has been discovered", "Set Discovered");
		button.addActionListener(e -> {
			logger.info("Discover button clicked.");
			DiscoverTacticsDialog intentDialog = new DiscoverTacticsDialog(worldWindAppFrame, Dialog.ModalityType.MODELESS, TacticsDialog.EditingActivity.ADD);
			worldWindAppFrame.setActiveTacticsDialog(intentDialog);
			intentDialog.getDialog().addWindowListener(new WindowAdapter() {
				@Override
				public void windowClosed(WindowEvent we) {
					worldWindAppFrame.setActiveTacticsDialog(null);
				}
			});
		});
		toolBar.add(button);
	}

	protected abstract void addCustomButtonsToToolBar(JToolBar toolBar);


}


// Unused buttons

//		// == Open track from file ==
//		JButton button = makeToolBarButton("24x24-open.gif", "Open track from file", "Open");
//		button.setFont(new Font("Dialog", Font.PLAIN, 12));
//		button.addActionListener(e -> worldWindAppFrame.newTrackFromFile());
//		toolBar.add(button);

//		// == New Track ==
//		button = makeToolBarButton("24x24-new.gif", "New track", "New");
//		button.addActionListener(e -> worldWindAppFrame.newTrack());
//		toolBar.add(button);

//		// == Save Track ==
//		button = makeToolBarButton("24x24-save.gif", "Save track", "Save");
//		button.addActionListener(e -> {
//			worldWindAppFrame.saveTrack(worldWindAppFrame.getCurrentTrack(), false);
//
//		});
//		toolBar.add(button);

//		// == LookAt ==
//		button = makeToolBarButton(null, "Look At", "LookAt");
//		button.addActionListener(e -> {
//			logger.info("LookAt button clicked.");
//			LookAtTacticsDialog tacticsDialog = new LookAtTacticsDialog(worldWindAppFrame, Dialog.ModalityType.MODELESS,
//					TacticsDialog.EditingActivity.ADD);
//			worldWindAppFrame.setActiveTacticsDialog(tacticsDialog);
//			tacticsDialog.getDialog().addWindowListener(new WindowAdapter() {
//				@Override
//				public void windowClosed(WindowEvent we) {
//					worldWindAppFrame.setActiveTacticsDialog(null);
//				}
//			});
//		});
//		toolBar.add(button);


//		// == Return To Launch =
//		button = makeToolBarButton(null, "Return To Launch", "RTL");
//		button.addActionListener(e -> {
//			logger.info("RTL button clicked.");
//			RtlTacticsDialog tacticsDialog = new RtlTacticsDialog(worldWindAppFrame, Dialog.ModalityType.MODELESS,
//					TacticsDialog.EditingActivity.ADD);
//			worldWindAppFrame.setActiveTacticsDialog(tacticsDialog);
//			tacticsDialog.getDialog().addWindowListener(new WindowAdapter() {
//				@Override
//				public void windowClosed(WindowEvent we) {
//					worldWindAppFrame.setActiveTacticsDialog(null);
//				}
//			});
//		});
//		toolBar.add(button);





//		// == Land ==
//		button = makeToolBarButton(null, "Land", "Land");
//		button.addActionListener(e -> {
//			logger.info("Land button clicked.");
//			LandTacticsDialog tacticsDialog = new LandTacticsDialog(worldWindAppFrame, Dialog.ModalityType.MODELESS,
//					TacticsDialog.EditingActivity.ADD);
//			worldWindAppFrame.setActiveTacticsDialog(tacticsDialog);
//			tacticsDialog.getDialog().addWindowListener(new WindowAdapter() {
//				@Override
//				public void windowClosed(WindowEvent we) {
//					worldWindAppFrame.setActiveTacticsDialog(null);
//				}
//			});
//		});
//		toolBar.add(button);

//		// == Screen Shot ==
//		button = new JButton(worldWindAppFrame.getScreenShotAction());
//		button.setText(null); // Make sure the toolbar button displays only an icon.
//		toolBar.add(button);
//
//		toolBar.addSeparator();

