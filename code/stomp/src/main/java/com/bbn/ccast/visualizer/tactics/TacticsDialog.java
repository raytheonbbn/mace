//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/**
 *
 */
package com.bbn.ccast.visualizer.tactics;


import com.bbn.ccast.AgentTelemPackage;
import com.bbn.ccast.TargetTelemPackage;
import com.bbn.ccast.visualizer.WorldWindAppFrame;
import com.bbn.ccast.visualizer.WorldWindAppPanel;
import com.bbn.ccast.visualizer.tracks.PositionType;
import com.bbn.ccast.visualizer.tracks.SDOPosition;
import com.bbn.ccast.visualizer.tracks.SDOTrack;
import com.bbn.ccast.visualizer.tracks.TrackController;
import com.bbn.ccast.visualizer.util.*;
import gov.nasa.worldwind.geom.Position;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import javax.swing.*;
import javax.swing.border.Border;
import javax.swing.table.TableColumn;
import java.awt.Dialog.ModalityType;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.List;
import java.util.*;

/**
 * @author ddiller
 *
 */
public abstract class TacticsDialog {

	public enum EditingActivity {
		ADD, EDIT
	}

	public enum DataSources {
		NAMED_OBJECTS, MOVING_OBJECTS, TRACKS, POSITIONS
	}

	public enum OnCompletionActivities {
		LOITER, RTL, LAND, REPEAT
	}

	public static final double MIN_TACTIC_PLAN_PRIORITY = 0.0;
	public static final double MAX_TACTIC_PLAN_PRIORITY = 10.0;

	public static final String POSITION_TEMP_TRACK_NAME = "Position-Not-Track";

	private static final Logger logger = LogManager.getLogger(TacticsDialog.class.getName());

	protected static final double POSSIBLE_OBJECTS_MAX_DISTANCE_FROM_DISPATCHER = 1000.0;

	protected Scanner scanner;

	JDialog dialog;
	String title;
	JComboBox<String> resourceSetTagComboBox;
	CheckedComboBox<?> tagConstraintSetComboBox;
	JPanel plannerParamsPanel;
	JPanel priorityPanel;
	JSpinner tacticPrioritySpinner;
	JFormattedTextField taticNameTextField;
	JSpinner tacticPlanPrioritySpinner;
	ModalityType modalityType;
	EditingActivity editingActivity;
	CheckBoxGroup agentCheckBoxGroup;
	CheckBoxGroup targetCheckBoxGroup;
	JSpinner airWildcardField;
	JSpinner groundWildcardField;
	CheckBoxGroup capabilitiesCheckBoxGroup;
	JCheckBox excludeAgentsFromOtherTasks;
	JTable resourceTable;
	JRadioButton repeatButton;
	JRadioButton rtlButton;
	JRadioButton landButton;
	JRadioButton loiterButton;
	JRadioButton objectButton;
	JRadioButton movingObjectButton;
	JRadioButton trackButton;
	JRadioButton positionButton;
	JTabbedPane tabbedDataPane;
	RadioButtonPanel dataSourceRadioPanel;
	JPanel namedObjectsPanel;
	JPanel movingObjectsPanel;
	JPanel trackPanel;
	SDOPanel positionPanel;
	JPanel preconditionsPanel;
	JComboBox<String> preconditionsSignalComboBox;
	DualListBox namedObjectsListBox;
	JComboBox<SDOTrack> trackComboBox;
	WorldWindAppFrame worldWindAppFrame;
	SDOTrack sdoPositionTrack;
	JComboBox<String> movingObjectsComboBox;

	JPanel customPanel;
	JPanel argPanel;
	String commandName;
	protected JPanel agentPanel;
	protected JPanel targetPanel;
	protected int commandPriority = 0;
	protected boolean includeSelectedFromMap = true;
	protected JPanel dataPanel;

	EnumSet<DataSources> dataSources;
	EnumSet<OnCompletionActivities> onCompletionActivities;

	static String UID_CALLSIGN_DELIMITER = " | ";

	public TacticsDialog() {
	}

	public TacticsDialog(WorldWindAppFrame parent, String title, ModalityType modalityType,
						 EditingActivity editingActivity, String commandName, EnumSet<DataSources> dataSources,
						 EnumSet<OnCompletionActivities> onCompletionActivities) {
		this.worldWindAppFrame = parent;
		this.title = title;
		this.modalityType = modalityType;
		this.editingActivity = editingActivity;
		this.commandName = commandName;
		this.dataSources = dataSources;
		this.onCompletionActivities = onCompletionActivities;

		setup();
	}

	public TacticsDialog(WorldWindAppFrame parent, String title, ModalityType modalityType,
							EditingActivity editingActivity, String commandName, EnumSet<DataSources> dataSources,
							EnumSet<OnCompletionActivities> onCompletionActivities, boolean includeSelectedFromMap) {
		this.worldWindAppFrame = parent;
		this.title = title;
		this.modalityType = modalityType;
		this.editingActivity = editingActivity;
		this.commandName = commandName;
		this.dataSources = dataSources;
		this.onCompletionActivities = onCompletionActivities;
		this.includeSelectedFromMap = includeSelectedFromMap;

		setup();
	}

	protected void setup() {
		this.dialog = new JDialog(this.worldWindAppFrame, title, modalityType);
		this.dataPanel = new JPanel();
		setupCustomPanel();


		if (!this.dataSources.isEmpty() || this.customPanel != null) {

			this.dialog.setPreferredSize(new Dimension(700, 300));

			// if (!(this.dataSources.size() == 1 &&
			// this.dataSources.contains(DataSources.CUSTOM_PANEL))) {
			// Data tabbed pane
			tabbedDataPane = new JTabbedPane();
			// Border tabbedDataPaneBorder = BorderFactory.createTitledBorder("Data Source");
			// tabbedDataPane.setBorder(tabbedDataPaneBorder);
			tabbedDataPane.setPreferredSize(new Dimension(600, 250));

			// Panel for dataradiopanel and oncompletepanel
			dataPanel.setPreferredSize(new Dimension(600, 50));
			dataPanel.setLayout(new BoxLayout(dataPanel, BoxLayout.Y_AXIS));

			// // Named Track ComboBox
			// if (this.dataSources.contains(DataSources.TRACKS)) {
			// 	setupTracksPanel();
			// }

			// Add Postions panel with no need to select it
			setupPositionsPanel();
			tabbedDataPane.add("Positions", positionPanel);

			

			// this.dataSourceRadioPanel = setupDataSourceRadioButtonPanel();



			if (!this.onCompletionActivities.isEmpty()) {
				RadioButtonPanel onCompleteRadioPanel = setupOnCompletionPanel();

				// put repeat and radiopanel in their own panel
				JPanel dataAndOnCompletePanel = new JPanel();
				dataAndOnCompletePanel.setPreferredSize(new Dimension(600, 60));
				dataAndOnCompletePanel.setLayout(new BoxLayout(dataAndOnCompletePanel, BoxLayout.X_AXIS));
				if(dataSourceRadioPanel != null) {
					dataAndOnCompletePanel.add(dataSourceRadioPanel);
				}
				dataAndOnCompletePanel.add(onCompleteRadioPanel);

				if (customPanel != null) {
					dataPanel.add(customPanel);
				}
				dataPanel.add(dataAndOnCompletePanel);
				if(tabbedDataPane != null) {
					dataPanel.add(tabbedDataPane);
				}
			} else {
				if (customPanel != null) {
					dataPanel.add(customPanel);
				}
				if (dataSourceRadioPanel != null) {
					dataPanel.add(dataSourceRadioPanel);
				}
				if (tabbedDataPane != null) {
					dataPanel.add(tabbedDataPane);
				}
			}

		} else {
			this.dialog.setPreferredSize(new Dimension(600, 650));
		}
		this.dialog.setLayout(new GridBagLayout());

		agentPanel = setupAgentsPanel(this.includeSelectedFromMap);
		targetPanel = setupTargetsPanel();
		targetPanel.setVisible(false);

		JPanel resourcesPanel = null;
		JPanel capabilitiesPanel = null;
		JTabbedPane resourcesAndAgentsTabbedPane = null;

		JPanel controlButtonPanel = setupControlButtons();
		// Put panels in the dialog
		GridBagConstraints gbc = new GridBagConstraints();


		layoutAgentsPanel(agentPanel, gbc);
		layoutAgentsPanel(targetPanel, gbc);
		layoutDataPanel(dataPanel, gbc);
		layoutDialogControls(controlButtonPanel, gbc);

		this.dialog.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		this.dialog.addWindowListener(new WindowAdapter() {
			public void windowClosed(WindowEvent we) {
				logger.info("Tactics window closed");
				removeTrack();
			}
		});
		this.dialog.pack();
		this.dialog.setLocationRelativeTo(null);
		this.dialog.setVisible(true);

		this.endOfSetupHook();
	}

	protected JPanel setupSwapReplacement() {
		return null;
	}

	protected void endOfSetupHook() {

	}

	protected void setupCustomPanel() {
	}

	protected void layoutPlannerParamsPanel(JPanel tacticNamePanel1, GridBagConstraints gbc) {
		// preconditions pane
		gbc.gridx = 0;
		gbc.gridy = 0;
		gbc.gridheight = 1;
		gbc.gridwidth = 2;
		gbc.anchor = GridBagConstraints.NORTH;
		gbc.fill = GridBagConstraints.BOTH;
		gbc.weightx = 0.0;
		gbc.weighty = 0.0;

		this.dialog.getContentPane().add(tacticNamePanel1, gbc);

	}

	protected void layoutPreconditionsPanel(JPanel preconditionsPanel1, GridBagConstraints gbc) {
		// preconditions pane
		gbc.gridx = 0;
		gbc.gridy = 4;
		gbc.gridheight = 1;
		gbc.gridwidth = 2;
		gbc.anchor = GridBagConstraints.SOUTH;
		gbc.fill = GridBagConstraints.BOTH;
		gbc.weightx = 0.0;
		gbc.weighty = 0.0;

		this.dialog.getContentPane().add(preconditionsPanel1, gbc);
	}

	protected void layoutDialogControls(JPanel controlButtonPanel, GridBagConstraints gbc) {
		// controls -- execute or cancel
		gbc.gridx = 0;
		gbc.gridy = 5; // was 3
		gbc.gridheight = 1;
		gbc.gridwidth = 2;
		gbc.anchor = GridBagConstraints.SOUTH;
		gbc.fill = GridBagConstraints.NONE;
		gbc.weightx = 0.0;
		gbc.weighty = 0.0;

		this.dialog.getContentPane().add(controlButtonPanel, gbc);
	}

	protected void layoutDataPanel(JPanel dataPanel, GridBagConstraints gbc) {
		if (dataPanel != null) {
			// if (!this.dataSources.isEmpty()) {
			gbc.gridx = 1;
			gbc.gridy = 1;
			gbc.gridheight = 2;
			gbc.gridwidth = 1;
			gbc.anchor = GridBagConstraints.NORTH;
			gbc.fill = GridBagConstraints.BOTH;
			gbc.weightx = 1.0;
			gbc.weighty = 1.0;

			this.dialog.getContentPane().add(dataPanel, gbc);
		}
	}

	protected void layoutResourcesAndAgentsPanel(JTabbedPane resourcesAndAgentsPane, GridBagConstraints gbc) {
		// resources and agents tabbed pane
		gbc.gridx = 0;
		gbc.gridy = 1;
		gbc.gridheight = 2;
		if (!this.dataSources.isEmpty() || this.customPanel != null) {
			gbc.gridwidth = 1;
		} else {
			gbc.gridwidth = 2;
		}
		gbc.anchor = GridBagConstraints.WEST;
		gbc.fill = GridBagConstraints.BOTH;
		gbc.weightx = 1.0;
		gbc.weighty = 1.0;

		this.dialog.getContentPane().add(resourcesAndAgentsPane, gbc);
	}

	protected  void layoutPriorityPanel(JPanel panel, GridBagConstraints gbc){
		if(panel != null) {
			gbc.gridx= 0;
			gbc.gridy= 3;
			gbc.gridheight=1;
			gbc.gridwidth = 2;
			gbc.anchor= GridBagConstraints.NORTHWEST;
			gbc.fill = GridBagConstraints.HORIZONTAL;
			gbc.weightx = 1.0;
			gbc.weighty = 1.0;

			this.dialog.getContentPane().add(panel,gbc);
		}
	}

	protected void layoutAgentsPanel(JPanel agentPanel, GridBagConstraints gbc) {
		// agent pane
		gbc.gridx = 0;
		gbc.gridy = 1;
		gbc.gridheight = 2;
		gbc.gridwidth = 1;
		gbc.anchor = GridBagConstraints.WEST;
		gbc.fill = GridBagConstraints.BOTH;
		gbc.weightx = 0.3;
		gbc.weighty = 0.3;

		this.dialog.getContentPane().add(agentPanel, gbc);
	}

	protected void layoutSwap(JPanel agentPanel, GridBagConstraints gbc) {
		// agent pane
		gbc.gridx = 1;
		gbc.gridy = 1;
		gbc.gridheight = 2;
		gbc.gridwidth = 1;
		gbc.anchor = GridBagConstraints.WEST;
		gbc.fill = GridBagConstraints.BOTH;
		gbc.weightx = 0.3;
		gbc.weighty = 0.3;

		this.dialog.getContentPane().add(agentPanel, gbc);
	}

	protected JPanel setupControlButtons() {
		// Execute and Cancel Button Panel
		JPanel controlButtonPanel = new JPanel();
		if (this.editingActivity.equals(EditingActivity.ADD)) {
			JButton executeButton = new JButton(new AbstractAction("Execute") {
				private static final long serialVersionUID = 1L;

				@Override
				public void actionPerformed(ActionEvent e) {
					boolean res = executeTactic();
					if (res) {
						dialog.setVisible(false);
						if (sdoPositionTrack != null) {
							sdoPositionTrack.firePropertyChange(TrackController.TRACK_REMOVE, null,
									sdoPositionTrack);
						}
						dialog.dispatchEvent(new WindowEvent(dialog, WindowEvent.WINDOW_CLOSING));
					}
				}
			});
			controlButtonPanel.add(executeButton);
		}

		JButton cancelButton = new JButton(new AbstractAction("Cancel") {
			private static final long serialVersionUID = 1L;

			@Override
			public void actionPerformed(ActionEvent e) {
				dialog.setVisible(false);
				if (sdoPositionTrack != null) {
					sdoPositionTrack.firePropertyChange(TrackController.TRACK_REMOVE, null, sdoPositionTrack);
				}
				dialog.dispatchEvent(new WindowEvent(dialog, WindowEvent.WINDOW_CLOSING));
				worldWindAppFrame.getTrackController().setIsRoute(true);
			}
		});
		controlButtonPanel.add(cancelButton);
		return controlButtonPanel;
	}

	protected abstract boolean executeCommand(String[] agentIDs);
	protected abstract boolean executeCommand(String[] agentIDs, SDOTrack sdoPositionTrack);

	protected boolean executeTactic() {
		String[] agentIDs = agentCheckBoxGroup.getSelected();
		if (sdoPositionTrack != null){
			return executeCommand(agentIDs, sdoPositionTrack);
		}else{
			return executeCommand(agentIDs);
		}
	}

	protected RadioButtonPanel setupOnCompletionPanel() {
		Border border;
		JPanel onCompletePanel = new JPanel();
		border = BorderFactory.createTitledBorder("On Completion");
		onCompletePanel.setBorder(border);

		int nbrButtons = this.onCompletionActivities.size();
		JRadioButton[] onCompleteRadioButtonArray = new JRadioButton[nbrButtons];
		nbrButtons = 0;
		if (this.onCompletionActivities.contains(OnCompletionActivities.LOITER)) {
			loiterButton = new JRadioButton("Loiter");
			onCompleteRadioButtonArray[nbrButtons] = this.loiterButton;
			onCompletePanel.add(loiterButton);
		}

		if (this.onCompletionActivities.contains(OnCompletionActivities.RTL)) {
			nbrButtons++;
			rtlButton = new JRadioButton("RTL");
			onCompleteRadioButtonArray[nbrButtons] = this.rtlButton;
			onCompletePanel.add(rtlButton);
		}
		if (this.onCompletionActivities.contains(OnCompletionActivities.LAND)) {
			nbrButtons++;
			landButton = new JRadioButton("Land");
			onCompleteRadioButtonArray[nbrButtons] = this.landButton;
			onCompletePanel.add(landButton);
		}
		if (this.onCompletionActivities.contains(OnCompletionActivities.REPEAT)) {
			nbrButtons++;
			repeatButton = new JRadioButton("Repeat");
			onCompleteRadioButtonArray[nbrButtons] = this.repeatButton;
			onCompletePanel.add(repeatButton);
		}

		RadioButtonPanel onCompleteRadioPanel = new RadioButtonPanel(RadioButtonPanel.HORIZONTAL, "On Completion",
				onCompleteRadioButtonArray);
		onCompleteRadioPanel.setLayout(new FlowLayout());
		if (onCompleteRadioButtonArray.length > 0) {
			onCompleteRadioButtonArray[0].setSelected(true);
		}

		return onCompleteRadioPanel;
	}

	protected RadioButtonPanel setupDataSourceRadioButtonPanel() {
		int nbrButtons = this.dataSources.size();

		JRadioButton[] radioButtonArray = new JRadioButton[nbrButtons];
		nbrButtons = 0;
		if (this.dataSources.contains(DataSources.NAMED_OBJECTS)) {
			tabbedDataPane.add("Named Objects", namedObjectsPanel);
			radioButtonArray[nbrButtons] = objectButton;
			nbrButtons++;
		}
		if (this.dataSources.contains(DataSources.MOVING_OBJECTS)) {
			tabbedDataPane.add("Moving Objects", movingObjectsPanel);
			radioButtonArray[nbrButtons] = movingObjectButton;
			nbrButtons++;
		}
		if (this.dataSources.contains(DataSources.TRACKS)) {
			tabbedDataPane.add("Tracks", trackPanel);
			radioButtonArray[nbrButtons] = trackButton;
			nbrButtons++;
		}
		if (this.dataSources.contains(DataSources.POSITIONS)) {
			tabbedDataPane.add("Positions", positionPanel);
			radioButtonArray[nbrButtons] = positionButton;
			nbrButtons++;
		}

		 RadioButtonPanel radioPanel = new RadioButtonPanel(RadioButtonPanel.HORIZONTAL, "Data Type", radioButtonArray);
		 radioPanel.setLayout(new FlowLayout());
		 if (nbrButtons == 1) {
		 	radioButtonArray[0].setSelected(true);
		 } else {
		 	radioPanel.clearSelection();
		 	tabbedDataPane.removeAll();
		 }
		 return radioPanel;
	}

	protected void setupPositionsPanel() {
		positionPanel = new SDOPanel();
		positionPanel.setPreferredSize(new Dimension(600, 250));
		sdoPositionTrack = new SDOTrack(POSITION_TEMP_TRACK_NAME);
		this.worldWindAppFrame.getTrackController().addTrack(sdoPositionTrack, false);

		positionPanel.getPositionTable().setSDOTrack(sdoPositionTrack);
		positionPanel.getVisibilityFlag().setVisible(false);

		positionButton = new JRadioButton("Position(s)");
		// add allow listener
		positionButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				tabbedDataPane.removeAll();
				tabbedDataPane.add("Positions", positionPanel);
				positionPanel.getVisibilityFlag().setVisible(true);

			}
		});

	}

	protected void setupTracksPanel() {
		trackPanel = new JPanel();
		trackPanel.setPreferredSize(new Dimension(600, 250));
		List<SDOTrack> trackList = worldWindAppFrame.getSdoListPanel().getAllTracks();
		SDOTrack[] trackArray = trackList.toArray(new SDOTrack[0]);
		trackComboBox = new JComboBox<SDOTrack>(trackArray);
		trackComboBox.setPreferredSize(new Dimension(300, 32));
		trackPanel.add(trackComboBox);

		trackButton = new JRadioButton("Track");
		// add allow listener
		trackButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				List<SDOTrack> tList = worldWindAppFrame.getSdoListPanel().getAllTracks();
				SDOTrack[] tArray = tList.toArray(new SDOTrack[0]);
				trackComboBox.removeAllItems();
				for (SDOTrack track : tArray) {
					trackComboBox.addItem(track);
				}
				tabbedDataPane.removeAll();
				tabbedDataPane.add("Tracks", trackPanel);
			}
		});

	}

	protected JPanel setupAgentsPanel() {
		return this.setupAgentsPanel(true);
	}

	protected JPanel setupAgentsPanel(boolean includeSelectedFromMap) {
		// Agents Panel
		JPanel agentPanel = new JPanel();
		agentPanel.setLayout(new BoxLayout(agentPanel, BoxLayout.Y_AXIS));
		Border border = BorderFactory.createTitledBorder("Payloads");
		agentPanel.setBorder(border);
		agentPanel.setPreferredSize(new Dimension(225,400));

		String[] agentAndGroupArray = getAgentsAndGroups();
		agentCheckBoxGroup = new CheckBoxGroup(this.worldWindAppFrame.getVisualization().getScreenSelector(), includeSelectedFromMap,
				"Payloads", agentAndGroupArray);
		agentPanel.add(agentCheckBoxGroup);
		return agentPanel;
	}

	protected JPanel setupTargetsPanel() {
		JPanel targetsPanel = new JPanel();
		targetsPanel.setLayout(new BoxLayout(targetsPanel, BoxLayout.Y_AXIS));
		Border border = BorderFactory.createTitledBorder("Targets");
		targetsPanel.setBorder(border);
		targetsPanel.setPreferredSize(new Dimension(225, 400));

		TargetTableModel tableModel = this.worldWindAppFrame.getTargetPanel().getTableModel();
		String[] targets = new String[tableModel.getRowCount()];
		// The target positions dialog pulls the list of targets from the 0th column of TargetTableModel
		// We want to include the callsign as well for user readability, unless the callsign and uid are the same
		for (int ii = 0; ii < targets.length; ++ii) {
			String targetUid = (String) tableModel.getValueAt(ii, 0);
			String targetCallsign = (String) tableModel.getValueAt(ii, 1);
			if (targetUid.equals(targetCallsign)) { targets[ii] = targetUid; }
			else { targets[ii] = targetUid + " | " + targetCallsign; }
			
		}
		targetCheckBoxGroup = new CheckBoxGroup(this.worldWindAppFrame.getVisualization().getScreenSelector(), false, "Targets", targets);
		targetsPanel.add(targetCheckBoxGroup);
		return targetsPanel;
	}

	protected String[] getAgentsAndGroups() {
		Map<String, AgentTelemPackage> agentTelemPackageMap = worldWindAppFrame.getVisualization().getAgentTelemMap();
		HashSet<String> agentIDs = new HashSet<String>();
		for (AgentTelemPackage atp : agentTelemPackageMap.values()) {
			StringJoiner capJoiner = new StringJoiner(", ");
			String loadout = atp.getLoadout();
			for (String capability : atp.getCapabilities()) {
				capJoiner.add(capability);
			}
			String displayStr = atp.getUid() + CheckBoxGroup.delimeter + " " + loadout;
			String cap = capJoiner.toString();
			if (cap.length() > 0) {
				agentIDs.add(displayStr + CheckBoxGroup.delimeter + " " + cap.trim());
			} else {
				agentIDs.add(displayStr);
			}
		}

		String[] agentAndGroupArray = agentIDs.toArray(new String[0]);
		Arrays.sort(agentAndGroupArray);
		return agentAndGroupArray;
	}

	protected static void setJTableColumnsWidth(JTable table, double... percentages) {
		double tablePreferredWidth = table.getPreferredSize().getWidth();

		double total = 0;
		for (int i = 0; i < table.getColumnModel().getColumnCount(); i++) {
			total += percentages[i];
		}

		for (int i = 0; i < table.getColumnModel().getColumnCount(); i++) {
			TableColumn column = table.getColumnModel().getColumn(i);
			column.setPreferredWidth((int) (tablePreferredWidth * (percentages[i] / total)));
		}
	}

	/**
	 * @return the commandName
	 */
	public String getCommandName() {
		return commandName;
	}

	/**
	 * @param commandName
	 *            the commandName to set
	 */
	public void setCommandName(String commandName) {
		this.commandName = commandName;
	}


	/**
	 * @return the dialog
	 */
	public JDialog getDialog() {
		return dialog;
	}

	/**
	 * @return the dialog
	 */
	public SDOTrack getTrack() {
		return sdoPositionTrack;
	}

	public void removeTrack() {
		if (sdoPositionTrack != null) {
			sdoPositionTrack.firePropertyChange(TrackController.TRACK_REMOVE, null, sdoPositionTrack);
		}
	}
	

	public void appendPositionTypeToTrack(PositionType positionType, Position position) {
		appendPositionTypeToTrack(positionType, position, 0.0, 0.0);
	}

	public void insertPositionTypeToTrack(PositionType positionType, Position position, boolean isAbove) {
		if (sdoPositionTrack != null && positionPanel != null) {
			SDOPosition sdoPos = new SDOPosition(positionType, position.getLatitude(),
					position.getLongitude(), position.getElevation(), 0.0, 0.0);

			int index = positionPanel.getPositionTable().getSelectionModel().getMinSelectionIndex();
			if (!isAbove) {
				index = positionPanel.getPositionTable().getSelectionModel().getMaxSelectionIndex() + 1;
			}

			if (index < 0) {
				sdoPositionTrack.appendPosition(sdoPos);
			} else {
				sdoPositionTrack.add(index, sdoPos);
			}
		}
	}


	public void appendPositionTypeToTrack(PositionType positionType, Position position, double duration, double radius) {
		SDOPosition sdoPos = new SDOPosition(positionType, position.getLatitude(), position.getLongitude(),
				position.getElevation(), duration, radius);
		sdoPositionTrack.appendPosition(sdoPos);
	}

	public void insertPositionTypeToTrack(PositionType positionType, Position position, double duration, double radius, boolean isAbove) {
		if (sdoPositionTrack != null && positionPanel != null) {
			SDOPosition sdoPos = new SDOPosition(positionType, position.getLatitude(), position.getLongitude(),
					position.getElevation(), duration, radius);

			int index = positionPanel.getPositionTable().getSelectionModel().getMinSelectionIndex();
			if (!isAbove) {
				index = positionPanel.getPositionTable().getSelectionModel().getMaxSelectionIndex() + 1;
			}

			if (index < 0) {
				sdoPositionTrack.appendPosition(sdoPos);
			} else {
				sdoPositionTrack.add(index, sdoPos);
			}
		}
	}

	public boolean isPositionDataSelected() {
		if (positionButton != null && positionButton.isSelected()) {
			return true;
		}
		return false;
	}

	public int getPositionMaxSelectionIndex() {
		if (isPositionDataSelected() && positionPanel != null && positionPanel.getPositionTable() != null) {
			return positionPanel.getPositionTable().getSelectionModel().getMaxSelectionIndex();
		}
		return -1;
	}

	public int getPositionMinSelectionIndex() {
		if (isPositionDataSelected() && positionPanel != null && positionPanel.getPositionTable() != null) {
			return positionPanel.getPositionTable().getSelectionModel().getMinSelectionIndex();
		}
		return -1;
	}

	public void setTargetPanelVisibility(boolean visible) {
		targetPanel.setVisible(visible);
	}

	public void setAgentPanelVisibility(boolean visible) {
		agentPanel.setVisible(visible);
	}

	//	Helper method to extract just UIDs from the UID | Callsign list
	public String[] getUidsFromCheckBoxGroupLabels(String[] checkBoxGroupLabels){
		String uidStrings[] = new String[checkBoxGroupLabels.length];
		String label = "";
		for (int i = 0; i < checkBoxGroupLabels.length; i++){
			label = checkBoxGroupLabels[i];
			uidStrings[i] = label.split(UID_CALLSIGN_DELIMITER)[0];
		} 
		return uidStrings;
	}
}
