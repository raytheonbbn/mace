//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;


import com.bbn.ccast.AgentTelemPackage;
import org.apache.log4j.Logger;
import org.jdesktop.swingx.JXTreeTable;

import javax.swing.*;
import javax.swing.event.PopupMenuEvent;
import javax.swing.event.PopupMenuListener;
import javax.swing.tree.TreePath;
import javax.swing.tree.TreeSelectionModel;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.*;

public class DispatcherCommandsPanel extends JPanel {
    @SuppressWarnings("unused")
    private static final Logger logger = Logger.getLogger(DispatcherCommandsPanel.class.getName());

    private static final long serialVersionUID = 1L;



    private JXTreeTable treeTable;
    private DispatcherCommandsModel commandsModel;
    private HashMap<Integer, Boolean> rowIsExpanded = new HashMap<Integer, Boolean>();
    private HashMap<DispatcherCommandNode, Boolean> tacticIsExpanded = new HashMap<>();

    public DispatcherCommandsPanel(DispatcherCommandsModel model) {
        super(new BorderLayout());
        //init();
        this.commandsModel = model;

        setupPanel();
    }

    private void setupPanel() {
        this.treeTable = new JXTreeTable(this.commandsModel);
        this.treeTable.setRootVisible(true);
        this.treeTable.setDragEnabled(false);
        this.treeTable.setDropMode(DropMode.ON_OR_INSERT);
        this.treeTable.getSelectionModel().setSelectionMode(TreeSelectionModel.CONTIGUOUS_TREE_SELECTION);
        expandTree(this.treeTable);

        final JPopupMenu popupMenu = new JPopupMenu();

        popupMenu.addPopupMenuListener(new PopupMenuListener() {
            @Override
            public void popupMenuWillBecomeVisible(PopupMenuEvent e) {
                SwingUtilities.invokeLater(new Runnable() {
                    @Override
                    public void run() {
                        int rowAtPoint = treeTable.rowAtPoint(SwingUtilities.convertPoint(popupMenu, new Point(0, 0), treeTable));
                        if (rowAtPoint > -1) {
                            treeTable.setRowSelectionInterval(rowAtPoint, rowAtPoint);
                        }
                    }
                });
            }

            @Override
            public void popupMenuWillBecomeInvisible(PopupMenuEvent popupMenuEvent) {

            }

            @Override
            public void popupMenuCanceled(PopupMenuEvent popupMenuEvent) {

            }
        });

        JMenuItem stopTactic = new JMenuItem("Stop Tactic");
        stopTactic.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent actionEvent) {
                int selectedRow = treeTable.getSelectedRow();
                TreePath path = treeTable.getPathForRow(selectedRow);
                DispatcherCommandNode node = (DispatcherCommandNode) path.getLastPathComponent();

                logger.info("Stop " + node.getGuid());

//                DispatcherCommand dc = new DispatcherCommand.Builder(Stop.getCommandName(), new String[] {node.getAgent()})
//                        .build();
//                dc.putArgument("guid", node.getGuid());
//                WorldWindAppFrame frame = WorldWindAppFrame.getInstance();
//                frame.sendDispatcherCommand(dc);
            }
        });

        JMenuItem pauseTactic = new JMenuItem("Pause/Unpause Tactic");
        pauseTactic.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent actionEvent) {
                int selectedRow = treeTable.getSelectedRow();
                TreePath path = treeTable.getPathForRow(selectedRow);
                DispatcherCommandNode node = (DispatcherCommandNode) path.getLastPathComponent();

                logger.info("Pause " + node.getGuid());

//                DispatcherCommand dc = new DispatcherCommand.Builder(Pause.getCommandName(), new String[] {node.getAgent()})
//                        .build();
//                dc.putArgument("guid", node.getGuid());
//                WorldWindAppFrame frame = WorldWindAppFrame.getInstance();
//                frame.sendDispatcherCommand(dc);
            }
        });

        popupMenu.add(stopTactic);
        popupMenu.add(pauseTactic);
        treeTable.setComponentPopupMenu(popupMenu);
    }

    /**
     * Collapse tree.
     *
     * @param tree the tree
     */
    @SuppressWarnings("unused")
    private void collapseTree(JXTreeTable tree) {
        for (int i = 0; i < tree.getRowCount(); i++) {
            tree.collapseRow(i);
        }
    }
    public enum COLUMN_NAMES  {
        GUID,
        Tactic,
        Origin,
        Priority,
        State,
        Progress,
        Payloads;

        public static String[] getValues() {
            String[] ret = new String[DispatcherCommandsModel.COLUMN_NAMES.values().length];
            int i = 0;
            for (COLUMN_NAMES c : values()) {
                ret[i] = c.toString();
                i++;
            }
            return ret;
        }
    }

    /**
     * Expand tree.
     *
     * @param tree the tree
     */
    private void expandTree(JXTreeTable tree) {
        for (int i = 0; i < tree.getRowCount(); i++) {
            tree.expandRow(i);
        }
    }

    private void setExpandedRows() {
        for(int i = 0; i < treeTable.getRowCount(); i++ ){
            rowIsExpanded.put(i,treeTable.isExpanded(i));

            TreePath path = treeTable.getPathForRow(i);
            DispatcherCommandNode node = (DispatcherCommandNode) path.getLastPathComponent();
            tacticIsExpanded.put(node, treeTable.isExpanded(i));
        }

        ArrayList<Integer> rowstoRemove = new ArrayList<>();
        for(Integer i : rowIsExpanded.keySet()) {
            if(i > treeTable.getRowCount()) {
                rowstoRemove.add(i);
                TreePath path = treeTable.getPathForRow(i);
                if(path != null) {
                    DispatcherCommandNode node = (DispatcherCommandNode) path.getLastPathComponent();
                    tacticIsExpanded.remove(node);
                }

            }
        }

        for(Integer i : rowstoRemove) {
            rowIsExpanded.remove(i);
        }
    }

    private void updateExpandedRows() {
        for(int i = 0; i < treeTable.getRowCount(); i++) {
            TreePath path = treeTable.getPathForRow(i);
            DispatcherCommandNode node = (DispatcherCommandNode) path.getLastPathComponent();
            if(node.getGuid().equalsIgnoreCase("root") || (tacticIsExpanded.get(node) != null && tacticIsExpanded.get(node))) {
                treeTable.expandRow(i);
            } else {
                treeTable.collapseRow(i);
            }
/*            if(rowIsExpanded.get(i) != null && rowIsExpanded.get(i)) {
                treeTable.expandRow(i);
            } else {
                treeTable.collapseRow(i);
            }*/
        }
    }

//    private void findTacticParent(ProtelisDevice device, String tacticGUID, String tacticOrigin,
//                                  Map<String, Set<String>> seenParentGUIDsToChildren, DispatcherCommandNode root,
//                                  DispatcherCommandNode node, String uid) {
//        DispatcherCommand dc = device.getCommandByGUID(tacticGUID);
//        if (dc != null) {
//            tacticOrigin = dc.getParentGUID() == null ? "root" : dc.getParentGUID();
//            if (seenParentGUIDsToChildren.containsKey(tacticOrigin)) {
//                seenParentGUIDsToChildren.get(tacticOrigin).add(node.getGuid() + ":" + uid);
//            } else {
//                HashSet<String> tmp = new HashSet<>();
//                tmp.add(node.getGuid() + ":" + uid);
//                seenParentGUIDsToChildren.put(tacticOrigin, tmp);
//            }
//        } else {
//            CommandType cmd = device.getCommandByGUID(tacticGUID);
//            if (cmd != null) {
//                tacticOrigin = cmd.getParentGuid() == null ? "root" : cmd.getParentGuid();
//                if (seenParentGUIDsToChildren.containsKey(tacticOrigin)) {
//                    seenParentGUIDsToChildren.get(tacticOrigin).add(node.getGuid() + ":" + uid);
//                } else {
//                    HashSet<String> tmp = new HashSet<>();
//                    tmp.add(node.getGuid() + ":" + uid);
//                    seenParentGUIDsToChildren.put(tacticOrigin, tmp);
//                }
//            }else {
//                tacticOrigin = "root";
//                if (seenParentGUIDsToChildren.containsKey(tacticOrigin)) {
//                    seenParentGUIDsToChildren.get(tacticOrigin).add(node.getGuid() + ":" + uid);
//                } else {
//                    HashSet<String> tmp = new HashSet<>();
//                    tmp.add(node.getGuid() + ":" + uid);
//                    seenParentGUIDsToChildren.put(tacticOrigin, tmp);
//                }
//            }
//        }
//    }

    public void addOrUpdateAll(
//            ProtelisDevice device,
            Map<String, AgentTelemPackage> tacticsFromAll) {
        commandsModel.clear();
        setExpandedRows();
        Map<String, Set<String>> seenParentGUIDsToChildren = new HashMap<>();
        DispatcherCommandNode root = (DispatcherCommandNode) commandsModel.getRoot();
        for(String uid : tacticsFromAll.keySet()) {
            AgentTelemPackage telemPackage = tacticsFromAll.get(uid);
//            for(String tacticToSplit : tactics) {
//                if(tacticToSplit != null && !tacticToSplit.toLowerCase().contains("continuousapril")) {
//                    Tactic.TacticDataStruct data = Tactic.TacticDataStruct.fromString(tacticToSplit);
//
//                    String tacticGUID = data.GUID;
//                    String tacticName  = data.name;
//                    String tacticPriority  = String.valueOf(data.priority);
//                    String tacticState = data.state;
//                    String tacticprogress = data.progress;
//                    String tacticOrigin = "";
//                    Tactic t = device.getTacticConfigFromName(tacticName);
//
//                    if (t == null){
//                        logger.info("Skipping unknown tactic: " + tacticName);
//                        continue;
//                    }
//
//                    if (t.isDaemon()) {
//                        continue;
//                    }
//
//                    DispatcherCommandNode node = new DispatcherCommandNode(tacticGUID,
//                            tacticName, tacticPriority, tacticState, tacticprogress, uid);
//                    DispatcherCommandNode oldNode = commandsModel.getNode(uid, tacticGUID);
//
//                    //Node has not been updated, so no need to updateUI
//                    if(!node.equals(oldNode) && oldNode != null) {
//                        if(!node.getAgent().equalsIgnoreCase(oldNode.getAgent())) {
//                            commandsModel.addOrUpdateNode(node);
//                        } else {
//                            oldNode.updateTo(node);
//                            commandsModel.updateLastSeen(node);
//                        }
//
//                    } else {
//                        commandsModel.addOrUpdateNode(node);
//                    }
//
//                    findTacticParent(device, tacticGUID, tacticOrigin, seenParentGUIDsToChildren, root, node, uid);
//                }
//            }
        }

        for(Map.Entry<String, Set<String>> entry : seenParentGUIDsToChildren.entrySet()) {
            String parentGUID = entry.getKey();
            for(String child : entry.getValue()) {
                String[] arr = child.split(":");
                String guid = arr[0];
                String uid = arr[1];
                DispatcherCommandNode parentNode = parentGUID.equalsIgnoreCase("root") ? (DispatcherCommandNode) commandsModel.getRoot() : commandsModel.getNode(uid,parentGUID);
                if(parentNode == null) {
                    parentNode = (DispatcherCommandNode) commandsModel.getRoot();
                }
                DispatcherCommandNode childNode = commandsModel.getNode(uid, guid);
                if(parentNode != null && !parentNode.getChildren().contains(childNode)) {
                    addObject(parentNode, commandsModel.getNode(uid, guid));
                }
            }
        }

        Collections.sort(root.getChildren(), nameAsComp);
        updateTable();
    }

    private void updateTable() {
        treeTable.updateUI();
        treeTable.expandAll();
        updateExpandedRows();
    }

    public void addObject(DispatcherCommandNode parent, DispatcherCommandNode child) {
        if(parent == null) {
            return;
        }
        parent.addChild(child);
        child.setParent(parent);
    }

    private void addTestData(DispatcherCommandsModel dcm, DispatcherCommandsPanel mpp) {

        DispatcherCommandNode root = (DispatcherCommandNode) dcm.getRoot();
        DispatcherCommandNode child1 = new DispatcherCommandNode("1", "1", "1", "Active", "Active", "Me");
        mpp.addObject(root, child1);
        DispatcherCommandNode child2 = new DispatcherCommandNode("2", "2", "1", "Active", "Active", "Me");
        mpp.addObject(root, child2);

        DispatcherCommandNode child1a = new DispatcherCommandNode("1a", "1a", "1", "Active", "Active", "Me");
        mpp.addObject(child1, child1a);

        DispatcherCommandNode child1aa = new DispatcherCommandNode("1aa", "1aa", "1", "Active", "Active", "Me");
        mpp.addObject(child1a, child1aa);

        DispatcherCommandNode child2a = new DispatcherCommandNode("2a", "2a", "1", "Active", "Active", "Me");
        mpp.addObject(child2, child2a);

    }

    private void doTest(DispatcherCommandsModel dcm, DispatcherCommandsPanel mpp) {
        for(int i = 0; i < 100; i++) {
            try {
                Thread.sleep(100);
                dcm.clear();
                mpp.getTreeTable().updateUI();
            } catch (Exception e) {

            }
            try {
                Thread.sleep(100);
                mpp.addTestData(dcm, mpp);
                mpp.getTreeTable().updateUI();
            } catch (Exception e) {

            }
        }
    }


    public static void main(String[] args) {

        JFrame f = new JFrame("DC_Test");
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        DispatcherCommandsModel dcm = new DispatcherCommandsModel();

        DispatcherCommandsPanel mpp = new DispatcherCommandsPanel(dcm);
        JScrollPane newScrollPane = new JScrollPane(mpp.getTreeTable());
        f.add(newScrollPane);
        f.setSize(400, 400);
        f.setLocation(200, 200);
        f.setVisible(true);

        mpp.doTest(dcm, mpp);
    }

    public JXTreeTable getTreeTable() {
        return treeTable;
    }

    /* Comparator for sorting the list by info, ascending order */
    public static Comparator<DispatcherCommandNode> nameAsComp = new Comparator<DispatcherCommandNode>() {

        @Override
        public int compare(DispatcherCommandNode m1, DispatcherCommandNode m2) {
            String id1 = m1.getTacticName();
            String id2 = m2.getTacticName();

            // ascending order
            return id1.compareTo(id2);
        }
    };
}
