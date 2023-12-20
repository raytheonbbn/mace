//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import org.jdesktop.swingx.treetable.DefaultTreeTableModel;
import org.jdesktop.swingx.treetable.TreeTableModel;

import java.util.*;

public class DispatcherCommandsModel extends DefaultTreeTableModel implements  TreeTableModel{


    private static long lastSeenTimeout = 2 * 1000;
    public DispatcherCommandsModel() {
        super(new DispatcherCommandNode("root"));
    }


    private HashMap<String, HashMap<String, DispatcherCommandNode>> activeNodes = new HashMap<>();
    private HashMap<String, Long> lastSeenNode = new HashMap<>();

    public void addOrUpdateNode(DispatcherCommandNode n) {
        String agent = n.getAgent();
        String guid = n.getGuid();
        if(activeNodes.get(agent) != null) {
            activeNodes.get(agent).put(guid, n);
        } else {
            HashMap<String, DispatcherCommandNode> tmp = new HashMap<>();
            tmp.put(guid, n);
            activeNodes.put(agent, tmp);
        }
        lastSeenNode.put(n.getGuid(), System.currentTimeMillis());
    }

    public void updateLastSeen(DispatcherCommandNode n){
        lastSeenNode.put(n.getGuid(), System.currentTimeMillis());
    }

    public boolean removeUnusedNodes() {
        boolean res = false;
        //Remove completed Nodes
        Set<String> guidToRemove = new HashSet<>();
        for (Map.Entry<String, HashMap<String, DispatcherCommandNode>> entry : activeNodes.entrySet()) {
            String agent = entry.getKey();
            for (Map.Entry<String, DispatcherCommandNode> entry2 : entry.getValue().entrySet()) {
                DispatcherCommandNode n = entry2.getValue();
                String guid = n.getGuid();
                String stateWithoutWhiteSpace = n.getState().replaceAll("\\s+", "");
 /*               if (TacticStatus.inFinishedState(stateWithoutWhiteSpace)) {
                    guidToRemove.add(guid);
                }*/

                if (lastSeenNode.get(guid) + lastSeenTimeout < System.currentTimeMillis()) {
                    guidToRemove.add(guid);
                }
            }
            for (String guid : guidToRemove) {
                if(activeNodes.get(agent) != null && activeNodes.get(agent).get(guid) != null) {
                    activeNodes.get(agent).get(guid).removeFromParent();
                    activeNodes.get(agent).remove(guid);
                }
                res = true;
            }
        }

        return res;
    }

    public DispatcherCommandNode getNode(String agent, String guid) {
        DispatcherCommandNode n = null;
        if(activeNodes.get(agent) != null) {
            n = activeNodes.get(agent).get(guid);
        }
        if(n == null) {
            return getNode(guid);
        }
        return n;
    }

    public DispatcherCommandNode getNode(String guid) {
        for(HashMap<String, DispatcherCommandNode> map : activeNodes.values()) {
            if(map.get(guid) != null) {
                return map.get(guid);
            }
        }
        return null;
    }

    public enum COLUMN_NAMES  {
        GUID,
        Tactic,
        Priority,
        State,
        Progress,
        Agents;

        public static String[] getValues() {
            String[] ret = new String[COLUMN_NAMES.values().length];
            int i = 0;
            for (COLUMN_NAMES c : values()) {
                ret[i] = c.toString();
                i++;
            }
            return ret;
        }
    }

    @Override
    public int getColumnCount() {
        return COLUMN_NAMES.values().length;
    }

    @Override
    public String getColumnName(int column) {
        return COLUMN_NAMES.getValues()[column];
    }


    @Override
    public Object getValueAt(Object o, int i) {
        DispatcherCommandNode node = (DispatcherCommandNode) o;
        return node.getValueAt(i);
    }

    @Override
    public boolean isCellEditable(Object node, int column) {
        return true;
    }

    @Override
    public void setValueAt(Object value, Object node, int column) {

    }

    public void clear() {
        ((DispatcherCommandNode)root).getChildren().clear();
        activeNodes.clear();

    }

    @Override
    public Object getChild(Object o, int i) {
        ArrayList<DispatcherCommandNode> children = ((DispatcherCommandNode)o).getChildren();
        return children.get(i);
    }

    @Override
    public int getChildCount(Object o) {
        ArrayList<DispatcherCommandNode> children = ((DispatcherCommandNode)o).getChildren();
        return (children == null) ? 0 : children.size();
    }

    @Override
    public int getIndexOfChild(Object o, Object o1) {
        return 0;
    }


}
