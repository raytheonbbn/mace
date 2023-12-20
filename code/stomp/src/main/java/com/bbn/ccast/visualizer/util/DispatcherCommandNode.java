//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;


import org.apache.log4j.Logger;
import org.jdesktop.swingx.treetable.AbstractMutableTreeTableNode;
import org.jdesktop.swingx.treetable.MutableTreeTableNode;
import org.jdesktop.swingx.treetable.TreeTableNode;

import javax.swing.tree.TreeNode;
import java.util.ArrayList;
import java.util.Enumeration;

public class DispatcherCommandNode extends AbstractMutableTreeTableNode {
    @SuppressWarnings("unused")
	private static final Logger logger = Logger.getLogger(DispatcherCommandNode.class.getName());
    private String guid = "";
    private ArrayList<DispatcherCommandNode> children = new ArrayList<>();
    private String tacticName = "";
    private String priority = "";
    private String state = "";
    private String progress = "";
    private String agent = "";

    public DispatcherCommandNode(String guid) {
        this.guid = guid;

    }

    public DispatcherCommandNode(String guid, String tacticName, String priority,
                                 String state, String progress, String agent) {
        this.guid = guid;
        this.tacticName = tacticName;
        this.priority = priority;
        this.state = state;
        this.progress = progress;
        this.agent = agent;
    }

    @Override
    public boolean equals(Object o) {
        if(o == null) return false;

        if(o instanceof DispatcherCommandNode) {
            DispatcherCommandNode other = (DispatcherCommandNode)o;
            //Ignore children for clarity in DC panel
   /*         if(other.getChildCount() != this.getChildCount()) return false;

            for(int i = 0; i < this.getChildCount(); i++) {
                DispatcherCommandNode myChild = (DispatcherCommandNode)this.getChildAt(i);
                DispatcherCommandNode otherChild = (DispatcherCommandNode)other.getChildAt(i);
                if(!myChild.getGuid().equalsIgnoreCase(otherChild.getGuid())) return false;
            }*/

            if(this.getParent() != null && other.getParent() != null) {
                if(!this.getParent().getGuid().equalsIgnoreCase(other.getParentGuid())) return false;
            } else if (this.getParent() == null && other.getParent() != null
                    || this.getParent() != null && other.getParent() == null) {
                return false;
            }

            return this.guid.equalsIgnoreCase(other.guid)
                    && this.tacticName.equalsIgnoreCase(other.tacticName)
                    && this.priority.equalsIgnoreCase(other.priority)
                    //Ignore state and progress for now better clarity in DC Panel
                   // && this.state.equalsIgnoreCase(other.state)
                   // && this.progress.equalsIgnoreCase(other.progress)
                    && this.agent.equalsIgnoreCase(other.agent);
        }

        return false;
    }

    @Override
    public int hashCode() {
        int res = 13;

        res += guid.hashCode() + tacticName.hashCode() + priority.hashCode() + agent.hashCode();

        if(parent != null) {
            res += ((DispatcherCommandNode)parent).getGuid().hashCode();
        }

/*        for(AbstractMutableTreeTableNode child : children) {
            DispatcherCommandNode dcnChild = (DispatcherCommandNode)child;
            res += dcnChild.guid.hashCode();
        }*/
        return res;
    }

    public ArrayList<DispatcherCommandNode> getChildren() {
        return  children;
    }

    public boolean addChild(DispatcherCommandNode child) {
        if (this.children == null) {
            this.children = new ArrayList<DispatcherCommandNode>();
        }
        boolean result = this.children.add(child);
        if (result) {
            child.parent = this;
        }
        return result;
    }


    public String getGuid() {
        return guid;
    }

    public String getTacticName() {
        return tacticName;
    }

    @Override
    public int getChildCount() {
        return children.size();
    }

    @Override
	public DispatcherCommandNode getParent() {
        return (DispatcherCommandNode)parent;
    }

    @Override
    public void setParent(MutableTreeTableNode newParent) {
        this.parent = newParent;
    }

    public void removeChild(DispatcherCommandNode node) {
        children.remove(node);
    }

    @Override
    public void removeFromParent() {
        if(getParent() != null) {
            getParent().removeChild(this);
        }

    }

    public void updateTo(DispatcherCommandNode n) {
        this.progress = n.progress;
        this.state = n.state;
        this.agent = n.agent;
        this.priority = n.priority;
    }


    @Override
    public void setValueAt(Object val, int i) {
        String s = (String) val;

        switch (i) {
            case 0:
                guid = s;
                break;
            case 1:
                tacticName = s;
                break;
            case 2:
                priority = s;
                break;
            case 3:
                state = s;
                break;
            case 4:
                progress = s;
                break;
            case 5:
                agent = s;
                break;
        }
    }

    @Override
    public Object getUserObject() {
        return null;
    }

    @Override
    public void setUserObject(Object userObject) {

    }

    @Override
    public int getIndex(TreeNode treeNode) {
        return 0;
    }

    @Override
    public boolean getAllowsChildren() {
        return true;
    }

    @Override
    public boolean isLeaf() {
        return children.size() == 0;
    }

    @Override
    public Enumeration<DispatcherCommandNode> children() {
        return null;
    }

    @Override
    public Object getValueAt(int i) {
        switch (i) {
            case 0:
                return getGuid();
            case 1:
                return getTacticName();
            case 2:
                return getPriority();
            case 3:
                return getState();
            case 4:
                return getProgress();
            case 5:
                return getAgent();

        }
        return null;
    }

    @Override
    public TreeTableNode getChildAt(int childIndex) {
        return children.get(childIndex);
    }

    @Override
    public int getColumnCount() {
        return DispatcherCommandsModel.COLUMN_NAMES.values().length;
    }

    public String getParentGuid() {
        if (parent != null) {
            return  ((DispatcherCommandNode)parent).getGuid();
        }
        return null;
    }

    public String getPriority() {
        return priority;
    }

    public String getState() {
        return state;
    }
    public String getProgress() {
        return progress;
    }
    public String getAgent() {
        return agent;
    }
}
