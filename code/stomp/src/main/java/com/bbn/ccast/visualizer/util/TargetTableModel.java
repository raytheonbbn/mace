//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.TargetTelemPackage;
import com.bbn.ccast.nullsim.blenodes.TargetEventHandler;

import javax.swing.table.AbstractTableModel;
import java.awt.*;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.List;
import java.util.*;
import com.bbn.ccast.visualizer.WorldWindAppFrame;


public class TargetTableModel extends AbstractTableModel {

	private static final long serialVersionUID = 1L;

	public static final long STALE_TIME = 3000;

    private final Map<String, TargetTelemPackage> uidToTelemPackageMap = new HashMap<>();
    public static final DateFormat df = new SimpleDateFormat("HH:mm:ss");


    public static String[] COLUMN_NAMES = {
            "UID",
            "Callsign",
            "Time",
            "Latitude",
            "Longitude",
            "Type",
            "Payloads",
            "Details",
            "Discovered",
            "Suppression",
            "IP Address"
    };

    private int selectedRow = -1;
    // Add the configuration file to check if white or blue force
    private final com.bbn.ccast.config.Configuration configuration;

    // private TargetEventHandler targetEventHandler;
    private WorldWindAppFrame worldWindAppFrame;

    public TargetTableModel(final com.bbn.ccast.config.Configuration configuration, WorldWindAppFrame worldWindAppFrame) {
        this.configuration = configuration;
        this.worldWindAppFrame = worldWindAppFrame;
    }

    @Override
    public int getRowCount() {
		return uidToTelemPackageMap == null ? 0 : uidToTelemPackageMap.size();
    }

    @Override
    public int getColumnCount() {
        return COLUMN_NAMES.length;
    }

	public Color getRowColor(int rowIndex) {
		TargetTelemPackage targetTelemPackage = getTargetTelemPackageForRow(rowIndex);
		long timeDiff = System.currentTimeMillis() - targetTelemPackage.getTimeReceived().getTime();

		if (selectedRow == rowIndex){
		    return new Color(150,150,255,128);
        }
		else if (targetTelemPackage.isCaptured()) {
			return  new Color(0,0,0,128);
		}
//		else if (agentTelemPackage.getBatteryLevel() <= 35 && agentTelemPackage.getBatteryLevel() > 0) {
//			return new Color(255, 0, 0, 128);
//		} else if (timeDiff > STALE_TIME) {
//			return new Color(255, 128, 0, 128);
//		} else if (agentTelemPackage.getBatteryLevel() > 35 && agentTelemPackage.getBatteryLevel() <= 50) {
//			return new Color(255, 255, 0, 128);
//		}
		else {
			return Color.WHITE;
		}
	}

    public TargetTelemPackage getTargetTelemPackageForRow(int rowIndex) {
		Set<String> uidSet = uidToTelemPackageMap.keySet();
		List<String> sortedUidList = (new LinkedList<>(uidSet));
		Collections.sort(sortedUidList);
		TargetTelemPackage targetTelemPackage = uidToTelemPackageMap.get(sortedUidList.get(rowIndex));
		return targetTelemPackage;
    }

    // Populate column info for targets
    @Override
    public Object getValueAt(int rowIndex, int columnIndex) {
        TargetTelemPackage targetTelemPackage = getTargetTelemPackageForRow(rowIndex);
        Object ret;
        // Check if user is white force or if the target has been discovered before revealing target parameters
        boolean isWhiteForceOrDiscovered = this.configuration.getForceType().equals("white") || targetTelemPackage.getDiscovered();
		switch(columnIndex) {
            case 0:
                ret = targetTelemPackage.getUid();
                break;
            case 1:
                ret = targetTelemPackage.getCallsign();
                break;
            case 2:
                ret = df.format(targetTelemPackage.getTimeReceived());
                break;
            case 3:
                ret = targetTelemPackage.getLatitude();
                break;
            case 4:
                ret = targetTelemPackage.getLongitude();
                break;
            case 5:
                ret = "Unknown";
                if (isWhiteForceOrDiscovered) {
                    ret = targetTelemPackage.getType();
                }
                break;
            case 6:
                ret = "Unknown";
                if (isWhiteForceOrDiscovered) {
                    ret = String.format("%d/%d", targetTelemPackage.getPayloads(), targetTelemPackage.getRequiredPayloads());
                }
                break;
            case 7:
                ret = "Unknown";
                if (isWhiteForceOrDiscovered) {
                    if (targetTelemPackage.getType().equals("LINK")) {
                        ret = targetTelemPackage.getNetworkNames().isEmpty() ? "" : targetTelemPackage.getNetworkNames();
                    } else if (targetTelemPackage.getType().equals("PERI")) {
                        ret = targetTelemPackage.isCaptured() ? "" :
                            targetTelemPackage.getCurrentDuration() == -1 ? "Ready" :
                            targetTelemPackage.getCurrentDuration() == 0 ? "Expired" :
                            String.format("%.2f s", targetTelemPackage.getCurrentDuration());
                    } else { ret = ""; }
                }
                break;
            case 8:
                if (targetTelemPackage.getDiscovered()){
                    return "Discovered";
                }else{
                    return "Undiscovered";
                }
            case 9:
                ret = "Unknown";
                if (isWhiteForceOrDiscovered) {
                    if (!targetTelemPackage.getSuppression()) {
                        ret = "False";
                    }
                    else { ret = "True"; }
                }
                break; 
            case 10:
                ret = targetTelemPackage.getIpAddress();
                break;  
            default:
                ret = "";
                break;
        }
		return ret;
    }

	public void addOrUpdate(Map<String, TargetTelemPackage> targetTelemPackages) {
		for (TargetTelemPackage telem : targetTelemPackages.values()) {
			uidToTelemPackageMap.put(telem.getUid(), telem);
		}
		fireTableDataChanged();
	}

    @Override
    public String getColumnName(int column) {
        return COLUMN_NAMES[column];
    }

    public void clear() {
    	
		uidToTelemPackageMap.clear();        
		fireTableDataChanged();
    }

    public void setSelectedRow(int rowIndex) {
        selectedRow = rowIndex;
    }

    // This contains logic for what happens when a target table cell is edited
    @Override
    public void setValueAt(Object aValue, int rowIndex, int columnIndex){
        String targetUid = getTargetTelemPackageForRow(rowIndex).getUid();
        TargetEventHandler targetEventHandler = this.worldWindAppFrame.getVisualization().getTargetEventHandler();
        targetEventHandler.sendTargetCallsign(targetUid, aValue.toString());
   }

   // This allows certain target table cells to be edited
    @Override
    public boolean isCellEditable(int rowIndex, int columnIndex) {

        return columnIndex == 1;
    }

}
