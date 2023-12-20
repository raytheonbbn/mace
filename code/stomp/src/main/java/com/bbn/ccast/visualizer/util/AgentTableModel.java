//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.AgentTelemPackage;
import com.bbn.ccast.Utils;
import gov.nasa.worldwind.geom.Angle;
import com.bbn.ccast.nullsim.blenodes.PayloadEventHandler;
import com.bbn.ccast.visualizer.WorldWindAppFrame;


import javax.swing.table.AbstractTableModel;
import java.awt.*;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.List;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

public class AgentTableModel extends AbstractTableModel {

	private static final long serialVersionUID = 1L;
	
	public static final long STALE_TIME = 4000;

    private final Map<String, AgentTelemPackage> uidToTelemPackageMap = new ConcurrentHashMap<>();
    public static final DateFormat df = new SimpleDateFormat("HH:mm:ss");

    private WorldWindAppFrame worldWindAppFrame;


    public static String[] COLUMN_NAMES = {
            "UID",
            "Callsign",
            "Time",
            "Altitude",
            "IP Address",
            "Intent(s)"
    };

    public AgentTableModel(WorldWindAppFrame worldWindAppFrame) {
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
		AgentTelemPackage agentTelemPackage = getAgentTelemPackageForRow(rowIndex);
		long timeDiff = System.currentTimeMillis() - agentTelemPackage.getTimestamp();
		
		if (agentTelemPackage.getNeutralized()) {
			return  new Color(0,0,0,128);
		} else if (timeDiff > STALE_TIME) {
			return new Color(255, 128, 0, 128);
		} else if (agentTelemPackage.getBatteryLevel() > 35 && agentTelemPackage.getBatteryLevel() <= 50) {
			return new Color(255, 255, 0, 128);
		} else {
			return Color.WHITE;
		}
	}

    public AgentTelemPackage getAgentTelemPackageForRow(int rowIndex) {
    	
		Set<String> uidSet = uidToTelemPackageMap.keySet();
		List<String> sortedUidList = (new LinkedList<>(uidSet));
		Collections.sort(sortedUidList);
		AgentTelemPackage agentTelemPackage = uidToTelemPackageMap.get(sortedUidList.get(rowIndex));
		return agentTelemPackage;
    }

    @Override
    public Object getValueAt(int rowIndex, int columnIndex) {
        AgentTelemPackage agentTelemPackage = getAgentTelemPackageForRow(rowIndex);
        Object ret;
		switch(columnIndex) {
            case 0:
                ret = agentTelemPackage.getUid();
                break;
            case 1:
                ret = agentTelemPackage.getCallsign();
                break;
            case 2:
                ret = df.format(agentTelemPackage.getTimestamp());
                break;
            case 3:
                double elevation = Utils.EARTH.getElevation(Angle.fromDegreesLatitude(agentTelemPackage.getLatitude()),
                                                            Angle.fromDegreesLongitude(agentTelemPackage.getLongitude()));
                ret = agentTelemPackage.getAltitude() - elevation;
                break;    
            case 4:
                ret = agentTelemPackage.getIpAddress();
                // if (agentTelemPackage.getIsSimPlatform()){
                //     ret = "Sim";
                // }
                break;
            case 5:
                ret = agentTelemPackage.getIntent().toString();
                break;
            default:
                ret = "";
                break;
        }
		return ret;
    }

	public void addOrUpdate(Map<String, AgentTelemPackage> agentTelemPackages) {
		for (AgentTelemPackage telem : agentTelemPackages.values()) {
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

    @Override
    public void setValueAt(Object aValue, int rowIndex, int columnIndex){


        // If changing Callsign
        if (columnIndex == 1){
            String payloadUid = getAgentTelemPackageForRow(rowIndex).getUid();
            PayloadEventHandler payloadEventHandler = this.worldWindAppFrame.getVisualization().getPayloadEventHandler();
            payloadEventHandler.sendPayloadCallsign(payloadUid, aValue.toString());
        }
        
   }

      // This allows certain target table cells to be edited
      @Override
      public boolean isCellEditable(int rowIndex, int columnIndex) {
  
          return columnIndex == 1 || columnIndex == 4;
      }
}
