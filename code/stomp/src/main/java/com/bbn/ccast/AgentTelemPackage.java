//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast;

import com.bbn.ccast.nullsim.SimVehicle.Intent;
import com.bbn.ccast.util.MathHelper;
import com.bbn.ccast.visualizer.tracks.SDOPosition;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import org.apache.log4j.Logger;

import java.util.*;

public class AgentTelemPackage implements Cloneable {

	protected static final Logger logger = Logger.getLogger(AgentTelemPackage.class);

	protected static final int DEFAULT_NUM_SATELLITES = -1;
	protected static final double DEFAULT_HEADING = 0.0;
	public static final int DEFAULT_RSSI = 0;

	public static final int NO_SYS_ID = -1;

	public static final long serialVersionUID = 4L;
	protected static final int DEFAULT_FIX_TYPE = -2;

	protected String uid;
	protected Collection<Intent> intent;
	protected Double latitude;
	protected Double longitude;
	protected Double altitude;
	protected Double heading;
	protected Long timestamp;
	protected String loadout;
	protected Set<String> capabilities;
	protected Boolean isSimPlatform;
	protected String ipAddress;
	protected Double batteryLevel;
	protected boolean isOccupied;
	protected Boolean neutralized;
	private MathHelper.LatLon currentGoalPosition;
	private List<SDOPosition> goalPositionList;
	private List<MathHelper.LatLon> currentGoalPositionList;
	private String callsign;




	public static <T> T valueOrDefaultIfNull(T value, T defaultVal) {
		return value == null ? defaultVal : value;
	}

	public AgentTelemPackage(String uid, double latitude, double longitude, double altitude,
							 double heading, long timestamp, String loadout, Set<String> capabilities,
							 Collection<Intent> intent, Boolean isSimPlatform, String ipAddress, double batteryLevel, boolean isOccupied,
							 Boolean isNeutralized, String callsign) {
		this.uid = uid;
		this.latitude = latitude;
		this.longitude = longitude;
		this.altitude = altitude;
		this.heading = heading;
		this.timestamp = timestamp;
		this.loadout = loadout;
		this.capabilities = capabilities;
		this.isSimPlatform = isSimPlatform;
		this.ipAddress = ipAddress;
		this.batteryLevel = batteryLevel;
		this.isOccupied = isOccupied;
		this.neutralized = isNeutralized;
		//this.intent = intent;
		this.callsign = callsign;

	}

	public String getLoadout(){
		return loadout;
	}

	public void setLoadout(String loadout){
		this.loadout = loadout;
	}

	public String getUid() { return uid; }
	public void setUid(String uid) {
		this.uid = uid;
	}

	public double getLatitude() {
		return latitude;
	}
	public void setLatitude(double latitude) {
		this.latitude = latitude;
	}

	public double getLongitude() {
		return longitude;
	}
	public void setLongitude(double longitude) {
		this.longitude = longitude;
	}

	public double getAltitude() {
		return altitude;
	}
	public void setAltitude(double altitude) {
		this.altitude = altitude;
	}

	public double getHeading() {
		return heading;
	}
	public void setHeading(double heading) {
		this.heading = heading;
	}

	public long getTimestamp() {
		return timestamp;
	}
	public void setTimestamp(long timestamp) {
		this.timestamp = timestamp;
	}


	public boolean getOccupied() {
		return isOccupied;
	}

	public Collection<Intent> getIntent() {
		return intent;
	}

	/*public String getIntentToString() {
		StringBuilder intentStr = new StringBuilder();
		int ii = 0;
		for (Intent entry : intent) {
			intentStr.append(entry.name());
			if (ii < intent.size() - 1) {
				intentStr.append(", ");
			}
			ii++;
		}

		return intentStr.toString();
	}*/

	public Set<String> getCapabilities() {return capabilities; }

	@Override
	public int hashCode() {
		return Objects.hash(getUid(), getLatitude(), getLongitude(), getAltitude(), getHeading(),
				getTimestamp(), getOccupied(), getLoadout());
	}

	@Override
	public boolean equals(Object o) {
		if (this == o)
			return true;
		if (o == null || getClass() != o.getClass())
			return false;
		AgentTelemPackage that = (AgentTelemPackage) o;
		return Double.compare(that.getLatitude(), getLatitude()) == 0
				&& Double.compare(that.getLongitude(), getLongitude()) == 0
				&& Double.compare(that.getAltitude(), getAltitude()) == 0
				&& Double.compare(that.getHeading(), getHeading()) == 0 && getTimestamp() == that.getTimestamp()
				&& Objects.equals(getUid(), that.getUid())
				&& Objects.equals(getLoadout(), that.getLoadout())
				&& Objects.equals(getOccupied(), that.getOccupied());
	}


	@Override
	public String toString() {
		String telemPackage = new String();
		telemPackage += "uid:" + uid + ",";
		telemPackage += "callsign:" + callsign + ",";
		telemPackage += "latitude:" + latitude + ",";
		telemPackage += "longitude:" + longitude + ",";
		telemPackage += "altitude:" + altitude + ",";
		telemPackage += "heading:" + heading + ",";
		telemPackage += "timestamp:" + timestamp + ",";
		telemPackage += "loadout:" + loadout + ",";
		return telemPackage;
	}

	public Double getBatteryLevel() {
		return batteryLevel;
	}

	public boolean getIsSimPlatform() { return isSimPlatform; }

	public String getIpAddress() { return this.ipAddress; }

	public Boolean getNeutralized() { return neutralized; }
	public void setNeutralized(Boolean neutralized) { this.neutralized = neutralized; }

	public String toJson()  {
		Gson gson = new GsonBuilder().create();
		return gson.toJson(this);
	}

	public static AgentTelemPackage fromJson(String json){
		Gson gson = new GsonBuilder().create();
		return gson.fromJson(json, AgentTelemPackage.class);
	}

	public MathHelper.LatLon getCurrentGoalPosition() {
		return currentGoalPosition;
	}

	public void setCurrentGoalPosition(MathHelper.LatLon currentGoalPosition) {
		this.currentGoalPosition = currentGoalPosition;
	}

	public List<MathHelper.LatLon> getCurrentGoalPositionList() {
		return currentGoalPositionList;
	}

	public void setCurrentGoalPositionList(List<MathHelper.LatLon> goalPositionList){
		this.currentGoalPositionList = goalPositionList;
	}

	public String getCallsign(){
        return this.callsign;
    }
}
