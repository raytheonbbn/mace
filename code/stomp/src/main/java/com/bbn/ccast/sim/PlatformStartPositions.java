//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.sim;

import com.bbn.ccast.config.Configuration;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import org.apache.log4j.Logger;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class PlatformStartPositions {
	
	private static final Logger logger = Logger.getLogger(PlatformStartPositions.class.getName());
	
	public List<ArrayList<Double>> solos = new ArrayList<ArrayList<Double>>();
	public List<ArrayList<Double>> rovers = new ArrayList<ArrayList<Double>>();
	public ArrayList<Double> origin = new ArrayList<Double>();

	private int startSoloPosition;
	private int startRoverPosition;
	private int endSoloPosition;
	private int endRoverPosition;
	private int nextSoloPosition;
	private int nextRoverPosition;

	public static PlatformStartPositions load(File file) {
		PlatformStartPositions platformStartPositions = null;
		ObjectMapper mapper = new ObjectMapper(new YAMLFactory());

		try {
			platformStartPositions = mapper.readValue(file, PlatformStartPositions.class);
		} catch (Exception e) {
			e.printStackTrace();
		}

		if(platformStartPositions != null) {
			Configuration config = Configuration.instance();
			platformStartPositions.startSoloPosition = config.getSoloStartPositionIndex();
			platformStartPositions.startRoverPosition = config.getRoverStartPositionIndex();
			platformStartPositions.endSoloPosition = config.getSoloEndPositionIndex();
			platformStartPositions.endRoverPosition = config.getRoverEndPositionIndex();

			platformStartPositions.nextSoloPosition = platformStartPositions.startSoloPosition;
			platformStartPositions.nextRoverPosition = platformStartPositions.startRoverPosition;
		}

		return platformStartPositions;
	}
	
	public double[] getOriginPosition() {
		return origin.stream().mapToDouble(Double::doubleValue).toArray(); 
	}
	
	public List<Double> getNextSoloPosition() throws OutOfPlatformPositions {

		if (nextSoloPosition >= solos.size()) {
			throw new OutOfPlatformPositions();
		}
		
		List<Double> result = solos.get(nextSoloPosition);
		nextSoloPosition++;
		return result;
	}

	public List<Double> getNextRoverPosition() throws OutOfPlatformPositions {

		if (nextRoverPosition >= rovers.size()) {
			throw new OutOfPlatformPositions();
		}
		
		List<Double> result = rovers.get(nextRoverPosition);
		nextRoverPosition++;
		return result;
	}

	@Override
	public String toString() {
		StringBuffer sb = new StringBuffer();
		sb.append("Origin: " + origin + "\n");
		sb.append("Solos: \n");
		for (List<Double> coords : solos) {
			sb.append(coords).append("\n");
		}
		sb.append("\nRovers: \n");
		for (List<Double> coords : rovers) {
			sb.append(coords).append("\n");
		}
		return sb.toString();
	}

	public static void main(String[] args) {
		File file = new File("regions/benning/platform-start-positions.yaml");
		PlatformStartPositions positions = load(file);
		System.out.println("Platform start positions:\n\n" + positions);
	}

	public void validateAvailability(int numSolos, int numDownFacingSolos, int numRovers) {

		int lastRoverIndex = Math.min(endRoverPosition, rovers.size() - 1);
		int lastSoloIndex = Math.min(endSoloPosition, solos.size() - 1);

		int roverPositionCount = lastRoverIndex - startRoverPosition + 1;
		int soloPositionCount = lastSoloIndex - startSoloPosition + 1;
		int totalDesiredSoloCount = numSolos + numDownFacingSolos;

		if (soloPositionCount < totalDesiredSoloCount || roverPositionCount < numRovers) {

			logger.error(
					"Not enough platform start positions. \n numSolos + numDownFacingSolos was " + totalDesiredSoloCount
							+ ", and we had " + soloPositionCount + " spawning positions.\n numRovers was " + numRovers
							+ ", and we had " + roverPositionCount + " spawning positions. \nAborting.");

			// Doh!
			System.exit(0);
		}
	}

}
