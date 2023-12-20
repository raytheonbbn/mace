//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast;

import com.bbn.ccast.visualizer.WorldWindAppFrame;
import gov.nasa.worldwind.Factory;
import gov.nasa.worldwind.WorldWind;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.avlist.AVList;
import gov.nasa.worldwind.avlist.AVListImpl;
import gov.nasa.worldwind.cache.FileStore;
import gov.nasa.worldwind.exception.WWRuntimeException;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.globes.ElevationModel;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.terrain.CompoundElevationModel;
import gov.nasa.worldwind.util.*;
import org.apache.log4j.Logger;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

import java.io.File;

public class AgentEarth extends Earth {

	private static final Logger logger = Logger.getLogger(AgentEarth.class.getName());

	private static final double ELEV_EPSILON = 2.0; //m of allowed deviation from expected
    private static final double EXPECTED_TEST_HEIGHT = 85.87; // m derived empirically from default WW model

    private static boolean terrainModelValid = true;

	public AgentEarth() {
		super();
		logger.info("Initializing AgentEarth.");
		loadInstalledDataFromFileStore(WorldWind.getDataFileStore());
		double elev = this.getElevation(Angle.fromDegrees(47.0304), Angle.fromDegrees(-122.5534));
		if((elev < EXPECTED_TEST_HEIGHT - ELEV_EPSILON) || elev > EXPECTED_TEST_HEIGHT + ELEV_EPSILON) {
            setTerrainModelValid(false);
        }
	}

	protected void loadInstalledDataFromFileStore(FileStore fileStore) {
		for (File file : fileStore.getLocations()) {
			if (!file.exists())
				continue;

			if (!fileStore.isInstallLocation(file.getPath()))
				continue;

			logger.info("Loading elevation data from: " + file);
			loadInstalledElevationDataFromDirectory(file);
		}
		
		// Hack to move the base world elevation model to the start since it's somehow ending up at the end of the list
		// and the list is supposedly ordered from lowest to highest resolution.
		ElevationModel defaultElevationModel = this.getElevationModel();
		if (defaultElevationModel instanceof CompoundElevationModel) {
			CompoundElevationModel cem = (CompoundElevationModel) defaultElevationModel;
			ElevationModel baseModel = null;
			for (ElevationModel em : cem.getElevationModels()) {
				if (em.getName().equalsIgnoreCase("USA 10m, World 30m, Ocean 900m")) {
					baseModel = em;
				}	
			}
			if (baseModel != null) {
				cem.removeElevationModel(baseModel);
				cem.addElevationModel(0, baseModel);
			}
		}		
		
		
	}

	// **************************************************************//
	// ************ Loading Previously Installed Elevation Data *****//
	// **************************************************************//

	protected void loadInstalledElevationDataFromDirectory(File dir) {
		String[] names = WWIO.listDescendantFilenames(dir, new DataConfigurationFilter(), false);
		if (names == null || names.length == 0)
			return;

		for (String filename : names) {
			Document doc = null;

			try {
				File dataConfigFile = new File(dir, filename);
				doc = WWXML.openDocument(dataConfigFile);
				doc = DataConfigurationUtils.convertToStandardDataConfigDocument(doc);
			} catch (WWRuntimeException e) {
				e.printStackTrace();
			}

			if (doc == null)
				continue;

			// This data configuration came from an existing file from disk, therefore we
			// cannot guarantee that the
			// current version of World Wind's data installer produced it. This data
			// configuration file may have been
			// created by a previous version of World Wind, or by another program. Set
			// fallback values for any missing
			// parameters that World Wind needs to construct a Layer or ElevationModel from
			// this data configuration.
			AVList params = new AVListImpl();
			WorldWindAppFrame.setFallbackParams(doc, filename, params);
			this.addDataToGlobe(doc.getDocumentElement(), params);

		}
	}

	protected void addDataToGlobe(Element domElement, AVList params) {
		String type = DataConfigurationUtils.getDataConfigType(domElement);
		if (type == null)
			return;

		if (type.equalsIgnoreCase("ElevationModel")) {
			addElevationModelToGlobe(domElement, params, this);
		}
	}

	protected static void addElevationModelToGlobe(Element domElement, AVList params, Globe globe) {
		ElevationModel em = null;
		try {
			Factory factory = (Factory) WorldWind.createConfigurationComponent(AVKey.ELEVATION_MODEL_FACTORY);
			em = (ElevationModel) factory.createFromConfigSource(domElement, params);
		} catch (Exception e) {
			String message = Logging.getMessage("generic.CreationFromConfigurationFailed",
					DataConfigurationUtils.getDataConfigDisplayName(domElement));
			Logging.logger().log(java.util.logging.Level.SEVERE, message, e);
		}

		if (em == null)
			return;

		ElevationModel defaultElevationModel = globe.getElevationModel();
		if (defaultElevationModel instanceof CompoundElevationModel) {
			if (!containsElevationModelByName((CompoundElevationModel) defaultElevationModel, em)) {
			//if (!((CompoundElevationModel) defaultElevationModel).containsElevationModel(em))
				((CompoundElevationModel) defaultElevationModel).addElevationModel(em);
			}
		} else {
			CompoundElevationModel cm = new CompoundElevationModel();
			cm.addElevationModel(defaultElevationModel);
			cm.addElevationModel(em);
			globe.setElevationModel(cm);
		}
	}
	
    public static boolean containsElevationModelByName(CompoundElevationModel defaultElevationModel, ElevationModel em) {
        if (em == null) {
            String msg = Logging.getMessage("nullValue.ElevationModelIsNull");
            Logging.logger().severe(msg);
            throw new IllegalArgumentException(msg);
        }

        for (ElevationModel  model : defaultElevationModel.getElevationModels()) {
        		if (model.getName().equals(em.getName())) {
        			return true;
        		}
            if (model instanceof CompoundElevationModel) {
            		if (containsElevationModelByName((CompoundElevationModel) model, em)) {
            			return true;
            		}
            }
        }

        return false;
    }

    public static boolean isTerrainModelValid() {
        return terrainModelValid;
    }

    public static void setTerrainModelValid(boolean terrainModelValid) {
        AgentEarth.terrainModelValid = terrainModelValid;
    }
}
