//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tracks;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Position.PositionList;
import gov.nasa.worldwind.render.Path;

public class LookFrustumPath extends Path {
		
	public LookFrustumPath() {
	}

	public LookFrustumPath(Path source) {
		super(source);
	}

	public LookFrustumPath(Iterable<? extends Position> positions) {
		super(positions);
	}

	public LookFrustumPath(PositionList positions) {
		super(positions);
	}

	public LookFrustumPath(Position posA, Position posB) {
		super(posA, posB);
	}
}
