//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tracks;

import java.io.IOException;
import java.io.InputStream;

public class MAVLinkTrackReader extends AbstractTrackReader {
	public MAVLinkTrackReader() {
	}

	@Override
	public String getDescription() {
		return "MAVLink Track (*.mavlink)";
	}

	protected SDOTrack[] doRead(InputStream inputStream, String name) throws IOException {
		MAVLinkReader reader = new MAVLinkReader();
		reader.readStream(inputStream, name); 
		return this.asArray(reader.getTracks());
	}

	@Override
	protected SDOTrack[] doRead(InputStream inputStream) throws IOException {
		MAVLinkReader reader = new MAVLinkReader();
		reader.readStream(inputStream, null); // un-named stream
		return this.asArray(reader.getTracks());
	}

	@Override
	protected boolean acceptFilePath(String filePath) {
		return filePath.toLowerCase().endsWith(".mavlink");
	}
}
