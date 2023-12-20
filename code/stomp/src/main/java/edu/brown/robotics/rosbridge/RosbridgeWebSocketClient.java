//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package edu.brown.robotics.rosbridge;

import org.apache.log4j.Logger;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;

import javax.json.JsonObject;
import java.net.URI;

public class RosbridgeWebSocketClient extends WebSocketClient {

	private static final Logger logger = Logger.getLogger(RosbridgeWebSocketClient.class.getName());

	protected final RosbridgeClient rosbridgeClient;

	public RosbridgeWebSocketClient(URI serverURI, RosbridgeClient rosbridgeClient) {
		super(serverURI);
		this.rosbridgeClient = rosbridgeClient;
	}

	@Override
	public void onOpen(ServerHandshake handshakedata) {
	}

	@Override
	public void onMessage(String message) {
		rosbridgeClient.messageReceived(message);
	}

	@Override
	public void onClose(int code, String reason, boolean remote) {
		logger.warn("ROS WebSocketClient Close: [" + code + "]: " + reason);
	}

	@Override
	public void onError(Exception ex) {
		logger.warn("ROS WebSocketClient error", ex);
	}

	public void send(JsonObject msg) {
		this.send(msg.toString());
	}

}