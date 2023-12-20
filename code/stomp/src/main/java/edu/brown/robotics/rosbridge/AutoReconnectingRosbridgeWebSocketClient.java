//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package edu.brown.robotics.rosbridge;

import org.apache.log4j.Logger;
import org.java_websocket.handshake.ServerHandshake;

import java.net.URI;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class AutoReconnectingRosbridgeWebSocketClient extends RosbridgeWebSocketClient {

    private static final Logger logger = Logger.getLogger(AutoReconnectingRosbridgeWebSocketClient.class.getName());

    private AtomicBoolean needsReconnect = new AtomicBoolean(false);
    private AtomicInteger reconnectAttemptMillis = new AtomicInteger(8000);

    private Thread reconnectThread = null;
    protected RosClientSubscription.OnConnectedCallback callback = null;

    public AutoReconnectingRosbridgeWebSocketClient(URI uri, RosbridgeClient rosbridgeClient, RosClientSubscription.OnConnectedCallback callback) {
        super(uri, rosbridgeClient);
        this.callback = callback;
    }

    @Override
    public void onOpen(ServerHandshake handshakedata) {
        logger.info("ROS Websocket open");
        needsReconnect.set(false);
        if(this.callback != null) {
            this.callback.onConnected();
        }
    }

    @Override
    public void onClose(int code, String reason, boolean remote) {
        logger.info("ROS Websocket closed because " + reason + "; trying to reconnect");
        needsReconnect.set(true);
        // Build and Start the reconnecting thread if necessary
        synchronized (needsReconnect) {
            if (reconnectThread == null) {
                reconnectThread = new Thread() {
                    @Override
                    public void run() {
                        while (needsReconnect.get()) {
                            try {
                                RosbridgeClient client = AutoReconnectingRosbridgeWebSocketClient.this.rosbridgeClient;
                                if (!client.isOpen()) {
                                    client.connectBlocking();
                                }
                                Thread.sleep(reconnectAttemptMillis.get());
                                if (client.isOpen() && callback != null) {
                                    callback.onConnected();
                                    needsReconnect.set(false);
                                }
                            } catch (Exception e) {
                                logger.warn("Encountered unexpected exception reconnecting to Websocket", e);
                            }
                        }
                    }
                };
                reconnectThread.start();
            }
        }
    }

}
