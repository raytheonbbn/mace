//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package edu.brown.robotics.rosbridge;

import com.bbn.ccast.ros.RosTopicPublisher;
import com.bbn.ccast.ros.RosTopicSubscription;
import org.apache.log4j.Logger;

import javax.json.Json;
import javax.json.JsonObject;
import javax.json.JsonReader;
import java.io.StringReader;
import java.util.HashMap;
import java.util.Map;
import java.util.UUID;

public class RosClientSubscription implements RosTopicSubscription, RosTopicPublisher {
	private static final Logger logger = Logger.getLogger(RosClientSubscription.class.getName());

	private final RosbridgeClient client;

	// Prevent people from calling the constructor
	private RosClientSubscription(RosbridgeClient client) {
		this.client = client;
	}

	public interface JsonMessageCallback {
		public void onNewMessage(JsonObject msg);
	}

	public static RosClientSubscription makeSubscription(String host, int port, String topic, String type,
			JsonMessageCallback callback) {

		RosbridgeClient client = new RosbridgeClient(host, (short) port);

		RosClientSubscription subscription = new RosClientSubscription(client);
		subscription.connect(new OnConnectedCallback() {
			@Override
			public void onConnected() {
				new Thread() {
					@Override
					public void run() {
						// Wait to finish "connecting"
						try {
							Thread.sleep(2000);
						} catch (InterruptedException ie) {
							/* ignore */ }
						subscription.subscribe(topic, callback, type);
					}
				}.start();
			}
		});
		return subscription;
	}

	public static RosClientSubscription makePublisher(String host, int port) {

		RosbridgeClient client = new RosbridgeClient(host, (short) port);

		RosClientSubscription subscription = new RosClientSubscription(client);
		subscription.connect(new OnConnectedCallback() {
			@Override
			public void onConnected() {
				// Do nothing
			}
		});
		return subscription;
	}

	public interface OnConnectedCallback {
		public void onConnected();
	}

	public RosClientSubscription connect(OnConnectedCallback callback) {
		if (!isOpen()) {
			client.connect(callback);
		} else {
			callback.onConnected();
		}
		return this;
	}

	@Override
	public boolean isOpen() {
		return client.isOpen();
	}

	@Override
	public void close() {
		client.close();
	}

	private Map<String, UUID> topicToUuidMap = new HashMap<>();

	public void publish(String topic, String type, JsonObject msg) {
		UUID publishID = null;
		if (topicToUuidMap.containsKey(topic)) {
			publishID = topicToUuidMap.get(topic);
		} else {
			logger.debug("Advertising a new topic/type: " + topic + " & " + type);
			publishID = client.Advertise(topic, type);
			logger.debug("ROS " + topic + " Advertise returned with a UUID: " + publishID);
			topicToUuidMap.put(topic, publishID);
		}
		logger.trace("Publishing to " + publishID);
		client.Publish(publishID, msg);
	}

	@Override
	public void publish(String topic, String type, com.google.gson.JsonObject msg) {
		try {
			JsonObject jsonMsg = null;
			if (msg != null) {
				String json = msg.toString();
				try(JsonReader reader = Json.createReader(new StringReader(json))) {
					jsonMsg = reader.readObject();
				}
			}
			publish(topic, type, jsonMsg);
		} catch (Exception e) {
			logger.info("Failed to publish to topic " + topic + " with json " + msg);
		}
	}

	public static JsonObject parseJson(String json) {
		logger.debug("RosClient parsing json:" + json);
		try (JsonReader jsonReader = Json.createReader(new StringReader(json))) {
			JsonObject object = jsonReader.readObject();
			return object;
		}
	}

	public RosClientSubscription subscribe(String topic, JsonMessageCallback callback, String type) {
		UUID subscriptionID = client.Subscribe(topic, type, new MessageHandler() {
			@Override
			public void messageReceived(String msg) {
				if (callback != null) {
					try (JsonReader jsonReader = Json.createReader(new StringReader(msg))) {
						JsonObject obj = jsonReader.readObject();
						callback.onNewMessage(obj);
					} catch (Exception e) {
						e.printStackTrace();
					}
				}
			}
		});

		topicToUuidMap.put(topic, subscriptionID);
		return this;
	}

	public static void main(String[] args) {

		RosbridgeClient client = new RosbridgeClient("127.0.0.1", (short) 9090).connectBlocking();

		/* UUID subscriptionID = */ client.Subscribe(args[0], args.length > 1 ? args[1] : null, new MessageHandler() {

			@Override
			public void messageReceived(String msg) {

				// System.out.println(msg);

				// /*
				// * {"topic": "/arducamera_tag_detections",
				// * "msg":
				// * {"detection":
				// * {"pose":
				// * {"header":
				// * {"stamp":
				// * {"secs": 1537820083,
				// * "nsecs": 101508832
				// * },
				// * "frame_id": "cam_b",
				// * "seq": 11667
				// * },
				// * "pose":
				// * {"pose":
				// * {"position":
				// * {"y": -0.03889624086005888,
				// * "x": 0.08094095617048976,
				// * "z": 0.5920547084022987
				// * },
				// * "orientation":
				// * {"y": -0.0064821722199432516,
				// * "x": 0.9943788709723165,
				// * "z": 0.008293797064223918,
				// * "w": -0.1053558509879246
				// * }
				// * },
				// * "covariance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				// 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				// 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
				// * }
				// * },
				// * "id": [1],
				// * "size": [0.022]
				// * },
				// * "distance_m": 5.920547084022987,
				// * "azimuth_deg": 97.8576255726028
				// * },
				// * "op": "publish"
				// * }

				try (JsonReader jsonReader = Json.createReader(new StringReader(msg))) {
					JsonObject obj = jsonReader.readObject();

					JsonObject message = obj.getJsonObject("msg");
					JsonObject detection = message.getJsonObject("detection");
					int id = detection.getJsonArray("id").getInt(0);
					double dist = message.getJsonNumber("distance_m").doubleValue();
					double azimuth = message.getJsonNumber("azimuth_deg").doubleValue();

					logger.trace("id = " + id + " @ dist = " + dist + "m & " + azimuth + " degrees");

				} catch (Exception e) {
					e.printStackTrace();
				}

			}
		});

	}

}
