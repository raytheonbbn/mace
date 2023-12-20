//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/**
 *
 * @author Stephen Brawner
 * Brown University
 *
 */

package edu.brown.robotics.rosbridge;

import com.bbn.ccast.ros.RosServiceInterface;
import com.google.gson.JsonElement;
import com.google.gson.JsonParser;
import org.apache.log4j.Logger;

import javax.json.*;
import java.io.StringReader;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.*;
import java.util.function.Consumer;

/**
 * The RosbridgeClient class represents the client that an application will
 * interact with. This class contains an example with a main method.
 */
public class RosbridgeClient implements MessageHandler, RosServiceInterface {

	private final String url;
	private final int port;

	private static final Logger logger = Logger.getLogger(RosbridgeClient.class);

	/**
	 * Main method for which serves as an example and test of the class.
	 *
	 * @param args Command line arguments.
	 */
	public static void main(String[] args) {
		RosbridgeClient client = new RosbridgeClient("127.0.0.1", (short) 9090).connectBlocking();
		int wait = 0;
		try {
			while (!client.client.getConnection().isOpen() && wait++ < 100) {
				System.out.println("Waiting to connect: " + wait);
				Thread.sleep(1);
			}
		} catch (Exception e) {
			System.err.println("Error sleeping");
		}

		System.out.println("Calling service...");
		client.CallService("/move_enu_Solo_0", Json.createArrayBuilder().add(1.0).add(2.0).add(3.0).build(),
				new MessageHandler() {

					@Override
					public void messageReceived(String msg) {
						System.out.println("Received service call result: " + msg);
					}
				});

		// client.StatusMessage("Connecting to rosbridge server", "info");
		UUID subscribeID = client.Subscribe("/something", "std_msgs/String", client);

		UUID publishID = client.Advertise("something", "std_msgs/String");
		for (int i = 0; i < 10; i++) {
			JsonObject obj = Json.createObjectBuilder().add("data", Integer.toString(i)).build();
			client.Publish(publishID, obj);
		}

		try {
			Thread.sleep(4000);
		} catch (InterruptedException e) {
			System.err.println("Thread failed sleeping");
			e.printStackTrace();
		}
		client.Unsubscribe(subscribeID);
		client.Unadvertise(publishID);
		client.close();
	}

	/**
	 * A set of unique ids for subscribers and publishers
	 */
	private HashSet<UUID> ids;

	/**
	 * A map of ids to MessageHandler callbacks. All services and topics require
	 * callbacks, and this serves as a lookup
	 */
	private HashMap<UUID, MessageHandler> handlers;

	/**
	 * Map from service types to handlers.
	 */
	private HashMap<String, MessageHandler> serviceHandlers;

	/**
	 * For any lookup that requires getting the name of a topic that we're
	 * publishing to
	 */
	private HashMap<UUID, String> publishedTopics;

	/**
	 * For any lookup that requires getting the name of a topic that we're
	 * subscribing to
	 */
	private HashMap<UUID, String> subscribedTopics;

	/**
	 * This is a map of topics to subscriber IDs. Subscriber messages don't return
	 * an ID, which makes calling their lookup by our UUID method tricky. Instead we
	 * lookup the UUID's that are associated with a given topic
	 */
	private HashMap<String, ArrayList<UUID>> subscriberLookup;

	/**
	 * This is a map of topics to types, saves service calls to /rosapi/topic_type
	 */
	private HashMap<String, String> topicTypes;

	/**
	 * Our implementation of the WebSocketClient
	 */
	private RosbridgeWebSocketClient client;

	/**
	 * Constructor
	 *
	 * @param url
	 *            IP4 url of the rosbridge server
	 * @param port
	 *            Desired port, typical default is 9090
	 */
	public RosbridgeClient(String url, short port) {
		this.url = url;
		this.port = port;
		this.ids = new HashSet<UUID>();
		this.handlers = new HashMap<UUID, MessageHandler>();
		this.serviceHandlers = new HashMap<>();
		this.publishedTopics = new HashMap<UUID, String>();
		this.subscribedTopics = new HashMap<UUID, String>();
		this.subscriberLookup = new HashMap<String, ArrayList<UUID>>();
		this.topicTypes = new HashMap<String, String>();
	}

	private static Map<URI, RosbridgeWebSocketClient> clientMap = new HashMap<>();

	private static RosbridgeWebSocketClient getOrCreateClient(URI uri, RosbridgeClient c, RosClientSubscription.OnConnectedCallback callback) {
		RosbridgeWebSocketClient client = null; //clientMap.get(uri);  // CANNOT REUSE, so always return a new one
		if(client == null) {
			client = new AutoReconnectingRosbridgeWebSocketClient(uri, c, callback);
			clientMap.put(uri, client);
		}
		return client;
	}

	public synchronized RosbridgeClient connect(RosClientSubscription.OnConnectedCallback callback) {
		RosbridgeWebSocketClient newClient = null;
		try {
			String uriStr = "ws://" + url + ":" + port + "/";
			logger.info("Connecting to " + uriStr);
			newClient = getOrCreateClient(new URI(uriStr), this, callback);
		} catch (URISyntaxException e) {
			System.err.println("URI improperly formed. URL: " + url + " port: " + port);
			e.printStackTrace();
		}

		this.client = newClient;
		if(!isOpen()) {
			this.client.connect();
		}
		return this;
	}

	public RosbridgeClient connectBlocking() {
		RosbridgeWebSocketClient newClient = null;
		try {
			newClient = new RosbridgeWebSocketClient(new URI("ws://" + url + ":" + port + "/"), this);
		} catch (URISyntaxException e) {
			System.err.println("URI improperly formed. URL: " + url + " port: " + port);
			e.printStackTrace();
		}

		this.client = newClient;
		try {
			this.client.connectBlocking();
		} catch (InterruptedException e) {
			// ignore
		}
		return this;
	}

	/**
	 * Generates a unique UUID for subscriber/publisher lookups
	 *
	 * @return The generated UUID
	 */
	private UUID createUniqueID() {
		UUID id = UUID.randomUUID();
		while (this.ids.contains(id)) {
			id = UUID.randomUUID();
		}
		this.ids.add(id);
		return id;
	}

	/**
	 * @return whether the internal WebSocket client is open.
	 */
	@Override
	public boolean isOpen() {
		return client != null && client.getConnection() != null && client.getConnection().isOpen();
	}

	/**
	 * Callback for the WebSocketClient, this method will call the proper subscriber
	 * or service callbacks for each message. The string parameter msg is converted
	 * into JSONObject and the message field is passed to the proper callback.
	 *
	 * @param msg
	 *            The received message as a string
	 * //@throws JSONException
	 *             Thrown if JSON parsing fails
	 */
	@Override
	public void messageReceived(String msg) {

		ArrayList<UUID> handlerIds = new ArrayList<UUID>();

		try (JsonReader jsonReader = Json.createReader(new StringReader(msg))) {
			JsonObject obj = jsonReader.readObject();

			// Service calls are a bit of a special case, since ROS does not provide proper
			// UUIDs.
			// So if this is a service call, dispatch it and be done.
			if (obj.containsKey("op") && obj.getString("op").equals("call_service")) {
				if (obj.containsKey("service")) {
					String serviceName = obj.getString("service");
					serviceHandlers.get(serviceName).messageReceived(msg);
				}
			} else {
				if (obj.containsKey("id")) {
					String idStr = obj.get("id").toString();
					idStr = idStr.replace("\"", "");
					handlerIds.add(UUID.fromString(idStr));
				} else {
					String subscriber = "";
					if (obj.containsKey("topic")) {
						subscriber = obj.get("topic").toString().replaceAll("\"", "").replaceAll("'", "");
					} else if (obj.containsKey("service")) {
						subscriber = obj.get("service").toString().replaceAll("\"", "").replaceAll("'", "");
					}

					// System.out.println("Subscriber = " + subscriber);
					if (subscriber != "") {
						// System.out.println("Subscriber list = " + this.subscriberLookup.keySet());
						ArrayList<UUID> subscribers = this.subscriberLookup.get(subscriber);
						if (subscribers != null) {
							handlerIds.addAll(subscribers);
						} else {
							// System.err.println("No subscribers for this msg: " + msg);
						}
					}
				}

				for (UUID id : handlerIds) {
					// System.out.println("Sending to handler for id=" + id);
					MessageHandler messageHandler = this.handlers.get(id);
					if (messageHandler != null && messageHandler != this) {
						messageHandler.messageReceived(msg);
					}
				}

			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Creates a subscription to a topic on the ROS system.
	 *
	 * @param topic
	 *            Topic to subscribe to on ROS system
	 * @param type
	 *            The type of message of this topic.
	 * @param handler
	 *            The method to be called for each received message
	 * @return A uniquely generated ID for this subscription
	 */
	public UUID Subscribe(String topic, String type, MessageHandler handler) {

		UUID id = this.createUniqueID();

		JsonObject call;

		// 'type' is optional - but don't add it if it's null
		if (type != null) {
			call = Json.createObjectBuilder().add("op", "subscribe").add("id", id.toString()).add("topic", topic)
					.add("type", type).build();
		} else {
			call = Json.createObjectBuilder().add("op", "subscribe").add("id", id.toString()).add("topic", topic)
					.build();
		}

		this.client.send(call);
		this.subscribedTopics.put(id, topic);
		this.handlers.put(id, handler);

		this.subscriberLookup.putIfAbsent(topic, new ArrayList<UUID>());
		this.subscriberLookup.get(topic).add(id);
		return id;
	}

	/**
	 * Unsubscribes from a topic on the ROS system
	 *
	 * @param id
	 *            The unique id of the subscriber that is to be unsubscribed
	 */
	public void Unsubscribe(UUID id) {
		JsonObject call = Json.createObjectBuilder().add("op", "unsubscribe").add("id", id.toString())
				.add("topic", this.subscribedTopics.get(id)).build();
		if (this.handlers.containsKey(id)) {
			this.handlers.remove(id);
		}
		if (this.subscribedTopics.containsKey(id)) {
			String topic = this.subscribedTopics.get(id);
			this.subscribedTopics.remove(id);
			if (this.subscriberLookup.containsKey(topic)) {
				this.subscriberLookup.get(topic).remove(id);
			}
		}
		this.client.send(call.toString());
	}

	/**
	 * This notifies the ROS system that a topic will be published to. It requires
	 * both a topic name and a topic type
	 *
	 * @param topic
	 *            The name of the topic
	 * @param type
	 *            The ROS message type of this topic
	 * @return The unique ID of this publisher.
	 */
	public UUID Advertise(String topic, String type) {
		UUID id = this.createUniqueID();

		this.publishedTopics.put(id, topic);
		this.topicTypes.put(topic, type);

		JsonObject call = Json.createObjectBuilder().add("op", "advertise").add("topic", topic).add("type", type)
				.add("id", id.toString()).build();

		this.client.send(call.toString());
		return id;
	}

	/**
	 * Notifies the ROS system that this topic will no longer be published to
	 *
	 * @param id
	 *            The unique ID of this publisher
	 */
	public void Unadvertise(UUID id) {
		JsonObject call = Json.createObjectBuilder().add("op", "unadvertise").add("id", id.toString())
				.add("topic", this.publishedTopics.get(id)).build();

		if (this.publishedTopics.containsKey(id)) {
			this.publishedTopics.remove(id);
		}
		this.client.send(call.toString());
	}

	public void AdvertiseService(String serviceName, String serviceType, MessageHandler handler) {

		serviceHandlers.put(serviceName, handler);

		JsonObject call = Json.createObjectBuilder().add("op", "advertise_service").add("service", serviceName)
				.add("type", serviceType).build();

		this.client.send(call.toString());
	}

	public void ServiceResponse(String serviceName, String id, JsonObject values, boolean result) {

		JsonObject call = Json.createObjectBuilder().add("op", "service_response").add("service", serviceName)
				.add("id", id).add("values", values).add("result", result).build();

		this.client.send(call.toString());
	}

	public void UnadvertiseService(String serviceName) {

		JsonObject call = Json.createObjectBuilder().add("op", "unadvertise_service").add("service", serviceName)
				.build();

		this.client.send(call.toString());
	}
	/**
	 * Publishes a message to a given topic. With this implementation, the topic
	 * must first be advertised
	 *
	 * @param id
	 *            The unique ID returned from the Advertise method
	 * @param obj
	 *            The JSONObject to publish
	 */
	public void Publish(UUID id, JsonObject obj) {
		//System.out.println("pub " + id + " : " + obj + " : " + this.publishedTopics.get(id));
		JsonObject call = Json.createObjectBuilder().add("op", "publish").add("topic", this.publishedTopics.get(id))
				.add("msg", obj).build();

		this.client.send(call.toString());
	}

	/**
	 * Calls a service on the ROS system with possibly arguments for the request
	 * message. The message will be called with the message upon receipt of the
	 * message service response
	 *
	 * @param service   The name of the service to call
	 * @param args      The arguments of a request, if null (or empty) then an empty
	 *                  request is sent
	 * @param handler   The callback method for this service request
	 */
	public void CallService(String service, JsonStructure args, MessageHandler handler) {

		UUID id = this.createUniqueID();
		JsonObject call = Json.createObjectBuilder().add("id", id.toString()).add("op", "call_service")
				.add("service", service).add("args", args).build();

		handlers.put(id, handler);

		System.out.println("CALL: " + call.toString());

		this.client.send(call.toString());
	}

	/**
	 * Calls a service on the ROS system with possibly arguments for the request
	 * message. The message will be called with the message upon receipt of the
	 * message service response
	 *
	 * @param service   The name of the service to call
	 * @param args      The arguments of a request, if null (or empty) then an empty
	 *                  request is sent
	 * @param handler   The callback method for this service request
	 */
	@Override
	public void callService(String service, JsonElement args, Consumer<JsonElement> handler) {
		MessageHandler messageHandler = new MessageHandler() {
			@Override
			public void messageReceived(String msg) {
				try {
					JsonElement json = new JsonParser().parse(msg);
					handler.accept(json);
				}catch (Exception e){
					logger.info("Failed to parse json: " + msg);
				}
			}
		};
		JsonStructure jsonArgs = null;
		if (args != null){
			String json = args.toString();
			try(JsonReader reader = Json.createReader(new StringReader(json))) {
				jsonArgs = reader.read();
			}catch (Exception e){
				logger.warn("Failed to convert args to other json class: " + args);
			}
		}
		CallService(service, jsonArgs, messageHandler);
	}

	/**
	 * A response to a service request from the ROS system
	 *
	 * @param id
	 *            The ID of the service that was requested
	 * @param values
	 *            A JSONArray of values of the response
	 */
	public void ServiceResponse(UUID id, JsonArray values) {
		JsonObject call = Json.createObjectBuilder().add("op", "service_response").add("id", id.toString())
				.add("service", this.publishedTopics.get(id)).add("values", values).build();
		this.client.send(call.toString());
	}

	/**
	 * Sends a status message to be displayed by the rosbridge server
	 *
	 * @param message
	 *            A message string to be displayed
	 * @param level
	 *            The message status level ('info', 'debug', 'warning', 'error')
	 */
	public void StatusMessage(String message, String level) {
		JsonObject call = Json.createObjectBuilder().add("op", "status").add("level", level).add("msg", message)
				.build();
		this.client.send(call.toString());
	}

	public void close() {
		if (client != null && client.getConnection().isOpen()) {
			client.close();
		}
	}
}