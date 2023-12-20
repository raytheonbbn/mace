//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.mqtt;

import org.apache.log4j.Logger;
import org.eclipse.paho.client.mqttv3.*;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;

import javax.net.ssl.SSLContext;
import javax.net.ssl.SSLSocketFactory;
import javax.net.ssl.TrustManager;
import javax.net.ssl.TrustManagerFactory;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.security.KeyManagementException;
import java.security.KeyStore;
import java.security.KeyStoreException;
import java.security.NoSuchAlgorithmException;
import java.security.cert.CertificateException;
import java.security.cert.CertificateFactory;
import java.security.cert.X509Certificate;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

public class MaceMessageTransport {
    private static final Logger logger = Logger.getLogger(MaceMessageTransport.class.getName());
    private static final int MAX_MESSAGES_IN_FLIGHT = 100;

    private MqttClient client;
    private SSLSocketFactory sslSocketFactory;
    private final String brokerAddr;
    private final AtomicBoolean connecting = new AtomicBoolean(false);
    private final Object connectionSignal = new Object();
    private final ConcurrentHashMap<String, Consumer<String>> subscriptions = new ConcurrentHashMap<>();

    private final String user;
    private final String password;


    public boolean isConnected() {
        return client.isConnected();
    }

    public enum QualityOfService {
        UNRELIABLE(0),
        RECEIVE_AT_LEAST_ONCE(1),
        RECEIVE_EXACTLY_ONCE(2);

        private final int qos;
        QualityOfService(int qos){
            this.qos = qos;
        }
    }

    public MaceMessageTransport(String brokerAddr, String clientId){
        this(brokerAddr, clientId, null, null);
    }

    public MaceMessageTransport(String brokerAddr, String clientId, String user, String password){
        MemoryPersistence persistence = new MemoryPersistence();
        this.brokerAddr = brokerAddr;
        this.user = user;
        this.password = password;
        try {
            KeyStore trustStore = KeyStore.getInstance(KeyStore.getDefaultType());
            trustStore.load(null,null);
            FileInputStream crtFile = new FileInputStream("config/ca_certificates/ca.crt");
            trustStore.setCertificateEntry("Custom CA", (X509Certificate) CertificateFactory.getInstance("X509").generateCertificate(crtFile));

            TrustManagerFactory tmf = TrustManagerFactory.getInstance(TrustManagerFactory.getDefaultAlgorithm());
            tmf.init(trustStore);
            TrustManager[] trustManagers = tmf.getTrustManagers();

            SSLContext sslContext = SSLContext.getInstance("TLSv1.2");
            sslContext.init(null, trustManagers, null);
            this.sslSocketFactory =  sslContext.getSocketFactory();
            this.client = new MqttClient(brokerAddr, clientId, persistence);
            MqttCallbackExtended callback = new MqttCallbackExtended() {
                @Override
                public void connectComplete(boolean reconnect, String serverURI) {
                    resubscribe();
                }

                @Override
                public void connectionLost(Throwable cause) {

                }

                @Override
                public void messageArrived(String topic, MqttMessage message) throws Exception {

                }

                @Override
                public void deliveryComplete(IMqttDeliveryToken token) {

                }
            };
            client.setCallback(callback);
            connectAsync();
        } catch (MqttException e) {
            logger.error("Failed to create MqttClient: ", e);
        } catch (FileNotFoundException e) {
            logger.error("Could not find MQTT certfile: ", e);
        } catch (KeyStoreException | CertificateException | NoSuchAlgorithmException | IOException | KeyManagementException e) {
            logger.error("Error setting up TLS for MQTT client:  ", e);
        }
    }

    private void resubscribe() {
        for (Map.Entry<String, Consumer<String>> entry : subscriptions.entrySet()){
            subscribe(entry.getKey(), entry.getValue(), false);
        }
    }

    private void connectAsync(){
        boolean alreadyConnecting = connecting.getAndSet(true);
        if (!alreadyConnecting) {
            new Thread(() -> {
                boolean connected = false;
                MqttConnectOptions connOpts = new MqttConnectOptions();
                connOpts.setMaxInflight(MAX_MESSAGES_IN_FLIGHT);
                connOpts.setCleanSession(true);
                connOpts.setAutomaticReconnect(true);
                connOpts.setSocketFactory(sslSocketFactory);
                if (user != null && password != null){
                    connOpts.setUserName(user);
                    connOpts.setPassword(password.toCharArray());
                }
                logger.trace("Connecting to broker " + brokerAddr);
                while (!connected) {
                    try {
                        client.connect(connOpts);
                        logger.trace("Connected.");
                        synchronized (connectionSignal) {
                            connectionSignal.notifyAll();
                        }
                        connected = true;
                    } catch (MqttException e) {
                        logger.error("Failed to connect to broker " + brokerAddr + ": ", e);
                    }
                }
                connecting.getAndSet(false);
            }, "MaceMessageTransport Connect Thread").start();
        }
    }

    public void publish(String topic, String content, QualityOfService qosLevel) throws MqttException {
        MqttMessage message = new MqttMessage(content.getBytes());
        message.setQos(qosLevel.qos);
        client.publish(topic, message);
    }

    public void subscribe(String topic, Consumer<String> messageHandler, boolean addToMap){
        if (addToMap){
            subscriptions.put(topic, messageHandler);
        }
        int maxQos = QualityOfService.RECEIVE_AT_LEAST_ONCE.qos;
        try {
            client.subscribe(topic, maxQos, new IMqttMessageListener() {
                @Override
                public void messageArrived(String topic, MqttMessage message) throws Exception {
                    messageHandler.accept(new String(message.getPayload()));
                }
            });
        } catch (MqttException e) {
            logger.warn("Failed to susbcribe to MQTT topic " + topic + ": ", e);
        }
    }

    public void subscribe(String topic, Consumer<String> messageHandler){
        subscribe(topic, messageHandler, true);
    }

    public void subscribeForBytes(String topic, Consumer<byte[]> messageHandler) throws MqttException {
        int maxQos = QualityOfService.RECEIVE_AT_LEAST_ONCE.qos;
        client.subscribe(topic, maxQos, new IMqttMessageListener() {
            @Override
            public void messageArrived(String topic, MqttMessage message) throws Exception {
                messageHandler.accept(message.getPayload());
            }
        });
    }

    public void disconnectAndClose(){
        try {
            logger.info("Disconnecting...");
            client.disconnect();
        } catch (MqttException e) {
            logger.warn("Failed to disconnect: ", e);
            logger.info("Attempting to disconnect forcibly...");
            try {
                client.disconnectForcibly();
            } catch (MqttException mqttException) {
                logger.error("Failed to disconnect forcibly: ", e);
            }
        }
        try {
            client.close();
        } catch (MqttException e) {
            logger.error("Failed to close client: ", e);
        }
    }

}
