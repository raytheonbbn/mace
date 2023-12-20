//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.utils;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
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
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;

public class MaceMessageTransport {
    private static final Logger logger = LogManager.getLogger(MaceMessageTransport.class.getName());

    private static final int MAX_MESSAGES_IN_FLIGHT = 100;

    private MqttClient client;
    private SSLSocketFactory sslSocketFactory;
    private final String brokerAddr;
    private final AtomicBoolean connecting = new AtomicBoolean(false);
    private final Set<SubInfo> subscriptions = ConcurrentHashMap.newKeySet();
    private final Map<String, List<IMqttMessageListener>> topicHandlers = new ConcurrentHashMap<>();

    private final String user;
    private final String password;


    public boolean isConnected() {
        return client.isConnected();
    }

    private static class SubInfo{
        int qos;
        String topic;
        IMqttMessageListener messageHandler;

        SubInfo(String topic, int qos, IMqttMessageListener messageHandler){
            this.qos = qos;
            this.topic = topic;
            this.messageHandler = messageHandler;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            SubInfo subInfo = (SubInfo) o;
            return qos == subInfo.qos && topic.equals(subInfo.topic) && messageHandler.equals(subInfo.messageHandler);
        }

        @Override
        public int hashCode() {
            return Objects.hash(qos, topic, messageHandler);
        }
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

    public MaceMessageTransport(String brokerAddr, String clientId, String user, String password){
        MemoryPersistence persistence = new MemoryPersistence();
        this.brokerAddr = brokerAddr;
        this.user = user;
        this.password = password;
        try {
            KeyStore trustStore = KeyStore.getInstance(KeyStore.getDefaultType());
            trustStore.load(null,null);
            trustStore.setCertificateEntry("Custom CA", (X509Certificate) CertificateFactory.getInstance("X509").generateCertificate(new FileInputStream("config/ca_certificates/ca.crt")));

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
        } catch (KeyStoreException | CertificateException | NoSuchAlgorithmException | IOException |
                 KeyManagementException e) {
            logger.error("Error setting up TLS for MQTT client:  ", e);
        }
    }

    private void resubscribe() {
        for (SubInfo info : subscriptions){
            subscribe(info.topic, info.qos, info.messageHandler, true);
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
                connOpts.setUserName(user);
                connOpts.setPassword(password.toCharArray());
                connOpts.setSocketFactory(sslSocketFactory);
                logger.trace("Connecting to broker " + brokerAddr);
                while (!connected) {
                    try {
                        client.connect(connOpts);
                        logger.trace("Connected.");
                        connected = true;
                    } catch (MqttException e) {
                        logger.error("Failed to connect to broker " + brokerAddr + ": ", e);
                    }
                }
                connecting.getAndSet(false);
            }, "MaceMessageTransport Connect Thread").start();
        }
    }

    public void publish(String topic, MqttMessage message) throws MqttException {
        client.publish(topic, message);
    }

    public void subscribe(String topic, int serviceLevel, IMqttMessageListener messageHandler, boolean isResubscribe){
        if (!isResubscribe) {
            SubInfo subInfo = new SubInfo(topic, serviceLevel, messageHandler);
            subscriptions.add(subInfo);
        }
        List<IMqttMessageListener> handlers = topicHandlers.computeIfAbsent(topic, s -> new ArrayList<>());
        if (handlers.size() == 0 || isResubscribe){
            try {
                IMqttMessageListener handlerHelper = new IMqttMessageListener() {
                    @Override
                    public void messageArrived(String topic, MqttMessage message) throws Exception {
                        for (IMqttMessageListener listener : topicHandlers.get(topic)){
                            listener.messageArrived(topic, message);
                        }
                    }
                };
                client.subscribe(topic, serviceLevel, handlerHelper);
            } catch (MqttException e) {
                logger.warn("Failed to susbcribe to MQTT topic " + topic + ": ", e);
            }
        }
        if (!isResubscribe){
            handlers.add(messageHandler);
        }

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
