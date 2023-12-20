//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.mqtt;

import org.eclipse.paho.client.mqttv3.MqttException;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;

import static org.junit.jupiter.api.Assertions.*;

class MaceMessageTransportTest {
    private static MaceMessageTransport client;
    @BeforeAll
    static void setUp() throws InterruptedException {
        client = new MaceMessageTransport("tcp://127.0.0.1:1883", "TEST-CLIENT");
        System.out.print("Waiting for MQTT client to connect.");
        while (!client.isConnected()){
            System.out.print(".");
            Thread.sleep(1000);
        }
        System.out.println("Connected.");
    }

    @AfterAll
    static void tearDown() {
        client.disconnectAndClose();
    }

    @Test
    void subscribeAndPublish() throws MqttException, InterruptedException {

        System.out.println("Starting subscribeAndPublish test.");
        int numTopics = 5;
        int numMsgsToSendPerTopic = 10;
        String[] testTopics = new String[numTopics];
        String[][] messageContents = new String[numTopics][numMsgsToSendPerTopic];
        AtomicBoolean[][] recvdMsgs = new AtomicBoolean[numTopics][numMsgsToSendPerTopic];
        AtomicInteger[] topicMsgRecvCounts = new AtomicInteger[numTopics];
        for (int topicNum=0; topicNum < numTopics; topicNum++){
            topicMsgRecvCounts[topicNum] = new AtomicInteger();
            testTopics[topicNum] = String.format("Test_Topic_%d", topicNum+1);
            for (int msgNum=0; msgNum < numMsgsToSendPerTopic; msgNum++) {
                    messageContents[topicNum][msgNum] = String.format("This is test message payload (%d of %d) on topic %s.",
                            msgNum+1, numMsgsToSendPerTopic, testTopics[topicNum]);
                    recvdMsgs[topicNum][msgNum] = new AtomicBoolean(false);
            }
            final int topicNumFinal = topicNum;
            client.subscribe(testTopics[topicNumFinal], new Consumer<>() {
                @Override
                public void accept(String msg) {
                    System.out.printf("Received message on topic %s: %s\n", testTopics[topicNumFinal], msg);
                    topicMsgRecvCounts[topicNumFinal].incrementAndGet();
                    int startIdx = msg.indexOf("(")+ 1;
                    int endIdx = msg.indexOf(" of", startIdx);
                    int msgNum = Integer.parseInt(msg.substring(startIdx, endIdx)) - 1;
                    assertEquals(msg, messageContents[topicNumFinal][msgNum], "Message contents did not match any expected message!");
                    boolean alreadyRecvd = recvdMsgs[topicNumFinal][msgNum].getAndSet(true);
                    assertFalse(alreadyRecvd, "Received duplicate message!");
                }
            });
        }

        for (int msgNum=0; msgNum < numMsgsToSendPerTopic; msgNum++) {
            for (int topicNum=0; topicNum < numTopics; topicNum++) {
                client.publish(testTopics[topicNum], messageContents[topicNum][msgNum], MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
            }
        }

        System.out.println("Waiting a bit for messages.");
        Thread.sleep(3000);
        System.out.println("Done waiting.");

        for (int topicNum=0; topicNum < numTopics; topicNum++){
            int msgsRecvd = topicMsgRecvCounts[topicNum].get();
            assertEquals(msgsRecvd, numMsgsToSendPerTopic,
                    String.format("Expected %d messages on topic %s, but received %d!",
                            numMsgsToSendPerTopic, testTopics[topicNum], msgsRecvd));
            for (int msgNum=0; msgNum < numMsgsToSendPerTopic; msgNum++){
                assertTrue(recvdMsgs[topicNum][msgNum].get(), "Failed to receive an expected message!");
            }
        }

        System.out.println("Received the expected count of messages, and exactly one of each desired message.");
    }
}