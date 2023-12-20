//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



//  WARNING: This document contains technical data whose export or disclosure to non-U.S. persons, wherever located, is restricted by the International Traffic in Arms Regulations (ITAR) (22 C.F.R. Sections 120-130). Violations are subject to severe criminal penalties.

//  This document is proprietary and confidential. No part of this document may be disclosed in any manner to a third party without the prior written consent of Raytheon BBN Technologies.

//  Copyright 2020 Raytheon BBN Technologies.


package com.bbn.mace.utils;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Utilities to do UDP networking off of the main thread.
 */
public class UdpNetworkingUtils {

	private static final Map<String, Queue<DatagramPacket>> udpNetworkQueue = new HashMap<>();
	private static Map<String, Thread> udpNetworkThread = new HashMap<>();

	private static final Logger logger = LogManager.getLogger(UdpNetworkingUtils.class.getName());


	private static String getStringKey(DatagramPacket packet) {
		
		return packet.getAddress().toString() + ":" + packet.getPort();
	}

	/**
	 * Queue a packet for sending on the UDP network thread. This will create the
	 * network thread if it does not already exist.
	 * 
	 * @param packet The UDP packet to send.
	 */
	public static void sendPacketAsync(DatagramPacket packet) {

		String key = getStringKey(packet);
		Queue<DatagramPacket> queue = udpNetworkQueue.get(getStringKey(packet));
		
		if (queue == null) {
			queue = startNetworkThread(key);
		}

		queue.add(packet);
	}

	/**
	 * Send a String over UDP to the provided destination.
	 * 
	 * @param message the String to send
	 * @param host    the destination hostname
	 * @param port    the destination pet
	 * @throws UnknownHostException if the host is invalid.
	 */
	public static void sendStringAsync(String message, String host, int port) throws UnknownHostException {
		DatagramPacket packet = new DatagramPacket(message.getBytes(), message.getBytes().length,
				InetAddress.getByName(host), port);
		sendPacketAsync(packet);

	}

	/**
	 * Start a thread for sending queued UDP packets.
	 */
	private static Queue<DatagramPacket> startNetworkThread(String key) {

		// Make sure we actually need to do anything
		Queue<DatagramPacket> existingQueue = udpNetworkQueue.get(key);
		if (existingQueue != null) {
			logger.warn("Tried to start UDP thread for " + key + " but we already have one going.");
			return existingQueue;
		}
		
		logger.info("Making new UDP thread for " + key);

		final LinkedBlockingQueue<DatagramPacket> queue = new LinkedBlockingQueue<>();
		udpNetworkQueue.put(key, queue);

		Thread newThread = new Thread(new Runnable() {
			
			@Override
			public void run() {
				
				// This outer loop lets the DatagramSocket get re-created if there's an IOError.
				boolean udpNetworkThreadRunning = true;
				while (udpNetworkThreadRunning) {

					try (DatagramSocket socket = new DatagramSocket()) {
						socket.setReuseAddress(true);
						socket.setSendBufferSize(1000000);
						// This inner loop will send packets on the same DatagramSocket unless there's
						// an IOError.
						while (udpNetworkThreadRunning) {
							
							// This will wait until a packet is available.
							DatagramPacket packet = queue.take();
							DatagramPacket toSend = new DatagramPacket(packet.getData(), packet.getLength(),
									packet.getAddress(), packet.getPort());

							long start = System.currentTimeMillis();
							socket.send(toSend);
							long delta = System.currentTimeMillis() - start;
							boolean shouldLog = delta > 1000;
							if (shouldLog) {
								logger.info("Waited " + delta / 1000.0 + " seconds to socket.send for " + key);
								logger.info(udpNetworkQueue.size());
								logger.error("~~~~~~Sending packet async truly" + Thread.currentThread().getId());

							}
						}
					} catch (IOException e) {
						// The next iteration will try to re-create the DatagramSocket
						// Since this is UDP, I don't try to re-queue failed packet sends.
						// logger.info("Failed to send UDP msg to " + address + ":" + port + ". -
						// Exception:", e);
					} catch (InterruptedException e) {
						// If the thread is interrupted, exit.
						udpNetworkThreadRunning = false;
						logger.info("InerruptedException");
						Thread.currentThread().interrupt();
					}

				}
			}
		}, "UDP send thread to " + key);
		
		udpNetworkThread.put(key, newThread);
		
		newThread.start();
		
		return queue;
	}

	/**
	 * Shutdown the thread that sends queued UDP packets.
	 * TODO actually maybe call this some day.
	 */
//	public static void stopNetworkThread() {
//		synchronized (udpNetworkThreadLock) {
//			udpNetworkThreadRunning = false;
//			if (udpNetworkThread != null) {
//				udpNetworkThread.interrupt();
//				udpNetworkThread = null;
//			}
//		}
//	}

	public static void main(String[] args) throws InterruptedException {
		System.out.println("Starting...");
		int testPort = 5172;
		AtomicBoolean running = new AtomicBoolean(true);
		AtomicInteger count = new AtomicInteger(0);
		// Test queueing a send.
		Thread recvThread = new Thread(new Runnable() {
			
			@Override
			public void run() {
				
				System.out.println("Starting receive thread.");
				try (DatagramSocket recvSock = new DatagramSocket(testPort)) {
					
					while (running.get()) {
						
						byte[] buffer = new byte[1024];
						DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

						recvSock.receive(packet);
						byte[] msg = new byte[packet.getLength()];
						System.arraycopy(buffer, 0, msg, 0, packet.getLength());
						System.out.println("Received: " + new String(msg));
						count.getAndIncrement();
					}
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		});
		recvThread.start();
		Thread.sleep(1000);
		System.out.println("Queueing packets...");
		int total = 20;
		try {
			for (int i = 0; i < total; i++) {
				sendStringAsync(String.format("This is test UDP message #%d", i), "127.0.0.1", testPort);
			}
		} catch (UnknownHostException e) {
			e.printStackTrace();
		}
		while (count.get() < total) {
			Thread.sleep(1000);
		}
		running.set(false);
		recvThread.interrupt();
	}

	public static final void sendUdpCot(String toSend, InetAddress cotAddress, int cotPort) {
		DatagramPacket packet = new DatagramPacket(toSend.getBytes(), toSend.getBytes().length, cotAddress, cotPort);
		UdpNetworkingUtils.sendPacketAsync(packet);
	}
}
