//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.sim;

/**
 * Time server that allows for time scales other than 1.0. (-For
 * faster-than-real-time simulation, for example.)
 * 
 * @author kmoffitt
 */
public class TimeServer {

	private final boolean isRealTime;

	private final long baseTime = System.currentTimeMillis();

	private double timeScale;

	private long realTimeAtLastQuery = baseTime;
	private long scaledTimeAtLastQuery = baseTime;

	/**
	 * Make one - real-time by default.
	 */
	public TimeServer() {

		this.isRealTime = true;
		this.timeScale = 1.0;
	}

	/**
	 * Make one that is not real-time.
	 * 
	 * @param timeScale
	 *            The rate at which time should progress, with 1.0 matching real
	 *            time.
	 */
	public TimeServer(double timeScale) {

		this.isRealTime = false;
		this.timeScale = timeScale;
	}

	/**
	 * If {@link #isRealTime}, simply default to {@link System#currentTimeMillis()}.
	 * <p>
	 * Otherwise, calculate the milliseconds from the epoch as if we started at the
	 * real time when the object was made and moved time forward from there at the
	 * rate specified in {@link #timeScale}.
	 * 
	 * @return milliseconds from the epoch, if {@code #isRealTime}, else
	 *         milliseconds from the epoch scaled by timeScale starting when the
	 *         object was made.
	 */
	public long currentTimeMillis() {

		if (isRealTime) {
			return System.currentTimeMillis();
		}

		// Adjust based on delta from last query so we can adjust scale dynamically
		long currentRealTime = System.currentTimeMillis();
		long realTimeDelta = currentRealTime - realTimeAtLastQuery;
		realTimeAtLastQuery = currentRealTime;

		long scaledTimeDelta = (long) (realTimeDelta * timeScale);
		scaledTimeAtLastQuery += scaledTimeDelta;

		return scaledTimeAtLastQuery;
	}

	public double getTimeScale() {

		return timeScale;
	}

	public void setTimeScale(double timeScale) {
		this.timeScale = timeScale;
	}
}
