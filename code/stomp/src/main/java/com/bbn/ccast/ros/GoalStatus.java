//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.ros;

/** DO NOT ALTER!  From ROS documentation at http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html */
public enum GoalStatus {
	
	/** The goal has yet to be processed by the action server */
	PENDING,
			
	/** The goal is currently being processed by the action server */
	ACTIVE,

	/** The goal received a cancel request after it started executing and has since completed its execution (Terminal State) */
	PREEMPTED,
	
	/** The goal was achieved successfully by the action server (Terminal State) */
	SUCCEEDED,
	
	/** The goal was aborted during execution by the action server due to some failure (Terminal State) */
	ABORTED,
	
	/** The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State) */
	REJECTED,
	
	/** The goal received a cancel request after it started executing and has not yet completed execution */
	PREEMPTING,
	
	/** The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled */
	RECALLING,
	
	/** The goal received a cancel request before it started executing and was successfully cancelled (Terminal State) */
	RECALLED,
	
	/** An action client can determine that a goal is LOST. This should not be sent over the wire by an action server */
	LOST;

	public static GoalStatus fromOrdinal(int ordinal) {
		return values()[ordinal];
	}
}