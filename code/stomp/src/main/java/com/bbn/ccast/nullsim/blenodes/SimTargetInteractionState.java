//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim.blenodes;

public class SimTargetInteractionState {
    protected final String payloadId;
    protected long payloadEnteredRangeTime;
    protected long payloadStartedInteracitonTime;

    public SimTargetInteractionState(String payloadId) {
        this.payloadId = payloadId;
    }
}
