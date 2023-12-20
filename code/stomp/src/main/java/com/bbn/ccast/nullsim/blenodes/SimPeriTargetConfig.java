//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim.blenodes;

public class SimPeriTargetConfig extends SimTargetConfig{
    public double captureRange;
    protected int payloadsRequired;
    protected double captureCountdown;

    public SimPeriTargetConfig(SimTargetConfig oldConfig, double captureRange, int payloadsRequired, double captureCountdown, boolean suppression) {
        super(oldConfig, SimTargetType.PERI.name());
        this.captureRange = captureRange;
        this.payloadsRequired = payloadsRequired;
        this.captureCountdown = captureCountdown;
        this.suppression = suppression;
    }
}
