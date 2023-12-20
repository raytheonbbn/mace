//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim.blenodes;

public class SimMassTargetConfig extends SimTargetConfig{
    public double captureRange;
    protected int payloadsRequired;

    public SimMassTargetConfig(SimTargetConfig oldConfig, double captureRange, int payloadsRequired, boolean suppression) {
        super(oldConfig, SimTargetType.MASS.name());
        this.captureRange = captureRange;
        this.payloadsRequired = payloadsRequired;
        this.suppression = suppression;
    }
}
