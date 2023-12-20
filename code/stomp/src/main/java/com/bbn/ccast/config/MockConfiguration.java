//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.config;

public class MockConfiguration extends Configuration {


    @Override
    public String getCotAddress() {return "127.0.0.1"; }

    @Override
    public Integer getCotPort() {return 8080;}

    public String getDispatcherAddress() {
        return "192.168.1.202";
    }

    public Integer getDispatcherPort() {
        return 8080;
    }


}
