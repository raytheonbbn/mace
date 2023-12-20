//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim;

import com.google.devtools.common.options.Option;
import com.google.devtools.common.options.OptionsBase;

public class NullSimCmdOptions extends OptionsBase {

    @Option(
            name = "numSolos",
            abbrev = 's',
            help = "Solo quadrotor count.",
            defaultValue = "0")
    public int numSolos;

    @Option(
            name = "numRovers",
            abbrev = 'r',
            help = "Rover count.",
            defaultValue = "0")
    public int numRovers;

    @Option(
            name = "numDownFacingSolos",
            abbrev = 'g',
            help = "Down-facing Solo quadrotor count.",
            defaultValue = "0")
    public int numDownFacingSolos;

    @Option(
            name = "numSkydios",
            abbrev = 'y',
            help = "Skydio quadrotor count.",
            defaultValue = "0")
    public int numSkydios;

    @Option(
            name = "numIfos",
            abbrev = 'p',
            help = "Ifo quadrotor count.",
            defaultValue = "0")
    public int numIfos;


}
