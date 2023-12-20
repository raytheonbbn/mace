//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.config;

import com.google.devtools.common.options.Converter;
import com.google.devtools.common.options.OptionsParsingException;

public class DoubleOptionsConverter implements Converter<Double> {

    @Override
    public Double convert(String input) throws OptionsParsingException {
        if(input == null) return null;
        if(CmdOptions.DEFAULT_DOUBLE.equals(input)) return null;
        try {
            return Double.parseDouble(input);
        } catch (Exception e) {
            throw new OptionsParsingException("Could not parse Double", e);
        }
    }

    @Override
    public String getTypeDescription() {
        return "Double";
    }
}
