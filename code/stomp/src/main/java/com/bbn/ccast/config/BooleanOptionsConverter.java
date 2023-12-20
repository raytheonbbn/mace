//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.config;

import com.google.devtools.common.options.Converter;
import com.google.devtools.common.options.OptionsParsingException;

public class BooleanOptionsConverter implements Converter<Boolean> {

    @Override
    public Boolean convert(String input) throws OptionsParsingException {
        if(input == null) return null;
        if(CmdOptions.DEFAULT_BOOLEAN.equals(input)) return null;
        try {
            return Boolean.parseBoolean(input);
        } catch (Exception e) {
            throw new OptionsParsingException("Could not parse Integer", e);
        }
    }

    @Override
    public String getTypeDescription() {
        return "Boolean";
    }
}
