/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Configuration management
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;


public class SeasonConfiguration extends Configuration {

    static {
        sDefaultConfiguration = "v1";
    }

    /**
     * Configuration constructor
     *
     * @param logger logger
     */
    public SeasonConfiguration(LogManager logger) {
        super(logger);
        
    }

}
