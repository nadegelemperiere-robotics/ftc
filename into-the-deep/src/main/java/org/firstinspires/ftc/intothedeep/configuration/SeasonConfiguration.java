/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Configuration management
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.configuration;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

import java.util.LinkedHashMap;


public class SeasonConfiguration extends Configuration {

    static protected SeasonConfiguration  sInstance = null;

    static {
        sDefaultConfiguration = "v1";
    }

    public  static  SeasonConfiguration getInstance() {
        if(sInstance == null) { sInstance = new SeasonConfiguration(); }
        return sInstance;
    }

    /**
     * Configuration constructor
     *
     */
    public SeasonConfiguration() {
        super();
    }


}
