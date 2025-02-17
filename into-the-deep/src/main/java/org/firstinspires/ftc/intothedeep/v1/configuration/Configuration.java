/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Configuration management
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.configuration;

/* Tools includes */



public class Configuration extends org.firstinspires.ftc.core.configuration.Configuration {

    static protected Configuration  sInstance = null;

    static {
        sDefaultConfiguration = "v1";
    }

    public  static  Configuration getInstance() {
        if(sInstance == null) { sInstance = new Configuration(); }
        return sInstance;
    }

    /**
     * Configuration constructor
     *
     */
    public Configuration() {
        super();
    }


}
