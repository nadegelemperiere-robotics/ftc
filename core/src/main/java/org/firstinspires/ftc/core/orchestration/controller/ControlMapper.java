/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Class mapping controller inputs to robot behavior
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.orchestration.controller;

/* System includes */
import java.util.Map;

/* Json includes */
import org.json.JSONObject;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

/* Components includes */
import org.firstinspires.ftc.core.components.controllers.Controller;

public class ControlMapper implements Configurable {

    LogManager              mLogger;

    boolean                 mConfigurationValid;


    Map<String,Controller>  mControllers;

    /**
     * ControlMapper constructor
     *
     * @param logger logger
     */
    public ControlMapper(Map<String,Controller> controllers, LogManager logger) {
        mLogger      = logger;
        mControllers = controllers;
        mConfigurationValid = true;
    }

    public void register(String topic) {}
    
    /**
     * Reads control configuration
     *
     * @param reader : JSON object containing configuration
     */
    public void read(JSONObject reader) {}

    /**
     * Writes control configuration
     *
     * @param writer : JSON object to store configuration
     */
    public void write(JSONObject writer) {}

    /**
     * Configuration checking
     *
     * @return true if object is correctly configured, false otherwise
     */
    public boolean isConfigured() {
        return mConfigurationValid;
    }

    /**
     * Configuration logging into HTML
     *
     * @return configuration as html string
     */
    public String  logConfigurationHTML() { return ""; }

    /**
     * Configuration logging into text
     *
     * @return configuration as string
     */
    public String  logConfigurationText() { return ""; }



    public void loop() {

    }


}
