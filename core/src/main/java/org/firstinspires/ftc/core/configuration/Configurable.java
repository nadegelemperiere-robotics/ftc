/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Configuration management
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.configuration;

/* Json includes */
import org.json.JSONObject;


public interface Configurable {

    /**
     * Reads configuration from a JSON object
     *
     * @param reader : JSON object containing configuration
     */
    public void read(JSONObject reader);

    /**
     * Writes configuration to a JSON object
     *
     * @param writer : JSON object to write configuration includes
     */
    public void write(JSONObject writer);

    /**
     * Configuration checking
     *
     * @return true if object is correctly configured, false otherwise
     */
    public boolean isConfigured();

    /**
     * Configuration logging into HTML
     *
     * @return configuration as html string
     */
    public String logConfigurationHTML();

    /**
     * Configuration logging into string
     *
     * @return configuration as string
     */
    public String logConfigurationText(String header);

}