/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Actuator with 2 reference positions to switch between
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.subsystems;

/* JSON object */
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONArray;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class ToggleActuator extends Actuator {

    static final String sToogleKey = "toggle";

    String  mPosition1Name;
    String  mPosition2Name;

    /**
     * Constructor
     *
     * @param name The subsystem name
     * @param hardware The robot current hardware
     * @param logger The logger to report events
     */
    public ToggleActuator(String name, Hardware hardware, LogManager logger) {
        super(name, hardware, logger);
    }

    /**
     * Reads and applies the actiuator configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    @Override
    public void read(JSONObject reader) {
        super.read(reader);

        mPosition1Name = "";
        mPosition2Name = "";

        try {
            if (reader.has(sToogleKey)) {
                JSONArray array = reader.getJSONArray(sToogleKey);
                if(array.length() == 2) {
                    String name = array.getString(0);
                    if(mPositions.containsKey(name)) {
                        mPosition1Name = name;
                    }
                    name = array.getString(1);
                    if(mPositions.containsKey(name)) {
                        mPosition2Name = name;
                    }
                }
            }
        } catch(JSONException e) {
            mLogger.error(e.getMessage());
        }

        if(mPosition1Name.isEmpty()) {
            mLogger.error("Toggle first position has not been found or is invalid");
            mConfigurationValid = false;
        }
        if(mPosition2Name.isEmpty()) {
            mLogger.error("Toggle second position has not been found or is invalid");
            mConfigurationValid = false;
        }

    }

    /**
     * Function to toggle from one position to another
     *
     * @param timeout (ms) The time allocated to change position
     */
    public  void  toggle(int timeout) {

        if(mPosition.equals(mPosition1Name)) { this.position(mPosition2Name,0,timeout); }
        else if(mPosition.equals(mPosition2Name)) { this.position(mPosition1Name, 0, timeout); }

    }
}