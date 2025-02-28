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

    static final String sToggleKey      = "toggle";
    static public final String sTypeKey = "toggle-actuator";

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
            if (reader.has(sToggleKey)) {
                JSONArray array = reader.getJSONArray(sToggleKey);
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

    /**
     * Writes the current toggle actuator configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) {
        try {
            writer.put(sTypeKey, "toggle-actuator");
            
            JSONArray toggle = new JSONArray();
            toggle.put(mPosition1Name);
            toggle.put(mPosition2Name);
            writer.put(sToggleKey,toggle);

        }
        catch(JSONException e) {
            mLogger.error(e.getMessage());
        }
        this.writeWithoutType(writer);
    }

    /**
     * Generates an HTML representation of the actuator configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted actuator configuration.
     */
    @Override
    public String                       logConfigurationHTML() {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {
            result.append(super.logConfigurationHTML());

            result.append("<details>\n");
            result.append("<summary style=\"padding-left:10px;font-size: 10px; font-weight: 500\"> TOGGLE </summary>\n");
            result.append("<ul>\n");

            result.append("<li style=\"padding-left:10px; font-size: 10px\">")
                    .append(mPosition1Name)
                    .append("</li>");

            result.append("<li style=\"padding-left:10px; font-size: 10px\">")
                    .append(mPosition2Name)
                    .append("</li>");

            result.append("</ul>\n");
            result.append("</details>\n");

        }

        return result.toString();

    }

    /**
     * Generates a text-based representation of the actuator configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted actuator configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {

            result.append(super.logConfigurationText(header));

            result.append(header)
                    .append("> TOGGLE\n");
            result.append(header)
                    .append("--> ")
                    .append(mPosition1Name)
                    .append("\n");
            result.append(header)
                    .append("--> ")
                    .append(mPosition2Name)
                    .append("\n");
        }

        return result.toString();

    }


}