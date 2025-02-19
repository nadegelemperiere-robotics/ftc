/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Robot manager
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.robot;

/* System includes */
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;

/* Json includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

/* Subsystems includes */
import org.firstinspires.ftc.core.subsystems.Subsystem;

/* Sequencer includes */
import org.firstinspires.ftc.core.orchestration.engine.Context;


public class Robot extends Context implements Configurable {

    protected static final String      sHardwareKey   = "hardware";
    protected static final String      sSubsystemsKey = "subsystems";

    protected final LogManager              mLogger;

    protected final Map<String, Subsystem>  mSubsystems;

    protected boolean                       mConfigurationValid;

    protected final Hardware                mHardware;

    /**
     * Robot constructor
     *
     * @param map Hardware map to look for components
     * @param logger Logger
     */
    public  Robot(HardwareMap map, LogManager logger) {
        mLogger             = logger;

        mConfigurationValid = false;

        mHardware           = new Hardware(map, logger);
        mSubsystems         = new LinkedHashMap<>();
    }

    /**
     * Robot start function
     */
    public void                         start() {}

    /**
     * Robot update function updating all subsystems + current state
     */
    public void                         update()
    {
        for (Map.Entry<String, Subsystem> subsystem : mSubsystems.entrySet()) {
            subsystem.getValue().update();
        }
        super.update();
    }

    /**
     * Robot logging function logging all subsystems + current state
     */
    public void                         log()
    {
        for (Map.Entry<String, Subsystem> subsystem : mSubsystems.entrySet()) {
            subsystem.getValue().log();
        }
    }

    /**
     * Robot persist function, storing all subsystems interopmodes data to the InterOpMode singleton
     */
    public void                         persist()
    {
        for (Map.Entry<String, Subsystem> subsystem : mSubsystems.entrySet()) {
            subsystem.getValue().persist();
        }
    }

    /**
     * Determines if the robot is configured correctly.
     *
     * @return True if the robot is configured, false otherwise.
     */
    public boolean                      isConfigured() { return mConfigurationValid;}

    /**
     * Reads all subsystems declared in the JSON object. Relies on
     * Default subsystem factory, so should be overloaded if using specific
     * subsystems
     *
     * @param reader The JSON object containing configuration settings.
     */
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;
        mSubsystems.clear();

        try {
            if(reader.has(sHardwareKey)) {
                JSONObject hardware = reader.getJSONObject(sHardwareKey);
                mHardware.read(hardware);
                if(!mHardware.isConfigured()) {
                    mLogger.warning("Hardware configuration is invalid");
                    mConfigurationValid = false;
                }
            }

            if(reader.has(sSubsystemsKey)) {
                JSONObject subsystems = reader.getJSONObject(sSubsystemsKey);

                Iterator<String> keys = subsystems.keys();
                while (keys.hasNext()) {

                    String key = keys.next();

                    Subsystem subsystem = Subsystem.factory(key, subsystems.getJSONObject(key), mHardware, mLogger);
                    if(subsystem == null) {
                        mLogger.warning("Subsystem " + key + " not recognized by factory");
                        mConfigurationValid = false;
                    }
                    else if(!subsystem.isConfigured()) {
                        mLogger.warning("Subsystem " + key + " configuration is invalid");
                        mConfigurationValid = false;
                    }
                    else {
                        mSubsystems.put(key, subsystem);
                    }

                }
            }

        } catch (JSONException e) {
            mLogger.error(e.getMessage());
        }

    }

    /**
     * Writes the current robot configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    public void                         write(JSONObject writer) {

        if(mConfigurationValid) {

            try {

                JSONObject hardware = new JSONObject();
                mHardware.write(hardware);
                writer.put(sHardwareKey, hardware);

                JSONObject subsystems = new JSONObject();
                for (Map.Entry<String, Subsystem> subsystem : mSubsystems.entrySet()) {
                    JSONObject temp = new JSONObject();
                    subsystem.getValue().write(temp);
                    subsystems.put(subsystem.getKey(),temp);
                }
                writer.put(sSubsystemsKey, subsystems);
            }
            catch (JSONException e) { mLogger.error(e.getMessage()); }
        }

    }

    /**
     * Generates an HTML representation of the robot configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted actuator configuration.
     */
    public String                       logConfigurationHTML() {
        StringBuilder result = new StringBuilder();

        // Log Hardware
        result.append(mHardware.logConfigurationHTML());

        // Log subsystem
        result.append("<details style=\"margin-left:10px\">\n");
        result.append("<summary style=\"font-size: 12px; font-weight: 500\"> SUBSYSTEMS </summary>\n");
        result.append("<ul>\n");
        mSubsystems.forEach((key, value) -> result.append("<details style=\"margin-left:10px\">\n")
                .append("<summary style=\"font-size: 11px; font-weight: 500\"> ")
                .append(key.toUpperCase())
                .append(" </summary>\n")
                .append("<ul>\n")
                .append(value.logConfigurationHTML())
                .append("</ul>\n")
                .append("</details>\n"));
        result.append("</ul>\n");
        result.append("</details>\n");


        return result.toString();

    }

    /**
     * Generates a text-based representation of the robot configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted robot configuration details.
     */
    public String                       logConfigurationText(String header)
    {
        StringBuilder result = new StringBuilder();

        // Log hardware
        result.append(header)
                .append("> HARDWARE\n")
                .append(mHardware.logConfigurationText(header + "--"));

        // Log subsystems
        result.append(header)
                .append("> SUBSYSTEMS\n");
        mSubsystems.forEach((key, value) -> result.append(header)
                .append("--> ")
                .append(key)
                .append("\n")
                .append(value.logConfigurationText(header + "----")));


        return result.toString();

    }


}