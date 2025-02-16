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

    protected LogManager                mLogger;

    protected Map<String, Subsystem>    mSubsystems;

    protected boolean                   mConfigurationValid;

    protected Hardware                  mHardware;

    protected RobotState                mState;

    public  Robot(HardwareMap map, LogManager logger) {
        mLogger             = logger;

        mConfigurationValid = false;

        mHardware           = new Hardware(map, logger);
        mSubsystems         = new LinkedHashMap<>();
    }

    public void        update()
    {
        super.update();
        for (Map.Entry<String, Subsystem> subsystem : mSubsystems.entrySet()) {
            subsystem.getValue().update();
        }
    }

    public boolean                      isConfigured() { return mConfigurationValid;}


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
                    if(!subsystem.isConfigured()) {
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

    public String                       logConfigurationHTML() {
        StringBuilder result = new StringBuilder();

        // Log Hardware
        result.append("<p style=\"margin-left:10px; font-size: 12px; font-weight: 500\"> HARDWARE </p>")
                        .append(mHardware.logConfigurationHTML());

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