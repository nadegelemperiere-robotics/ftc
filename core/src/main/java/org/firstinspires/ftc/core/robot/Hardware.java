/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Hardware manager
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.robot;

/* System includes */
import java.util.Map;
import java.util.LinkedHashMap;
import java.util.Iterator;
import java.util.List;
import java.util.ArrayList;

/* Json includes */
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONArray;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

/* Components includes */
import org.firstinspires.ftc.core.components.motors.MotorComponent;
import org.firstinspires.ftc.core.components.motors.MotorCoupled;

public class Hardware implements Configurable {

    static final String sMotorsKey = "motors";
    static final String sImusKey = "imus";
    static final String sServosKey = "servos";
    static final String sOdometersKey = "odometers";


    LogManager mLogger;

    boolean mConfigurationValid;

    HardwareMap mMap;

    Map<String, MotorComponent> mMotors;

    Map<String, MotorComponent> mMotorsForTuning;


    public Hardware(HardwareMap map, LogManager logger) {

        mLogger = logger;

        mConfigurationValid = true;

        mMap = map;

        mMotors = new LinkedHashMap<>();
        mMotorsForTuning = new LinkedHashMap<>();

    }

    public Map<String,MotorComponent>   motors() { return mMotors; }

    public boolean                      isConfigured() { return mConfigurationValid;}

    public void                         read(JSONObject reader) {

        mConfigurationValid = true;
        try {

            // Read Motors
            if (reader.has(sMotorsKey)) {

                JSONObject motors = reader.getJSONObject(sMotorsKey);
                Iterator<String> keys = motors.keys();
                while (keys.hasNext()) {

                    String key = keys.next();

                    MotorComponent motor = MotorComponent.factory(key, motors.getJSONArray(key), mMap, mLogger);
                    if (!motor.isConfigured()) {
                        mLogger.warning("Motor " + key + " configuration is invalid");
                        mConfigurationValid = false;
                    } else {
                        mMotors.put(key, motor);
                    }

                }
            }
        } catch (JSONException e) {
            mLogger.error(e.getMessage());
        }
    }

    public void                         write(JSONObject writer) {

        try {

            // Write motors
            JSONObject motors = new JSONObject();
            for (Map.Entry<String, MotorComponent> motor : mMotors.entrySet()) {
                JSONArray array = new JSONArray();
                JSONObject temp = new JSONObject();
                motor.getValue().write(temp);
                if (temp.has(MotorCoupled.sFirstKey)) {
                    array.put(temp.getJSONObject(MotorCoupled.sFirstKey));
                }
                if (temp.has(MotorCoupled.sSecondKey)) {
                    array.put(temp.getJSONObject(MotorCoupled.sSecondKey));
                }
                if (!temp.has(MotorCoupled.sSecondKey) && !temp.has(MotorCoupled.sSecondKey)) {
                    array.put(temp);
                }
                motors.put(motor.getKey(), array);
            }
            writer.put(sMotorsKey, motors);

        } catch (JSONException e) { mLogger.error(e.getMessage()); }
    }

    public String                       logConfigurationHTML()
    {
        StringBuilder result = new StringBuilder();

        // Log motors
        result.append("<details style=\"margin-left:10px\">\n");
        result.append("<summary style=\"font-size: 12px; font-weight: 500\"> MOTORS </summary>\n");
        result.append("<ul>\n");
        mMotors.forEach((key, value) -> {
            result.append("<details style=\"margin-left:10px\">\n")
                    .append("<summary style=\"font-size: 11px; font-weight: 500\"> ")
                    .append(key.toUpperCase())
                    .append(" </summary>\n")
                    .append("<ul>\n")
                    .append(value.logConfigurationHTML())
                    .append("</ul>\n")
                    .append("</details>\n");
        });
        result.append("</ul>\n");
        result.append("</details>\n");

        return result.toString();

    }

    public String                       logConfigurationText(String header)
    {
        StringBuilder result = new StringBuilder();

        // Log motors
        result.append(header)
                .append("> MOTORS\n");

         mMotors.forEach((key, value) -> {
            result.append(header)
                    .append("--> ")
                    .append(key)
                    .append("\n")
                    .append(value.logConfigurationText(header + "----"));
        });

        return result.toString();

    }
}