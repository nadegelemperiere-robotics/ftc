/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   ServoComponent is an interface for servo management
   It supersedes Servo and provides additional capabilities
   such as :
   - Synchronizing 2 coupled servos
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.servos;

/* JSON includes */
import org.json.JSONObject;
import org.json.JSONArray;
import org.json.JSONException;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

public interface ServoComponent extends Configurable {

    String sHwMapKey      = "hwmap";
    String sReverseKey    = "reverse";

    static ServoComponent factory(String name, JSONArray reader, HardwareMap map, LogManager logger) {

        ServoComponent result = null;

        // Configure motor
        try {
            if (reader.length() == 0) {
                result = new ServoMock(name, logger);
            } else if (reader.length() == 1) {
                result = new ServoSingle(name, map, logger);
                result.read(reader.getJSONObject(0));
            } else if (reader.length() == 2) {
                JSONObject configuration = new JSONObject();
                configuration.put(ServoCoupled.sFirstKey, reader.getJSONObject(0));
                configuration.put(ServoCoupled.sSecondKey, reader.getJSONObject(1));
                result = new ServoCoupled(name, map, logger);
                result.read(configuration);
            } else {
                logger.error("Can not managed more than 3 coupled Servos");
            }
        } catch (JSONException e) { logger.error(e.getMessage()); }

        return result;

    }


    /* --------------------- Custom functions ---------------------- */

    String                      name();
    void                        log();

    /* ------------------ Configurable functions ------------------- */

    void                        read(JSONObject reader);
    void                        write(JSONObject writer);
    boolean                     isConfigured();
    String                      logConfigurationHTML();
    String                      logConfigurationText(String header);

    /* ------------------- Servo methods override ------------------ */

    ServoControllerComponent    controller();

    Servo.Direction             direction();
    double	                    position();
    void	                    scaleRange(double min, double max);

    void	                    direction(Servo.Direction direction);
    void	                    position(double position);

}
