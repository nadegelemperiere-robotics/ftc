/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Robot subsystem interface
   ------------------------------------------------------- */
package org.firstinspires.ftc.core.subsystems;

/* Json includes */
import org.json.JSONObject;
import org.json.JSONException;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;


public interface Subsystem extends Configurable {

    String sTypeKey = "type";

    static Subsystem   factory(String name, JSONObject reader, Hardware hardware, LogManager logger) {
        Subsystem result = null;
        try {
            if (reader.has(sTypeKey)) {
                String type = reader.getString(sTypeKey);
                switch (type) {
                    case "mecanum-drive" :
                        result = new MecanumDrive(name,hardware,logger);
                        result.read(reader);
                        break;
                    case "actuator" :
                        result = new Actuator(name,hardware,logger);
                        result.read(reader);
                        break;
                    case "toggle-actuator" :
                        result = new ToggleActuator(name,hardware,logger);
                        result.read(reader);
                        break;
                    case "default-slides" :
                        result = new DefaultSlides(name,hardware,logger);
                        result.read(reader);
                        break;
                }
            }
        }
        catch(JSONException e) { logger.error(e.getMessage());}

        return result;
    }

    void    update();
    boolean hasFinished();





}