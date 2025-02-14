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


public interface Subsystem extends Configurable {

    public static final String sTypeKey = "type";

    public static Subsystem   factory(JSONObject reader, LogManager logger) {
        Subsystem result = null;
        try {
            if (reader.has(sTypeKey)) {
                String type = reader.getString(sTypeKey);
                switch (reader.getString(sTypeKey)) {
                    case "mechanum-drive" :
                        //result = new MechanumDrive(logger);
                        //result.read(reader);
                        break;
                }
            }
        }
        catch(JSONException e) { logger.error(e.getMessage());}




        return result;
    }





}