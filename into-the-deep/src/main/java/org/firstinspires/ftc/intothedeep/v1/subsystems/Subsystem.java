/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Robot subsystem interface
   ------------------------------------------------------- */
package org.firstinspires.ftc.intothedeep.v1.subsystems;

/* Json includes */
import org.json.JSONObject;
import org.json.JSONException;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

public interface Subsystem extends org.firstinspires.ftc.core.subsystems.Subsystem {

    String sTypeKey = "type";

    static org.firstinspires.ftc.core.subsystems.Subsystem   factory(String name, JSONObject reader, Hardware hardware, LogManager logger) {

        org.firstinspires.ftc.core.subsystems.Subsystem result = null;

        result = org.firstinspires.ftc.core.subsystems.Subsystem.factory(name, reader, hardware, logger);
        if(result == null) {
            try {
                if (reader.has(sTypeKey)) {
                    String type = reader.getString(sTypeKey);
                    switch (type) {
                        case "intake-arm":
                            result = new IntakeArm(name, hardware, logger);
                            result.read(reader);
                            break;
                        case "outtake-arm":
                            result = new OuttakeArm(name, hardware, logger);
                            result.read(reader);
                            break;
                    }
                }
            } catch (JSONException e) { logger.error(e.getMessage()); }
        }
        return result;
    }
}