/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Robot subsystem interface
   ------------------------------------------------------- */
package org.firstinspires.ftc.core.subsystems;

/* Json includes */
import org.json.JSONObject;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

public interface Subsystem extends Configurable {

    public static final String sTypeKey = "type";

    public static Subsystem   factory(JSONObject reader) {
        Subsystem result = null;




        return result;
    }



}