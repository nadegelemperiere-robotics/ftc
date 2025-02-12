/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Robot manager
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.robot;

/* Json includes */
import org.json.JSONObject;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;


/* Sequencer includes */
import org.firstinspires.ftc.core.orchestration.sequencer.Context;

public class Robot { //} extends Context implements Configurable {

    LogManager  mLogger;

    public  Robot(LogManager logger) {
        mLogger = logger;
    }

}