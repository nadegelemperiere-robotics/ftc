/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Season robot configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.orchestration;

/* System includes */
import java.util.Map;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Components includes */
import org.firstinspires.ftc.core.components.controllers.Controller;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Robot;


public class Dispatcher extends org.firstinspires.ftc.core.orchestration.dispatcher.Dispatcher {

    public Dispatcher(Map<String, Controller> controllers, Robot robot, LogManager logger) {
        super(controllers, robot, logger);
    }

    @Override
    protected     void commands() {


    }


}
