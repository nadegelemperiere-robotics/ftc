/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Class mapping controller inputs to robot behavior
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.orchestration.controller;

/* System includes */
import java.util.Map;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Components includes */
import org.firstinspires.ftc.core.components.controllers.Controller;


public class ControlMapper {

    LogManager              mLogger;

    Map<String,Controller>  mControllers;

    /**
     * ControlMapper constructor
     *
     * @param logger logger
     */
    public ControlMapper(Map<String,Controller> controllers, LogManager logger) {
        mLogger      = logger;
        mControllers = controllers;
    }

    public void loop() {

    }


}
