/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Robot State generic design
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.robot;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.State;

public abstract class RobotState implements State {

    public abstract static class SharedData{};

    protected LogManager        mLogger;

    protected SharedData        mData;

    public  RobotState(SharedData data, LogManager logger) {
        mLogger = logger;
        mData   = data;
    }

}