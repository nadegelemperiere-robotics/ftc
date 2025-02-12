/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Robot State generic design
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.robot;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;

/* Tools includes */
import org.firstinspires.ftc.core.orchestration.sequencer.Context;
import org.firstinspires.ftc.core.tools.LogManager;

/* Subsystems includes */
import org.firstinspires.ftc.core.subsystems.Subsystem;

/* Sequencer includes */
import org.firstinspires.ftc.core.orchestration.sequencer.State;

public class RobotState { //extends State {

    LogManager              mLogger;

    Map<String, Subsystem>  mSubsystems;

    public  RobotState(LogManager logger) {
        mLogger = logger;
    }

    /**
     * Step to next state
     *
     * @param context context to update
     */
    void        next(Context context) {}

}