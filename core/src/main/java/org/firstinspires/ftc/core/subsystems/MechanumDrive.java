/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Mechanum Drive management
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.subsystems;

/* System includes */
import java.util.Map;

/* Tools includes */
import org.firstinspires.ftc.core.orchestration.sequencer.Context;
import org.firstinspires.ftc.core.tools.LogManager;

/* Subsystems includes */
import org.firstinspires.ftc.core.subsystems.Subsystem;

/* Sequencer includes */
import org.firstinspires.ftc.core.orchestration.sequencer.State;

public class MechanumDrive {

    LogManager              mLogger;



    public  MechanumDrive(LogManager logger) {
        mLogger = logger;
    }

    /**
     * Step to next state
     *
     * @param context context to update
     */
    void        next(Context context) {}

}