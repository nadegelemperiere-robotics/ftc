/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Season robot configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.orchestration;

/* System includes */

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Controller includes */
import org.firstinspires.ftc.core.orchestration.controller.Scheduler;
import org.firstinspires.ftc.core.orchestration.controller.Condition;


public class SeasonScheduler extends Scheduler {

    public SeasonScheduler(LogManager logger) {
        super(logger);
    }

    @Override
    private     commands() {

        registerCommand(
                new Condition(() -> mControllers.get("mechanisms").buttons.left_bumper.pressedOnce(), mLogger),
                () -> mRobot.powerOuttakeSlides(0.9));



    }


}
