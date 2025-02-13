/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Robot implementation for tests
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.orchestration.test;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Robot;


public class RobotTest extends Robot {

    private boolean     mHasTest1BeenCalled;

    public RobotTest(LogManager logger) {
        super(logger);
        mHasTest1BeenCalled = false;
    }

    public void test1() {
        mHasTest1BeenCalled = true;
    }

    public boolean test1Check() { return mHasTest1BeenCalled;}

}

