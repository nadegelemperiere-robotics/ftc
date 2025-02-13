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

    private double      mValue1;
    private double      mValue2;

    public RobotTest(LogManager logger) {
        super(logger);
        mHasTest1BeenCalled = false;
        mValue1 = -1.0f;
        mValue2 = -1.0f;
    }

    public void test1() {
        mHasTest1BeenCalled = true;
    }

    public void test2(double value1, double value2) {
        mValue1 = value1;
        mValue2 = value2;
    }

    public boolean test1Check() { return mHasTest1BeenCalled;}

    public double  test2Value1() { return mValue1; }
    public double  test2Value2() { return mValue2; }

}

