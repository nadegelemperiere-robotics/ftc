/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Robot implementation for tests
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.test.orchestration;


/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Robot;


public class RobotTest extends Robot {

    private boolean     mHasTest1BeenCalled;

    private double      mValue1;
    private double      mValue2;
    private double      mValue3;
    private double      mValue4;
    private String      mValue5;

    private double      mX;
    private double      mY;
    private double      mHeading;

    public RobotTest(LogManager logger) {
        super(null,logger);
        reset();
    }

    public void test1() {
        mHasTest1BeenCalled = true;
    }

    public void test2(double value1, double value2, String value3) {
        mValue1 = value1;
        mValue2 = value2;
        mValue5 = value3;
    }

    public void drive(double x, double y, double heading) {
        mX = x;
        mY = y;
        mHeading = heading;
    }

    public void command(double value3, double value4){
        mValue3 = value3;
        mValue4 = value4;
    }

    public boolean test1Check() { return mHasTest1BeenCalled;}

    public double  test2Value1() { return mValue1; }
    public double  test2Value2() { return mValue2; }
    public String  test2Value3() { return mValue5; }

    public double  x()           { return mX; }
    public double  y()           { return mY; }
    public double  heading()     { return mHeading; }

    public double  commandValue3() { return mValue3; }
    public double  commandValue4() { return mValue4; }

    public void    reset() {
        mHasTest1BeenCalled = false;
        mValue1     = -10000.0f;
        mValue2     = -10000.0f;
        mValue3     = -10000.0f;
        mValue4     = -10000.0f;
        mValue5     = "";
        mX          = -10000.0f;
        mY          = -10000.0f;
        mHeading    = -10000.0f;
    }


}

