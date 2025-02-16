/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Timer for actuators timeout
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.tools;

public class Timer {

    LogManager      mLogger;

    private long    mStartTime;
    private boolean mIsRunning;
    private boolean mHasAlreadyBeenCalled;
    private int     mTarget;

    public Timer(LogManager logger){
        mIsRunning = false;
        mHasAlreadyBeenCalled = false;
        mLogger = logger;
    }

    public void arm(int milliseconds)
    {
        mStartTime = System.nanoTime();
        mIsRunning = true;
        mTarget = milliseconds;
        mHasAlreadyBeenCalled = true;
    }

    public boolean isArmed()
    {
        if(mHasAlreadyBeenCalled) {
            double delta = (System.nanoTime() - mStartTime) / 1_000_000.0;
            if (delta >= mTarget) { mIsRunning = false; }
        }
        return mIsRunning;
    }

}