
/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Timer returning the middle between current time and last time
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.tools;

public class MidPointTimer {

    private long        mStartTime;
    private long        mLastTime;

    public MidPointTimer(){
        mStartTime = System.nanoTime();
        mLastTime  = 0;
    }

    public  void reset() {
        mStartTime = System.nanoTime();
        mLastTime  = 0;
    }

    public double seconds() {
        long result = (System.nanoTime() - mStartTime);
        mLastTime = result;
        return 1e-9 * result;

    }

    public double split() {
        long current = (System.nanoTime() - mStartTime);
        double result = 0.5e-9 * (mLastTime + current);
        mLastTime = current;
        return result;
    }

}
