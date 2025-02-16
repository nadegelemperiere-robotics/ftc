/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   State context for state machine
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.orchestration.engine;

public class Context {

    protected State mState;

    /**
     * Check if current state is over and move to next step
     */
    public void update() {
        if(mState.hasFinished()) { mState = mState.next(); }
    }


}