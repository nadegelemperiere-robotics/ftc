/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   State context for state machine
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.orchestration.sequencer;

public class Context {

    protected State mState;


    public void update() {
        if(mState.isFinished()) { mState.next(this); }
    }

    public void setState(State state) {
        this.mState = state;
    }

}