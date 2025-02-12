/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   State interface for state machine
   ------------------------------------------------------- */
package org.firstinspires.ftc.core.orchestration.sequencer;

public interface State {

    /**
     * Step to next state
     *
     * @param context context to update
     */
    void        next(Context context);

    boolean     isFinished();

}