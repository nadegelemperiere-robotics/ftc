/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   State interface for state machine
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.orchestration.engine;

public interface State {

    /**
     * Step to next state
     */
    State       next();

    /**
     * Check if state is ready for change
     *
     * @return true if state is ready, false otherwise
     */
    boolean     hasFinished();

}