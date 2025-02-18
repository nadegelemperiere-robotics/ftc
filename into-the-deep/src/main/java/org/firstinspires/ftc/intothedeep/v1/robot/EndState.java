/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Transfer state
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.robot;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.State;

public class EndState extends RobotState {

    /**
     * Constructs a end state to mark the end of an autonomous state
     *
     * @param data   The data shared among all states
     * @param logger The logging manager for error reporting and debugging.
     */
    public  EndState(SharedData data, LogManager logger) {
        super(data,logger);


    }

    /* ---------------------- TeleOp commands ---------------------- */
    // No commands are allowed

    /* --------------------- States management --------------------- */

    /**
     * Declare the state finished once transfer sequence is over
     */
    @Override
    public boolean      hasFinished() { return true; }

    /**
     * Update sequencer periodically to switch between tasks
     */
    @Override
    public void         update() {  }

    /**
     * Transfer restart is not allowed until current is over
     *
     * @return this - meaning the state does not change
     */
    @Override
    public  RobotState  toTransfer() {
        mLogger.info("Transfer asked in end state: not taken into account");
        return this;
    }

    /**
     * Picking is not allowed until transfer is over
     *
     * @return this - meaning the state does not change
     */
    @Override
    public  RobotState  toPick() {
        mLogger.info("Pick asked in end state : not taken into account");
        return this;
    }

    /**
     * After transfer is over, switch to default state
     *
     * @return a default state to restore all commands
     */
    @Override
    public  State       next() {
        return this;
    }


}
