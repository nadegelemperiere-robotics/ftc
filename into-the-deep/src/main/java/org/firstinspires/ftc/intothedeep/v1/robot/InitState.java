/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Init state
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.robot;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;
import org.firstinspires.ftc.core.tools.Condition;

/* Subsystems includes */
import org.firstinspires.ftc.intothedeep.v1.subsystems.IntakeArm;
import org.firstinspires.ftc.intothedeep.v1.subsystems.OuttakeArm;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.State;
import org.firstinspires.ftc.core.orchestration.engine.Task;
import org.firstinspires.ftc.core.orchestration.engine.Sequencer;

public class InitState extends RobotState {

    Sequencer   mSequencer;

    public  InitState(SharedData data, LogManager logger) {
        super(data,logger);

        mSequencer = new Sequencer(mLogger);
        mSequencer.sequence(
                "INIT",
                new Task(
                        "Move arms",
                        () -> {
                            ((SharedData)mData).intakeArm.position(IntakeArm.Position.INIT);
                            ((SharedData)mData).intakeSlides.power(0);
                            ((SharedData)mData).outtakeArm.position(OuttakeArm.Position.INIT);
                            ((SharedData)mData).outtakeSlides.power(0);
                            ((SharedData)mData).chassis.drive(0,0,0);
                        },
                        Condition.and(
                                new Condition(() -> ((SharedData)mData).intakeArm.hasFinished()),
                                new Condition(() -> ((SharedData)mData).outtakeArm.hasFinished())
                        )
                )
        );

        mSequencer.run();

    }

    /* ---------------------- TeleOp commands ---------------------- */
    // No commands are allowed until initialization is over

    /* --------------------- States management --------------------- */
    /**
     * Declare the state finished once transfer sequence is over
     */
    @Override
    public boolean      hasFinished() { return mSequencer.hasFinished(); }

    /**
     * Update sequencer periodically to switch between tasks
     */
    @Override
    public void         update() { mSequencer.update(); }

    /**
     * Transfer is not allowed until init is over
     *
     * @return this - meaning the state does not change
     */
    @Override
    public  RobotState  toTransfer() {

        mLogger.info("Transfer asked while init is on-going : not taken into account");
        return this;
    }

    /**
     * Picking is not allowed until init is over
     *
     * @return this - meaning the state does not change
     */
    @Override
    public  RobotState  toPick() {
        mLogger.metric("STATE","INIT -> PICK");
        return this;
    }

    /**
     * After init is over, switch to default state
     *
     * @return a default state to activate all commands
     */
    @Override
    public  State       next() {
        mLogger.metric("STATE","INIT -> DEFAULT");
        return new DefaultState(((SharedData)mData), mLogger);
    }



}
