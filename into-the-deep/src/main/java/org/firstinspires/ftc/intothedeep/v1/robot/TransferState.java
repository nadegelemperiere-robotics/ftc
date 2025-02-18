/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Transfer state
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

public class TransferState extends RobotState {

    final Sequencer   mSequencer;

    /**
     * Constructs a transfer state with the associated sample transfer sequence
     *
     * @param data   The data shared among all states
     * @param logger The logging manager for error reporting and debugging.
     */
    public  TransferState(SharedData data, LogManager logger) {
        super(data,logger);

        mSequencer = new Sequencer(mLogger);
        mSequencer.sequence(
                "TRANSFER",
                new Task(
                        "Stop all mechanisms motors",
                        () -> {
                            ((SharedData)mData).intakeSlides.power(0);
                            ((SharedData)mData).outtakeSlides.power(0);
                        },
                        new Condition(()-> true)
                ),
                new Task(
                        "Move slides",
                        () -> {
                            ((SharedData)mData).intakeSlides.position("transfer-away", 40,2000);
                            ((SharedData)mData).outtakeSlides.position("transfer",5, 5000);
                        },
                        Condition.and(
                                new Condition(() -> ((SharedData)mData).intakeSlides.hasFinished()),
                                new Condition(() -> ((SharedData)mData).outtakeSlides.hasFinished())
                        )
                ),
                new Task(
                        "Position arms",
                        () -> {
                            ((SharedData)mData).intakeArm.position(IntakeArm.Position.TRANSFER);
                            ((SharedData)mData).outtakeArm.position(OuttakeArm.Position.TRANSFER);
                        },
                        Condition.and(
                                new Condition(() -> ((SharedData)mData).intakeArm.hasFinished()),
                                new Condition(() -> ((SharedData)mData).outtakeArm.hasFinished())
                        )
                ),
                new Task(
                        "Micro release inttake claw",
                        () -> {
                            ((SharedData)mData).intakeArm.microrelease();
                        },
                        new Condition(() -> ((SharedData)mData).intakeArm.hasFinished())
                ),
                new Task(
                        "Close inttake claw",
                        () -> {
                            ((SharedData)mData).intakeArm.close();
                        },
                        new Condition(() -> ((SharedData)mData).intakeArm.hasFinished())
                ),
                new Task(
                        "Move intake slides to exchange position",
                        () -> {
                            ((SharedData)mData).intakeSlides.position("transfer-exchange", 40,2000);
                        },
                        new Condition(() -> ((SharedData)mData).intakeSlides.hasFinished())
                ),
                new Task(
                        "Close outtake claw",
                        () -> {
                            ((SharedData)mData).outtakeArm.toogleClaw();
                        },
                        new Condition(() -> ((SharedData)mData).outtakeArm.hasFinished())
                ),
                new Task(
                        "Open inttake claw",
                        () -> {
                            ((SharedData)mData).intakeArm.toogleClaw();
                        },
                        new Condition(() -> ((SharedData)mData).intakeArm.hasFinished())
                ),
                new Task(
                        "Release slides",
                        () -> {
                            // power slides to stop running to position and release motor constraint
                            ((SharedData)mData).intakeSlides.power(0.05);
                            ((SharedData)mData).outtakeSlides.power(0.05);
                        },
                        new Condition(() -> true)
                )
        );

        mSequencer.run();

    }

    /* ---------------------- TeleOp commands ---------------------- */
    // Only driving is allowed in this state
    @Override
    public  void        drive(double x, double y, double heading) { ((SharedData)mData).chassis.drive(x,y,heading); }

    @Override
    public  void        tuneDriveSpeed(double multiplier) { ((SharedData)mData).chassis.driveSpeedMultiplier(multiplier); }

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
     * Transfer restart is not allowed until current is over
     *
     * @return this - meaning the state does not change
     */
    @Override
    public  RobotState  toTransfer() {
        mLogger.info("Transfer asked while transfer is on-going : not taken into account");
        return this;
    }

    /**
     * Picking is not allowed until transfer is over
     *
     * @return this - meaning the state does not change
     */
    @Override
    public  RobotState  toPick() {
        mLogger.info("Pick asked while transfer is on-going : not taken into account");
        return this;
    }

    /**
     * After transfer is over, switch to default state
     *
     * @return a default state to restore all commands
     */
    @Override
    public  State       next() {
        mLogger.metric("STATE","TRANSFER -> DEFAULT");
        return new DefaultState(((SharedData)mData), mLogger);
    }


}
