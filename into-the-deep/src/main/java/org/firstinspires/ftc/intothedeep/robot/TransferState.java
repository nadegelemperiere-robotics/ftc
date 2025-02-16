/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Season robot configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.robot;

/* Tools includes */
import org.firstinspires.ftc.core.orchestration.sequencer.Task;
import org.firstinspires.ftc.core.tools.LogManager;

/* Robot includes */
import org.firstinspires.ftc.core.robot.RobotState;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.sequencer.State;
import org.firstinspires.ftc.core.orchestration.sequencer.Sequencer;
import org.firstinspires.ftc.core.orchestration.scheduler.Condition;
import org.firstinspires.ftc.intothedeep.subsystems.IntakeArm;
import org.firstinspires.ftc.intothedeep.subsystems.OuttakeArm;

public class TransferState extends SeasonRobotState {

    Sequencer   mSequencer;

    public  TransferState(SeasonSharedData data, LogManager logger) {
        super(data,logger);

        mSequencer = new Sequencer(mLogger);
        mSequencer.sequence(
                "TRANSFER",
                new Task(
                        "Move slides",
                        () -> {
                            ((SeasonSharedData)mData).intakeSlides.position("transfer-away", 40,2000);
                            ((SeasonSharedData)mData).outtakeSlides.position("transfer",5, 5000);
                        },
                        Condition.and(
                                new Condition(() -> ((SeasonSharedData)mData).intakeSlides.hasFinished()),
                                new Condition(() -> ((SeasonSharedData)mData).outtakeSlides.hasFinished())
                        )
                ),
                new Task(
                        "Position arms",
                        () -> {
                            ((SeasonSharedData)mData).intakeArm.position(IntakeArm.Position.TRANSFER);
                            ((SeasonSharedData)mData).outtakeArm.position(OuttakeArm.Position.TRANSFER);
                        },
                        Condition.and(
                                new Condition(() -> ((SeasonSharedData)mData).intakeArm.hasFinished()),
                                new Condition(() -> ((SeasonSharedData)mData).outtakeArm.hasFinished())
                        )
                ),
                new Task(
                        "Micro release inttake claw",
                        () -> {
                            ((SeasonSharedData)mData).intakeArm.microrelease();
                        },
                        new Condition(() -> ((SeasonSharedData)mData).intakeArm.hasFinished())
                ),
                new Task(
                        "Close inttake claw",
                        () -> {
                            ((SeasonSharedData)mData).intakeArm.close();
                        },
                        new Condition(() -> ((SeasonSharedData)mData).intakeArm.hasFinished())
                ),
                new Task(
                        "Move intake slides to exchange position",
                        () -> {
                            ((SeasonSharedData)mData).intakeSlides.position("transfer-exchange", 40,2000);
                        },
                        new Condition(() -> ((SeasonSharedData)mData).intakeSlides.hasFinished())
                ),
                new Task(
                        "Close outtake claw",
                        () -> {
                            ((SeasonSharedData)mData).outtakeArm.toogleClaw();
                        },
                        new Condition(() -> ((SeasonSharedData)mData).outtakeArm.hasFinished())
                ),
                new Task(
                        "Open inttake claw",
                        () -> {
                            ((SeasonSharedData)mData).intakeArm.toogleClaw();
                        },
                        new Condition(() -> ((SeasonSharedData)mData).intakeArm.hasFinished())
                ),
                new Task(
                        "Release slides",
                        () -> {
                            // power slides to stop running to position and release motor constraint
                            ((SeasonSharedData)mData).intakeSlides.power(0.05);
                            ((SeasonSharedData)mData).outtakeSlides.power(0.05);
                        },
                        new Condition(() -> true)
                )
        );

        mSequencer.run();

    }

    @Override
    public  void        drive(double x, double y, double heading) { ((SeasonSharedData)mData).chassis.drive(x,y,heading); }

    @Override
    public  void        tuneDriveSpeed(double multiplier) { ((SeasonSharedData)mData).chassis.driveSpeedMultiplier(multiplier); }

    @Override
    public  RobotState  toTransfer() {

        mLogger.info("Transfer asked while transfer is on-going : not taken into account");
        return this;
    }

    @Override
    public  RobotState  toPick() {

        mLogger.info("Pick asked while transfer is on-going : not taken into account");
        return this;
    }

    @Override
    public  State       next() {
        mLogger.info("Transfer over : switching back to default state");
        return new DefaultState(((SeasonSharedData)mData), mLogger);
    }

    @Override
    public boolean      hasFinished() {
        return mSequencer.hasFinished();
    }



}
