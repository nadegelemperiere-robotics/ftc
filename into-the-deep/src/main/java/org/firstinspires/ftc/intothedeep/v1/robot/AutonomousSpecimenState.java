/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Transfer state
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.robot;

/* Tools includes */
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.core.tools.LogManager;
import org.firstinspires.ftc.core.tools.Condition;

/* Subsystems includes */
import org.firstinspires.ftc.intothedeep.v1.subsystems.IntakeArm;
import org.firstinspires.ftc.intothedeep.v1.subsystems.OuttakeArm;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.State;
import org.firstinspires.ftc.core.orchestration.engine.Task;
import org.firstinspires.ftc.core.orchestration.engine.Sequencer;

public class AutonomousSpecimenState extends RobotState {

    final Sequencer   mSequencer;

    /**
     * Constructs a transfer state with the associated sample transfer sequence
     *
     * @param data   The data shared among all states
     * @param logger The logging manager for error reporting and debugging.
     */
    public  AutonomousSpecimenState(SharedData data, LogManager logger) {
        super(data,logger);

        mLogger.info("start");

        mSequencer = new Sequencer(mLogger);

        TelemetryPacket telemetry = new TelemetryPacket();

        Action trajectory1 = ((SharedData)mData).chassis.trajectory(new Pose2d(new Vector2d(0,0),0))
                .lineToX(-31)
                .build();

        Action trajectory2 = ((SharedData)mData).chassis.trajectory(((SharedData)mData).chassis.finalPlannedPose())
                .lineToX(-27)
                .build();

        Action trajectory3 = ((SharedData)mData).chassis.trajectory(((SharedData)mData).chassis.finalPlannedPose())
                .lineToX(-15)
                .turn(Math.PI/2)
                .lineToY(39.5)
                .turn(Math.PI/2)
                .lineToX(-5)
                .build();

        Action trajectory4 = ((SharedData)mData).chassis.trajectory(((SharedData)mData).chassis.finalPlannedPose())
                .lineToX(-10)
                .turn(-Math.PI/2)
                .lineToY(-10)
                .turn(-Math.PI/2)
                .lineToX(-25.5)
                .build();

        Action trajectory5 = ((SharedData)mData).chassis.trajectory(((SharedData)mData).chassis.finalPlannedPose())
                .lineToX(-5)
                .turn(Math.PI/2)
                .lineToY(39)
                .build();

        mSequencer.sequence(
                "AUTONOMOUS SPECIMEN",
                new Task(
                        "Init arms",
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
                ),
                new Task(
                        "Move to submersible with initial specimen",
                        () -> {},
                        new Condition(() -> trajectory1.run(telemetry))
                ),
                new Task(
                        "Position outtake elbow to pass under the submersible bar",
                        () -> {
                            ((SharedData)mData).outtakeArm.position(OuttakeArm.Position.SPECIMEN_BEHIND);
                        },
                        new Condition(() -> ((SharedData)mData).outtakeSlides.hasFinished())
                ),
                new Task(
                        "Elevate outtake slides",
                        () -> {
                            ((SharedData)mData).outtakeSlides.position("autonomous-specimen-submersible-under",25,5000);
                        },
                        new Condition(() -> ((SharedData)mData).outtakeSlides.hasFinished())
                ),
                new Task(
                        "Position outtake elbow to align the hook with the bar",
                        () -> {
                            ((SharedData)mData).outtakeArm.position(OuttakeArm.Position.VERTICAL);
                        },
                        new Condition(() -> ((SharedData)mData).outtakeSlides.hasFinished())
                ),
                new Task(
                        "Move so that the hook clips on the bar",
                        () -> {},
                        new Condition(() -> trajectory2.run(telemetry))
                ),
                new Task(
                        "Open outtake claw",
                        () -> {
                            ((SharedData)mData).outtakeArm.toogleClaw();
                        },
                        new Condition(() -> ((SharedData)mData).outtakeArm.hasFinished())
                ),
                new Task(
                        "Move to the observation zone to collect specimen",
                        () -> {
                            ((SharedData)mData).outtakeArm.position(OuttakeArm.Position.SPECIMEN_AHEAD);
                            ((SharedData)mData).outtakeSlides.position("transfer",5,5000);
                        },
                        new Condition(() -> true)
                ),
                new Task(
                        "Close claw to grab specimen",
                        () -> {
                            ((SharedData)mData).outtakeArm.toogleClaw();
                        },
                        new Condition(() -> ((SharedData)mData).outtakeArm.hasFinished())
                ),
                new Task(
                        "Retrieve arm before moving",
                        () -> {
                            ((SharedData)mData).outtakeArm.position(OuttakeArm.Position.INIT);
                        },
                        new Condition(() -> ((SharedData)mData).outtakeArm.hasFinished())
                ),
                new Task(
                        "Move to the submersible",
                        () -> {},
                        new Condition(() -> trajectory3.run(telemetry))
                ),
                new Task(
                        "Elevate outtake slides",
                        () -> {
                            ((SharedData)mData).outtakeSlides.position("autonomous-specimen-submersible-over",25,5000);
                        },
                        new Condition(() -> ((SharedData)mData).outtakeSlides.hasFinished())
                ),
                new Task(
                        "Position outtake elbow to position specimen on bar",
                        () -> {
                            ((SharedData)mData).outtakeArm.position(OuttakeArm.Position.SPECIMEN_AHEAD);
                        },
                        new Condition(() -> ((SharedData)mData).outtakeArm.hasFinished())
                ),
                new Task(
                        "Ultraclose outtake claw",
                        () -> {
                            ((SharedData)mData).outtakeArm.ultraclose();
                        },
                        new Condition(() -> ((SharedData)mData).outtakeArm.hasFinished())
                ),
                new Task(
                        "Move towards the submersible to clip",
                        () -> {},
                        new Condition(() -> trajectory4.run(telemetry))
                ),
                new Task(
                        "Open outtake claw",
                        () -> {
                            ((SharedData)mData).outtakeArm.toogleClaw();
                        },
                        new Condition(() -> ((SharedData)mData).outtakeArm.hasFinished())
                ),
                new Task(
                        "Move back to observation zone to park",
                        () -> {
                            ((SharedData)mData).outtakeArm.position(OuttakeArm.Position.INIT);
                            ((SharedData)mData).outtakeSlides.position("transfer",5,5000);
                        },
                        new Condition(() -> trajectory5.run(telemetry))
                )

        );

        mSequencer.run();

        mLogger.info("stop");

    }

    /* ---------------------- TeleOp commands ---------------------- */
    // No commands are allowed in autonomous

    /* --------------------- States management --------------------- */

    /**
     * Declare the state finished once autonomous is over
     */
    @Override
    public boolean      hasFinished() { return mSequencer.hasFinished(); }

    /**
     * Update sequencer periodically to switch between tasks
     */
    @Override
    public void         update() { mSequencer.update(); }

    /**
     * Transfer is not allowed in autonomous
     *
     * @return this - meaning the state does not change
     */
    @Override
    public  RobotState  toTransfer() {
        mLogger.info("Transfer asked in autonomous : not taken into account");
        return this;
    }

    /**
     * Picking is not allowed in autonomous
     *
     * @return this - meaning the state does not change
     */
    @Override
    public  RobotState  toPick() {
        mLogger.info("Pick asked in autonomous : not taken into account");
        return this;
    }

    /**
     * After autonomous is over, switch to end state
     *
     * @return a end state
     */
    @Override
    public  State       next() {
        mLogger.metric("STATE","AUTONOMOUS -> END");
        return new EndState(((SharedData)mData), mLogger);
    }


}
