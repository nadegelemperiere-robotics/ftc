/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Transfer state
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.robot;

/* Roadrunner includes */
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

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

public class AutonomousSampleState extends RobotState {

    final Sequencer   mSequencer;

    /**
     * Constructs a transfer state with the associated sample transfer sequence
     *
     * @param data   The data shared among all states
     * @param logger The logging manager for error reporting and debugging.
     */
    public  AutonomousSampleState(SharedData data, LogManager logger) {
        super(data,logger);

        mLogger.info("start");

        mSequencer = new Sequencer(mLogger);

        TelemetryPacket telemetry = new TelemetryPacket();

        Action trajectory1 = ((SharedData)mData).chassis.trajectory(new Pose2d(new Vector2d(0,0),0))
                .setTangent(Math.PI)
                .splineTo(new Vector2d(-10.75,-11.25),5*Math.PI/4)
                .build();

        Action trajectory2 = ((SharedData)mData).chassis.trajectory(((SharedData)mData).chassis.finalPlannedPose())
                .lineToXConstantHeading(-11.5)
                .turn(Math.PI/4)
                .lineToXConstantHeading(-14)
                .build();

        Action trajectory3 = ((SharedData)mData).chassis.trajectory(((SharedData)mData).chassis.finalPlannedPose())
                .lineToXConstantHeading(-11)
                .turn(-7 * Math.PI/24)
                .build();

        Action trajectory4 = ((SharedData)mData).chassis.trajectory(((SharedData)mData).chassis.finalPlannedPose())
                .splineTo(new Vector2d(-51,-10),-Math.PI/2)
                .lineToYConstantHeading(14.5)
                .build();


        mSequencer.sequence(
                "AUTONOMOUS SAMPLE",
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
                        "Move to basket with initial sample while raising the slides",
                        () -> {
                            ((SharedData)mData).outtakeSlides.position("max",25,5000);
                        },
                        Condition.and(
                            new Condition(() -> ((SharedData)mData).outtakeSlides.hasFinished()),
                            new Condition(() -> trajectory1.run(telemetry))
                        )
                ),
                new Task(
                        "Elevate outtake slides",
                        () -> {
                            ((SharedData)mData).outtakeSlides.position("max",25,5000);
                        },
                        new Condition(() -> ((SharedData)mData).outtakeSlides.hasFinished())
                ),
                new Task(
                        "Position outtake arm",
                        () -> {
                            ((SharedData)mData).outtakeArm.position(OuttakeArm.Position.DROP);
                        },
                        new Condition(() -> ((SharedData)mData).outtakeArm.hasFinished())
                ),
                new Task(
                        "Open outtake claw",
                        () -> {
                            ((SharedData)mData).outtakeArm.toogleClaw();
                        },
                        new Condition(() -> ((SharedData)mData).outtakeArm.hasFinished())
                ),
                new Task(
                        "Move outtake slides back",
                        () -> {
                            ((SharedData)mData).outtakeSlides.position("min",25,5000);
                        },
                        new Condition(() -> ((SharedData)mData).outtakeSlides.hasFinished())
                ),
                new Task(
                        "Move to first sample",
                        () -> {},
                        new Condition(() -> trajectory2.run(telemetry))
                ),
                new Task("Extend slides",
                        () -> {
                            ((SharedData)mData).intakeSlides.position("autonomous-sample",10,5000);
                        },
                        new Condition(() -> ((SharedData)mData).intakeSlides.hasFinished())
                ),
                new Task("Position intake arm for grabbing",
                        () -> {
                            ((SharedData)mData).intakeArm.position(IntakeArm.Position.GRAB);
                        },
                        new Condition(() -> ((SharedData)mData).intakeArm.hasFinished())
                ),
                new Task("Grab specimen",
                        () -> {
                            ((SharedData)mData).intakeArm.toogleClaw();
                        },
                        new Condition(() -> ((SharedData)mData).intakeArm.hasFinished())
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
                        "Moving back to basket",
                        () -> {
                            ((SharedData)mData).intakeArm.position(IntakeArm.Position.INIT);
                        },
                        Condition.and(
                            new Condition(() -> trajectory3.run(telemetry)),
                            new Condition(() -> ((SharedData)mData).intakeArm.hasFinished())
                        )
                ),
                new Task(
                        "Elevate outtake slides",
                        () -> {
                            ((SharedData)mData).outtakeSlides.position("max",25,5000);
                        },
                        new Condition(() -> ((SharedData)mData).outtakeSlides.hasFinished())
                ),
                new Task(
                        "Position outtake arm",
                        () -> {
                            ((SharedData)mData).outtakeArm.position(OuttakeArm.Position.DROP);
                        },
                        new Condition(() -> ((SharedData)mData).outtakeArm.hasFinished())
                ),
                new Task(
                        "Open outtake claw",
                        () -> {
                            ((SharedData)mData).outtakeArm.toogleClaw();
                        },
                        new Condition(() -> ((SharedData)mData).outtakeArm.hasFinished())
                ),
                new Task(
                        "Move outtake slides to ascend position",
                        () -> {
                            ((SharedData)mData).outtakeSlides.position("autonomous-sample-ascend",25,5000);
                        },
                        new Condition(() -> ((SharedData)mData).outtakeSlides.hasFinished())
                ),
                new Task(
                        "Move to ascend area",
                        () -> {},
                        new Condition(() -> trajectory4.run(telemetry))
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
