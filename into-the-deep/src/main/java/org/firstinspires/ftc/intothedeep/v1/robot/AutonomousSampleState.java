/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Transfer state
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.robot;

/* PedroPathing includes */
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

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

        Pose initial = ((SharedData)mData).chassis.getPose();
        Pose underTheBasket = new Pose(10.75, 11.25,-Math.PI/4);
        Pose sample1 = new Pose(14, 10.5,0);
        Pose preAscend = new Pose(51, 10, Math.PI / 2);
        Pose ascend = new Pose(51, -14.5, Math.PI / 2);

        PathChain trajectory1 = ((SharedData)mData).chassis.pathBuilder()
                .addPath(new BezierLine(new Point(initial), new Point(underTheBasket)))
                .setTangentHeadingInterpolation()
                .build();

        PathChain trajectory2 = ((SharedData)mData).chassis.pathBuilder()
                .addPath(new BezierLine(new Point(underTheBasket), new Point(sample1)))
                .setLinearHeadingInterpolation(underTheBasket.getHeading(), sample1.getHeading())
                .build();

        PathChain trajectory3 = ((SharedData)mData).chassis.pathBuilder()
                .addPath(new BezierLine(new Point(sample1), new Point(underTheBasket)))
                .setLinearHeadingInterpolation(sample1.getHeading(), underTheBasket.getHeading()).build();

        PathChain trajectory4 = ((SharedData)mData).chassis.pathBuilder()
                .addPath(new BezierLine(new Point(underTheBasket), new Point(preAscend)))
                .setLinearHeadingInterpolation(underTheBasket.getHeading(),preAscend.getHeading())
                .addPath(new BezierLine(new Point(preAscend), new Point(ascend)))
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
                            ((SharedData)mData).chassis.followPath(trajectory1);
                        },
                        Condition.and(
                            new Condition(() -> ((SharedData)mData).outtakeSlides.hasFinished()),
                            new Condition(() -> ((SharedData)mData).chassis.hasFinished())
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
                        () -> {
                            ((SharedData)mData).chassis.followPath(trajectory2);
                        },
                        new Condition(() -> ((SharedData)mData).chassis.hasFinished())
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
                            ((SharedData)mData).chassis.followPath(trajectory3);
                        },
                        Condition.and(
                            new Condition(() -> ((SharedData)mData).chassis.hasFinished()),
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
                        () -> {
                            ((SharedData)mData).chassis.followPath(trajectory4);
                        },
                        new Condition(() -> ((SharedData)mData).chassis.hasFinished())
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
