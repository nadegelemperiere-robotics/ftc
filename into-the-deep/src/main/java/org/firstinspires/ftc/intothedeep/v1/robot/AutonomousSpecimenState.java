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

        Pose initial = ((SharedData)mData).chassis.getPose();
        Pose clipSpecimen1 = new Pose(31, 0,-Math.PI);
        Pose lockSpecimen1 = new Pose(27, 0,-Math.PI);
        Pose preGrabSpecimen = new Pose(15, -39.5, 0);
        Pose grabSpecimen = new Pose(5, -39.5, 0);
        Pose clipSpecimen2 = new Pose(25.5,10,-Math.PI);
        Pose lockSpecimen2 = new Pose(30, 0,-Math.PI);
        Pose park = new Pose(39,5,-Math.PI/2);

        PathChain trajectory1 = ((SharedData)mData).chassis.pathBuilder()
                .addPath(new BezierLine(new Point(initial), new Point(clipSpecimen1)))
                .setLinearHeadingInterpolation(initial.getHeading(), clipSpecimen1.getHeading())
                .build();

        PathChain trajectory2 = ((SharedData)mData).chassis.pathBuilder()
                .addPath(new BezierLine(new Point(clipSpecimen1), new Point(lockSpecimen1)))
                .setLinearHeadingInterpolation(clipSpecimen1.getHeading(), lockSpecimen1.getHeading())
                .build();

        PathChain trajectory3 = ((SharedData)mData).chassis.pathBuilder()
                .addPath(new BezierLine(new Point(lockSpecimen1), new Point(preGrabSpecimen)))
                .setLinearHeadingInterpolation(lockSpecimen1.getHeading(), preGrabSpecimen.getHeading())
                .addPath(new BezierLine(new Point(preGrabSpecimen), new Point(grabSpecimen)))
                .build();

        PathChain trajectory4 = ((SharedData)mData).chassis.pathBuilder()
                .addPath(new BezierLine(new Point(grabSpecimen), new Point(preGrabSpecimen)))
                .addPath(new BezierLine(new Point(preGrabSpecimen), new Point(clipSpecimen2)))
                .build();

        PathChain trajectory5 = ((SharedData)mData).chassis.pathBuilder()
                .addPath(new BezierLine(new Point(clipSpecimen2), new Point(lockSpecimen2)))
                .build();

        PathChain trajectory6 = ((SharedData)mData).chassis.pathBuilder()
                .addPath(new BezierLine(new Point(lockSpecimen2), new Point(clipSpecimen2)))
                .addPath(new BezierLine(new Point(clipSpecimen2), new Point(park)))
                .setLinearHeadingInterpolation(clipSpecimen2.getHeading(), park.getHeading())
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
                        () -> {
                            ((SharedData)mData).chassis.followPath(trajectory1);
                        },
                        new Condition(() -> ((SharedData)mData).chassis.hasFinished())
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
                        () -> {
                            ((SharedData)mData).chassis.followPath(trajectory2);
                        },
                        new Condition(() -> ((SharedData)mData).chassis.hasFinished())
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
                            ((SharedData)mData).chassis.followPath(trajectory3);
                        },
                        new Condition(() -> ((SharedData)mData).chassis.hasFinished())
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
                        () -> {
                            ((SharedData)mData).chassis.followPath(trajectory4);
                        },
                        new Condition(() -> ((SharedData)mData).chassis.hasFinished())
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
                        () -> {
                            ((SharedData)mData).chassis.followPath(trajectory5);
                        },
                        new Condition(() -> ((SharedData)mData).chassis.hasFinished())
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
                            ((SharedData)mData).chassis.followPath(trajectory6);
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
