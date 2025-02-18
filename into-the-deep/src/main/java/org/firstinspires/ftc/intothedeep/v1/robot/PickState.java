/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Season robot configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.robot;

/* Tools includes */
import org.firstinspires.ftc.core.orchestration.engine.Task;
import org.firstinspires.ftc.core.tools.Condition;
import org.firstinspires.ftc.core.tools.LogManager;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.Sequencer;
import org.firstinspires.ftc.core.orchestration.engine.State;

public class PickState extends RobotState {

    final Sequencer mSequencer;

    /**
     * Constructs a pick state with the associated sample picking sequence
     *
     * @param data   The data shared among all states
     * @param logger The logging manager for error reporting and debugging.
     */
    public  PickState(SharedData data, LogManager logger) {
        super(data,logger);
        mSequencer = new Sequencer(mLogger);
        mSequencer.sequence(
                "PICK",
                new Task(
                        "Stop all motors",
                        () -> {
                            ((SharedData)mData).intakeSlides.power(0);
                            ((SharedData)mData).outtakeSlides.power(0);
                            ((SharedData)mData).chassis.drive(0,0,0);
                        },
                        new Condition(()-> true)
                ));
    }

    /* ---------------------- TeleOp commands ---------------------- */
    // Driving and outtake manipulation are allowed in this state
    @Override
    public  void    drive(double x, double y, double heading) { ((SharedData)mData).chassis.drive(x,y,heading); }

    @Override
    public  void    tuneDriveSpeed(double multiplier) { ((SharedData)mData).chassis.driveSpeedMultiplier(multiplier); }


    @Override
    public  void    powerOuttakeSlides(double power) { ((SharedData)mData).outtakeSlides.power(power);}

    @Override
    public  void    positionOuttakeSlides(String position) { ((SharedData)mData).outtakeSlides.position(position,5,5000);}

    @Override
    public  void    moveOuttakeArm(String direction)  { ((SharedData)mData).outtakeArm.move(direction);}

    @Override
    public  void    toggleOuttakeClaw() { ((SharedData)mData).outtakeArm.toogleClaw(); }

    /* --------------------- States management --------------------- */

    /**
     * Declare the state finished once pick sequence is over
     */
    @Override
    public boolean      hasFinished() { return mSequencer.hasFinished(); }

    /**
     * Update sequencer periodically to switch between tasks
     */
    @Override
    public void         update() { mSequencer.update(); }

    /**
     * Tramsfer is not allowed until picking is over
     *
     * @return this - meaning the state does not change
     */
    @Override
    public  RobotState  toTransfer() {

        mLogger.info("Transfer asked while picking is on-going : not taken into account");
        return this;
    }

    /**
     * Pick restart is not allowed until current is over
     *
     * @return this - meaning the state does not change
     */
    @Override
    public  RobotState  toPick() {

        mLogger.info("Pick asked while picking is on-going : not taken into account");
        return this;
    }

    /**
     * After picking is over, switch to default state
     *
     * @return a default state to restore all commands
     */
    @Override
    public State        next() {
        mLogger.metric("STATE","PICK -> DEFAULT");
        return new DefaultState(((SharedData)mData), mLogger);
    }


}
