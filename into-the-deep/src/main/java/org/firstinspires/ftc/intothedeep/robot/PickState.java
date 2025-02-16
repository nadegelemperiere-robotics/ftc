/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Season robot configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.robot;

/* Tools includes */
import org.firstinspires.ftc.core.orchestration.sequencer.Sequencer;
import org.firstinspires.ftc.core.orchestration.sequencer.State;
import org.firstinspires.ftc.core.tools.LogManager;


/* Robot includes */
import org.firstinspires.ftc.core.robot.RobotState;
import org.firstinspires.ftc.intothedeep.robot.DefaultState;

public class PickState extends SeasonRobotState {

    Sequencer mSequencer;

    public  PickState(SeasonSharedData data, LogManager logger) {
        super(data,logger);
        mSequencer = new Sequencer(mLogger);
        mSequencer.sequence("PICK");
    }

    @Override
    public  void    drive(double x, double y, double heading) { ((SeasonSharedData)mData).chassis.drive(x,y,heading); }

    @Override
    public  void    tuneDriveSpeed(double multiplier) { ((SeasonSharedData)mData).chassis.driveSpeedMultiplier(multiplier); }


    @Override
    public  void    powerOuttakeSlides(double power) { ((SeasonSharedData)mData).outtakeSlides.power(power);}

    @Override
    public  void    positionOuttakeSlides(String position) {}

    @Override
    public  void    moveOuttakeArm(String direction)  {}

    @Override
    public  void    toggleOuttakeClaw() { ((SeasonSharedData)mData).outtakeArm.toogleClaw(); }

    @Override
    public  RobotState  toTransfer() {

        mLogger.info("Transfer asked while picking is on-going : not taken into account");
        return this;
    }

    @Override
    public  RobotState  toPick() {

        mLogger.info("Pick asked while picking is on-going : not taken into account");
        return this;
    }


    @Override
    public State        next() {
        return new DefaultState(((SeasonSharedData)mData), mLogger);
    }

    @Override
    public boolean      hasFinished() {
        return mSequencer.hasFinished();
    }

}
