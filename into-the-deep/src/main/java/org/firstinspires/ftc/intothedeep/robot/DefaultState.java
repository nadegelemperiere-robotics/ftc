/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Season robot configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.robot;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Robot includes */
import org.firstinspires.ftc.core.robot.RobotState;
import org.firstinspires.ftc.intothedeep.robot.TransferState;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.sequencer.State;

public class DefaultState extends SeasonRobotState {

    public  DefaultState(SeasonSharedData data, LogManager logger) {
        super(data,logger);
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
    public  void    toggleOuttakeClaw() { ((SeasonSharedData)mData).outtakeArm.toogleClaw();}


    @Override
    public  void    powerIntakeSlides(double power) { ((SeasonSharedData)mData).intakeSlides.power(power);}

    @Override
    public  void    moveIntakeArm(String direction)  {}

    @Override
    public  void    toggleIntakeClaw() { ((SeasonSharedData)mData).intakeArm.toogleClaw();}

    @Override
    public  void    toggleIntakeWrist() { ((SeasonSharedData)mData).intakeArm.rotate(); }


    @Override
    public  RobotState  toTransfer() {
        return new TransferState(((SeasonSharedData)mData),mLogger);
    }

    @Override
    public  RobotState  toPick() {
        return new PickState(((SeasonSharedData)mData),mLogger);
    }

    @Override
    public  State       next() {
        return this;
    }

    @Override
    public  boolean     hasFinished() {
        return true;
    }

}
