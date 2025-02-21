/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Season robot configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.robot;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Robot includes */
import org.firstinspires.ftc.intothedeep.v1.robot.TransferState;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.State;

public class DefaultState extends RobotState {

    public  DefaultState(SharedData data, LogManager logger) {
        super(data,logger);
    }

    @Override
    public  void    drive(double x, double y, double heading) { ((SharedData)mData).chassis.drive(x,y,heading); }

    @Override
    public  void    tuneDriveSpeed(double multiplier) { ((SharedData)mData).chassis.driveSpeedMultiplier(multiplier); }


    @Override
    public  void    powerOuttakeSlides(double power) { ((SharedData)mData).outtakeSlides.power(power);}

    @Override
    public  void    positionOuttakeSlides(String position) {((SharedData)mData).outtakeSlides.position(position,5,5000);}

    @Override
    public  void    moveOuttakeArm(String direction)  { ((SharedData)mData).outtakeArm.move(direction);}

    @Override
    public  void    toggleOuttakeClaw() { ((SharedData)mData).outtakeArm.toogleClaw();}


    @Override
    public  void    powerIntakeSlides(double power) { ((SharedData)mData).intakeSlides.power(power);}

    @Override
    public  void    moveIntakeArm(String direction)  { ((SharedData)mData).intakeArm.move(direction);}

    @Override
    public  void    toggleIntakeClaw() { ((SharedData)mData).intakeArm.toogleClaw();}

    @Override
    public  void    toggleIntakeWrist() { ((SharedData)mData).intakeArm.rotate(); }

    /* --------------------- States management --------------------- */

    /**
     * Update sequencer periodically to switch between tasks
     */
    @Override
    public void         update() {
        mLogger.info("STATE : Default");
    }

    @Override
    public  RobotState  toTransfer() {
        mLogger.info("TRANSITION : Default -> Transfer");
        return new TransferState(((SharedData)mData),mLogger);
    }

    @Override
    public  RobotState  toPick() {
        mLogger.info("TRANSITION : Default -> Pick");
        return new PickState(((SharedData)mData),mLogger);
    }

    @Override
    public  State       next() {
        return this;
    }


}
