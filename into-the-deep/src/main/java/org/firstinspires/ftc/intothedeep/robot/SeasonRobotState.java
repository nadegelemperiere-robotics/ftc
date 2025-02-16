/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Season robot configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.robot;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Subsystem includes */
import org.firstinspires.ftc.core.subsystems.DefaultSlides;
import org.firstinspires.ftc.core.subsystems.MecanumDrive;
import org.firstinspires.ftc.intothedeep.subsystems.IntakeArm;
import org.firstinspires.ftc.intothedeep.subsystems.OuttakeArm;

/* Robot includes */
import org.firstinspires.ftc.core.robot.RobotState;


public abstract class SeasonRobotState extends RobotState {

    public static class SeasonSharedData extends RobotState.SharedData {
        public IntakeArm       intakeArm;
        public DefaultSlides   intakeSlides;
        public OuttakeArm      outtakeArm;
        public DefaultSlides   outtakeSlides;
        public MecanumDrive    chassis;
    };

    SharedData      mData;

    public              SeasonRobotState(SeasonSharedData data, LogManager logger) {
        super(data,logger);
    }

    public  void        drive(double x, double y, double heading) {  }
    public  void        tuneDriveSpeed(double multiplier) {}

    public  void        powerOuttakeSlides(double power) {}
    public  void        positionOuttakeSlides(String position) {}
    public  void        moveOuttakeArm(String direction)  {}
    public  void        toggleOuttakeClaw() {}

    public  void        powerIntakeSlides(double power) {}
    public  void        moveIntakeArm(String direction)  {}
    public  void        toggleIntakeClaw() {}
    public  void        toggleIntakeWrist() {}

    public  RobotState  toTransfer() { return null; }
    public  RobotState  toPick()     { return null; }
}
