/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Mechanum Drive management
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.subsystems;

/* ACME robotics includes */
import com.acmerobotics.roadrunner.Pose2d;

public abstract class DriveTrain extends Subsystem {

    enum Mode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }

    protected static final String       sPidfKey                    = "pidf";
    protected static final String       sPhysicsKey                 = "physics";
    protected static final String       sMotorsKey                  = "motors";
    protected static final String       sOdometerKey                = "odometer";


    public void                         initialize(Pose2d pose) {}

    public void                         driveSpeedMultiplier(double multiplier) { }

    public void                         drive(double xSpeed, double ySpeed, double headingSpeed) {}

}