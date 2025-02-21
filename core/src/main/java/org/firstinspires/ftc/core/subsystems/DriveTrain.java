/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Mechanum Drive management
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.subsystems;

/* System includes */
import java.util.List;

/* ACME robotics includes */
import com.acmerobotics.roadrunner.Pose2d;

/* Component includes */
import org.firstinspires.ftc.core.components.motors.MotorComponent;

public abstract class DriveTrain extends Subsystem {

    enum Mode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }

    protected static final String       sPidfKey                    = "pidf";
    protected static final String       sPhysicsKey                 = "physics";
    protected static final String       sMotorsKey                  = "motors";
    protected static final String       sOdometerKey                = "odometer";

    /* ---------------------- Drive functions ---------------------- */

    public abstract void                    initialize(Pose2d pose);
    public abstract void                    driveSpeedMultiplier(double multiplier);
    public abstract void                    drive(double xSpeed, double ySpeed, double headingSpeed);

    /* --------------------- Tuning functions ---------------------- */

    public abstract List<MotorComponent>    left();
    public abstract List<MotorComponent>    right();

}