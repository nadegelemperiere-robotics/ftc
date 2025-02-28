/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Mechanum Drive management
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.subsystems;

/* Pedro Pathing includes */
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Path;

public interface DriveTrain extends Subsystem {

    public enum Mode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }

    /* ---------------------- Drive functions ---------------------- */

    void        initialize(Pose pose);
    void        start();
    void        driveSpeedMultiplier(double multiplier);
    void        drive(double xSpeed, double ySpeed, double headingSpeed);

    /* -------------------- Autonomous functions ------------------- */

    PathBuilder pathBuilder();

    void        turn(double radians, boolean isLeft);
    void        turnTo(double radians);

    void        followPath(Path path, boolean hold);
    void        followPath(Path path);
    void        followPath(PathChain chain, boolean holdd);
    void        followPath(PathChain chain);
    void        followPath(PathChain chain, double maxPower, boolean hold) ;
}
