/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Into-The-Deep Autonomous Sample Opmode
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/* ACME robotics includes */
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.FtcDashboard;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.intothedeep.v1.configuration.Configuration;

/* Robot includes */
import org.firstinspires.ftc.intothedeep.v1.robot.Robot;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.InterOpMode;

@Autonomous(name = "Robot V1 Sample Autonomous", group = "V1", preselectTeleOp = "Robot V1 Teleop")
public class AutonomousSampleOpMode extends LinearOpMode {

    LogManager      mLogger;

    Configuration   mConfiguration;

    Robot           mRobot;

    @Override
    public void runOpMode() {

        try {
            // Log initialization
            mLogger = new LogManager(telemetry, FtcDashboard.getInstance(),"auto-sample");
            mLogger.clear();

            // Configuration initialization
            mConfiguration = Configuration.getInstance();
            mConfiguration.logger(mLogger);

            // Robot initialization
            mRobot = new Robot(hardwareMap, mLogger);
            InterOpMode.instance().clear();

            // Register configurables
            mConfiguration.register("logging",mLogger);
            mConfiguration.register("robot",mRobot);
            mConfiguration.read();
            mConfiguration.log();

            mLogger.update();

        } catch (Exception e) {
            mLogger.error(e.getMessage());
        }

        waitForStart();

        try {

            // Initialize robot position with position in FIELD CENTRIC reference,
            // Meaning X is oriented towards the opponent alliance station, Y oriented to the left
            // and Z to the top. Center is the robot starting point
            mRobot.start(Robot.Mode.AUTO_SPECIMEN, new Pose2d(new Vector2d(0,0),-Math.PI / 2));

            while(opModeIsActive() && !mRobot.state().equals("EndState")) {
                mRobot.update();
                mLogger.update();
            }

            mRobot.persist();
            InterOpMode.instance().log(mLogger);
            mLogger.stop();

        } catch (Exception e) {
            mLogger.error(e.getMessage());
        }
    }
}
