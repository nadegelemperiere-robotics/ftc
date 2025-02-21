/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Mock Localization
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.odometers;

/* JSON includes */
import org.json.JSONObject;

/* ACME includes */
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

import java.util.ArrayList;
import java.util.List;

public class MockOdometer implements OdometerComponent {

    final LogManager            mLogger;

    final boolean               mConfigurationValid;

    final String                mName;

    Pose2d                      mInitialPose;
    Pose2d                      mCurrentPose;
    PoseVelocity2d              mCurrentVelocity;

    public MockOdometer(String name, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = true;

        mName               = name;

        mCurrentPose     = new Pose2d(new Vector2d(0,0),0);
        mInitialPose     = new Pose2d(new Vector2d(0,0),0);
        mCurrentVelocity = new PoseVelocity2d(new Vector2d(0,0),0);

    }

    @Override
    public void         pose(Pose2d current) {
        if(mConfigurationValid) {
            mInitialPose = current;
            mCurrentPose = current;
        }
    }

    @Override
    public void         update() {
        if (mConfigurationValid) {

        }
    }

    @Override
    public Pose2d           pose() {
        return mCurrentPose;
    }

    @Override
    public PoseVelocity2d   velocity() { return mCurrentVelocity; }

    @Override
    public void             log() {
        if (mConfigurationValid) {
            mLogger.metric("x", mCurrentPose.position.x + " inches");
            mLogger.metric("y", mCurrentPose.position.y + " inches");
            mLogger.metric("heading", mCurrentPose.heading.toDouble() + " rad");

            mLogger.metric("vx",mCurrentVelocity.linearVel.x + " inches/s");
            mLogger.metric("vy",mCurrentVelocity.linearVel.y + " inches/s");
            mLogger.metric("vheading",mCurrentVelocity.angVel + " rad/s");
        }
    }

    /* ------------------ Configurable functions ------------------- */

    /**
     * Determines if the driver odometer component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    /**
     * Reads and applies the drive odometer configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    @Override
    public void                         read(JSONObject reader) { }

    /**
     * Writes the current drive odometer configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) { }

    /**
     * Generates an HTML representation of the drive odometer configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted drive odometer configuration.
     */
    @Override
    public String                       logConfigurationHTML() { return "<p>Mock</p>\n"; }

    /**
     * Generates a text-based representation of the drive odometer configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted drive odometer configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {  return header + "> Mock\n"; }


    /**
     * List of encoders measuring forward displacement for tuning
     *
     * @return A list of encoders measuring forward displacement
     */
    public List<Encoder> forward(){
        List<org.firstinspires.ftc.core.components.odometers.Encoder> result = new ArrayList<>();
        return result;
    }

    /**
     * List of encoders measuring lateral displacement for tuning
     *
     * @return A list of encoders measuring lateral displacement
     */
    public List<Encoder>     lateral(){
        List<Encoder> result = new ArrayList<>();
        return result;
    }


}
