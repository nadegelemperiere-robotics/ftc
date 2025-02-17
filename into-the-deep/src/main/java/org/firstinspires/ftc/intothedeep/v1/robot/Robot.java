/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Season robot management
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.robot;

/* System includes */
import java.util.Iterator;
import java.util.Map;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.core.tools.LogManager;

/* Subsystem includes */
import org.firstinspires.ftc.core.subsystems.MecanumDrive;
import org.firstinspires.ftc.core.subsystems.DefaultSlides;
import org.firstinspires.ftc.intothedeep.v1.subsystems.IntakeArm;
import org.firstinspires.ftc.intothedeep.v1.subsystems.OuttakeArm;
import org.firstinspires.ftc.intothedeep.v1.subsystems.Subsystem;


public class Robot extends org.firstinspires.ftc.core.robot.Robot {

    static final String sIntakeArmKey       = "intake-arm";
    static final String sIntakeSlidesKey    = "intake-slides";
    static final String sOuttakeArmKey      = "outtake-arm";
    static final String sOuttakeSlidesKey   = "outtake-slides";
    static final String sChassisKey         = "drive-train";

    RobotState.SharedData  mData;

    /**
     * Constructs a season robot instance.
     *
     * @param map    The FTC HardwareMap to retrieve hardware.
     * @param logger The logging manager for error reporting and debugging.
     */
    public  Robot(HardwareMap map, LogManager logger) {
        super(map, logger);
        mData  = new RobotState.SharedData();
    }

    /* ---------------------- TeleOp commands ---------------------- */
    public void drive(double x, double y, double heading)   { ((RobotState)mState).drive(x,y,heading); }
    public void tuneDriveSpeed(double multiplier)           { ((RobotState)mState).tuneDriveSpeed(multiplier); }

    public void powerOuttakeSlides(double power)            { ((RobotState)mState).powerOuttakeSlides(power); }
    public void positionOuttakeSlides(String position)      { ((RobotState)mState).positionOuttakeSlides(position); }
    public void moveOuttakeArm(String direction)            { ((RobotState)mState).moveOuttakeArm(direction); }
    public void toggleOuttakeClaw()                         { ((RobotState)mState).toggleOuttakeClaw(); }

    public void powerIntakeSlides(double power)             { ((RobotState)mState).powerIntakeSlides(power); }
    public void moveIntakeArm(String direction)             { ((RobotState)mState).moveIntakeArm(direction); }
    public void toggleIntakeClaw()                          { ((RobotState)mState).toggleIntakeClaw(); }
    public void toggleIntakeWrist()                         { ((RobotState)mState).toggleIntakeWrist(); }

    /* --------------------- States management --------------------- */
    public void transfer()                                  { this.mState = ((RobotState)mState).toTransfer(); }
    public void pick()                                      { this.mState = ((RobotState)mState).toPick(); }

    /**
     * Starts the robot in initial position
     */
    public void                         start() {
        mState = new InitState((RobotState.SharedData)mData, mLogger);
    }

    /* ------------------ Configurable functions ------------------- */
    @Override
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;
        mSubsystems.clear();

        RobotState.SharedData data = (RobotState.SharedData)mData;

        data.chassis       = null;
        data.intakeArm     = null;
        data.outtakeArm    = null;
        data.intakeSlides  = null;
        data.outtakeSlides = null;

        try {
            if(reader.has(sHardwareKey)) {
                JSONObject hardware = reader.getJSONObject(sHardwareKey);
                mHardware.read(hardware);
                if(!mHardware.isConfigured()) {
                    mLogger.warning("Hardware configuration is invalid");
                    mConfigurationValid = false;
                }
            }

            if(reader.has(sSubsystemsKey)) {
                JSONObject subsystems = reader.getJSONObject(sSubsystemsKey);

                Iterator<String> keys = subsystems.keys();
                while (keys.hasNext()) {

                    String key = keys.next();

                    org.firstinspires.ftc.core.subsystems.Subsystem subsystem = Subsystem.factory(key, subsystems.getJSONObject(key), mHardware, mLogger);
                    if(!subsystem.isConfigured()) {
                        mLogger.warning("Subsystem " + key + " configuration is invalid");
                        mConfigurationValid = false;
                    }
                    else {
                        mSubsystems.put(key, subsystem);
                    }

                }
            }

        } catch (JSONException e) {
            mLogger.error(e.getMessage());
        }

        for (Map.Entry<String, org.firstinspires.ftc.core.subsystems.Subsystem> subsystem : mSubsystems.entrySet()) {
            if(subsystem.getKey().equals(sChassisKey)) {
                org.firstinspires.ftc.core.subsystems.Subsystem chassis = subsystem.getValue();
                if (chassis instanceof MecanumDrive) {
                    data.chassis = (MecanumDrive) chassis;
                }
            }
            if(subsystem.getKey().equals(sIntakeArmKey)) {
                org.firstinspires.ftc.core.subsystems.Subsystem arm = subsystem.getValue();
                if (arm instanceof IntakeArm) {
                    data.intakeArm = (IntakeArm) arm;
                }
            }
            if(subsystem.getKey().equals(sIntakeSlidesKey)) {
                org.firstinspires.ftc.core.subsystems.Subsystem slides = subsystem.getValue();
                if (slides instanceof DefaultSlides) {
                    data.intakeSlides = (DefaultSlides) slides;
                }
            }
            if(subsystem.getKey().equals(sOuttakeSlidesKey)) {
                org.firstinspires.ftc.core.subsystems.Subsystem slides = subsystem.getValue();
                if (slides instanceof DefaultSlides) {
                    data.outtakeSlides = (DefaultSlides) slides;
                }
            }
            if(subsystem.getKey().equals(sOuttakeArmKey)) {
                org.firstinspires.ftc.core.subsystems.Subsystem arm = subsystem.getValue();
                if (arm instanceof OuttakeArm) {
                    data.outtakeArm = (OuttakeArm) arm;
                }
            }
        }

        if(data.chassis == null) {
            mLogger.error("Chassis not found in subsystems");
            mConfigurationValid = false;
        }
        if(data.intakeArm == null) {
            mLogger.error("Intake arm not found in subsystems");
            mConfigurationValid = false;
        }
        if(data.intakeSlides == null) {
            mLogger.error("Intake slides not found in subsystems");
            mConfigurationValid = false;
        }
        if(data.outtakeArm == null) {
            mLogger.error("Outtake arm not found in subsystems");
            mConfigurationValid = false;
        }
        if(data.outtakeSlides == null) {
            mLogger.error("Outtake slides not found in subsystems");
            mConfigurationValid = false;
        }
    }


}
