/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Season robot configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.robot;

/* System includes */
import java.util.Iterator;
import java.util.Map;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Tools includes */
import org.firstinspires.ftc.core.subsystems.Subsystem;
import org.firstinspires.ftc.core.tools.LogManager;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Robot;

/* Subsystem includes */
import org.firstinspires.ftc.core.subsystems.MecanumDrive;
import org.firstinspires.ftc.core.subsystems.DefaultSlides;
import org.firstinspires.ftc.intothedeep.subsystems.IntakeArm;
import org.firstinspires.ftc.intothedeep.subsystems.OuttakeArm;
import org.firstinspires.ftc.intothedeep.subsystems.SeasonSubsystem;


public class SeasonRobot extends Robot {

    static final String sIntakeArmKey       = "intake-arm";
    static final String sIntakeSlidesKey    = "intake-slides";
    static final String sOuttakeArmKey      = "outtake-arm";
    static final String sOuttakeSlidesKey   = "outtake-slides";
    static final String sChassisKey         = "drive-train";

    SeasonRobotState.SharedData  mData;

    public  SeasonRobot(HardwareMap map, LogManager logger) {
        super(map, logger);
        mData  = new SeasonRobotState.SeasonSharedData();
        mState = new DefaultState((SeasonRobotState.SeasonSharedData)mData, mLogger);
    }

    public void drive(double x, double y, double heading)   { ((SeasonRobotState)mState).drive(x,y,heading); }
    public void tuneDriveSpeed(double multiplier)           { ((SeasonRobotState)mState).tuneDriveSpeed(multiplier); }


    public void powerOuttakeSlides(double power)            { ((SeasonRobotState)mState).powerOuttakeSlides(power); }
    public void positionOuttakeSlides(String position)      { ((SeasonRobotState)mState).positionOuttakeSlides(position); }
    public void moveOuttakeArm(String direction)            { ((SeasonRobotState)mState).moveOuttakeArm(direction); }
    public void toggleOuttakeClaw()                         { ((SeasonRobotState)mState).toggleOuttakeClaw(); }


    public void powerIntakeSlides(double power)             { ((SeasonRobotState)mState).powerIntakeSlides(power); }
    public void moveIntakeArm(String direction)             { ((SeasonRobotState)mState).moveIntakeArm(direction); }
    public void toggleIntakeClaw()                          { ((SeasonRobotState)mState).toggleIntakeClaw(); }
    public void toggleIntakeWrist()                         { ((SeasonRobotState)mState).toggleIntakeWrist(); }


    public void transfer()                                  { this.mState = ((SeasonRobotState)mState).toTransfer(); }
    public void pick()                                      { this.mState = ((SeasonRobotState)mState).toPick(); }

    @Override
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;
        mSubsystems.clear();

        SeasonRobotState.SeasonSharedData data = (SeasonRobotState.SeasonSharedData)mData;

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

                    Subsystem subsystem = SeasonSubsystem.factory(key, subsystems.getJSONObject(key), mHardware, mLogger);
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

        for (Map.Entry<String, Subsystem> subsystem : mSubsystems.entrySet()) {
            if(subsystem.getKey().equals(sChassisKey)) {
                Subsystem chassis = subsystem.getValue();
                if (chassis instanceof MecanumDrive) {
                    data.chassis = (MecanumDrive) chassis;
                }
            }
            if(subsystem.getKey().equals(sIntakeArmKey)) {
                Subsystem arm = subsystem.getValue();
                if (arm instanceof IntakeArm) {
                    data.intakeArm = (IntakeArm) arm;
                }
            }
            if(subsystem.getKey().equals(sIntakeSlidesKey)) {
                Subsystem slides = subsystem.getValue();
                if (slides instanceof DefaultSlides) {
                    data.intakeSlides = (DefaultSlides) slides;
                }
            }
            if(subsystem.getKey().equals(sOuttakeSlidesKey)) {
                Subsystem slides = subsystem.getValue();
                if (slides instanceof DefaultSlides) {
                    data.outtakeSlides = (DefaultSlides) slides;
                }
            }
            if(subsystem.getKey().equals(sOuttakeArmKey)) {
                Subsystem arm = subsystem.getValue();
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
