/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localization interface providing robot precise position
   and orientation on the mat
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.localizers;

/* System includes */
import java.util.Map;
import java.util.List;

/* JSON includes */
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* PedroPathing includes */
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

/* Components includes */
import org.firstinspires.ftc.core.components.motors.MotorComponent;
import org.firstinspires.ftc.core.components.imus.ImuComponent;

public abstract class LocalizerComponent extends Localizer implements Configurable {

    static public LocalizerComponent factory(String name, JSONObject reader, HardwareMap map, Map<String, MotorComponent> motors, Map<String,ImuComponent> imus, LogManager logger) {

        LocalizerComponent result = null;

        switch (name) {
            case LocalizerOTOS.sTypeKey:
                result = new LocalizerOTOS(name, map, logger);
                result.read(reader);
                break;
            case LocalizerTwoDeadWheels.sTypeKey:
                result = new LocalizerTwoDeadWheels(name, map, motors, imus, logger);
                result.read(reader);
                break;
            case LocalizerThreeDeadWheels.sTypeKey:
                result = new LocalizerThreeDeadWheels(name, map, motors, logger);
                result.read(reader);
                break;
            case LocalizerThreeDeadWheelsImu.sTypeKey:
                result = new LocalizerThreeDeadWheelsImu(name, map, motors, imus, logger);
                result.read(reader);
                break;
            case LocalizerMecanumDriveEncoder.sTypeKey:
                result = new LocalizerMecanumDriveEncoder(name, map, motors, imus, logger);
                result.read(reader);
                break;
            case LocalizerPinPoint.sTypeKey:
                result = new LocalizerPinPoint(name, map, logger);
                result.read(reader);
                break;
            case LocalizerMock.sTypeKey:
                result = new LocalizerMock(name, logger);
                result.read(reader);
                break;
        }

        return result;

    }

    /* ------------------ Custom functions ------------------------- */
    public abstract void    log();
    public abstract String  name();

    /* ------------------ Localizer functions ---------------------- */
    public abstract Pose    getPose();
    public abstract Pose    getVelocity();
    public abstract Vector  getVelocityVector();
    public abstract void    setStartPose(Pose setStart);
    public abstract void    setPose(Pose setPose);
    public abstract void    update();
    public abstract double  getTotalHeading();
    public abstract double  getForwardMultiplier();
    public abstract double  getLateralMultiplier();
    public abstract double  getTurningMultiplier();
    public abstract void    setForwardMultiplier(double value);
    public abstract void    setLateralMultiplier(double value);
    public abstract void    setTurningMultiplier(double value);
    public abstract void    resetIMU() throws InterruptedException;
    public abstract boolean isNAN();

    /* ------------------ Configurable functions ------------------- */

    public abstract void    read(JSONObject reader);
    public abstract void    write(JSONObject writer);
    public abstract boolean isConfigured();
    public abstract String  logConfigurationHTML();
    public abstract String  logConfigurationText(String header);



}