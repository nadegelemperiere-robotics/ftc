/**
 * -------------------------------------------------------
 * Copyright (c) 2025 Nadege LEMPERIERE
 * All rights reserved
 * -------------------------------------------------------
 * MotorCoupled class extends the FTC motor functionality
 * to manage a pair of motors that control the same hardware.
 * <p>
 * WARNING: This configuration can be dangerous and may
 * result in motor damage if not properly tuned. The coupled
 * motors must be identical models.
 * -------------------------------------------------------
 * <p>
 * This class ensures synchronized control of two motors,
 * allowing them to operate as a single unit. It provides
 * features for reading and writing configurations, setting
 * encoder corrections, and logging motor statuses.
 * <p>
 * Features:
 * - Couples two motors to function as one.
 * - Manages motor configurations using JSON input.
 * - Provides encoder correction for reversed directions.
 * - Supports logging of motor positions, power, and velocity.
 * - Implements standard FTC DcMotor and DcMotorEx behaviors.
 * <p>
 * Dependencies:
 * - Qualcomm Robotics SDK
 * - FTC SDK
 * - JSON Processing (org.json)
 * - Custom LogManager for logging
 * <p>
 * Usage:
 * 1. Create an instance of MotorCoupled with the robot's
 *    hardware map and logger.
 * 2. Configure the motors by reading a JSON configuration.
 * 3. Control the motors using standard FTC motor functions.
 * <p>
 * Example:
 * {@code
 *      JSONObject config = new JSONObject();
 *      JSONObject firstMotor = new JSONObject();
 *      JSONObject secondMotor = new JSONObject();
 *      firstMotor.put("hwmap", "left_motor");
 *      secondMotor.put("hwmap", "right_motor");
 *      config.put("first", firstMotor);
 *      config.put("second", secondMotor);
 * <p>
 *      MotorCoupled coupledMotor = new MotorCoupled("drive", hardwareMap, logger);
 *      coupledMotor.read(config);
 *      coupledMotor.setPower(0.5);
 * }
 */

package org.firstinspires.ftc.core.components.motors;

/* JSON includes */
import org.json.JSONObject;
import org.json.JSONException;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class MotorCoupled implements MotorComponent {

    public static final String  sFirstKey  = "first";
    public static final String  sSecondKey = "second";

    final LogManager            mLogger;

    final String                mName;
    String                      mFirstHwName;
    String                      mSecondHwName;

    boolean                     mConfigurationValid;
    
    DcMotorSimple.Direction     mDirection;

    final HardwareMap           mMap;
    DcMotorEx                   mFirst;
    DcMotorEx                   mSecond;
    int                         mFirstInvertPosition;
    int                         mSecondInvertPosition;


    /* ----------------------- Constructors ------------------------ */
    /**
     * Constructs a MotorCoupled instance.
     *
     * @param name   The name of the coupled motor component.
     * @param hwMap  The FTC HardwareMap to retrieve motor hardware.
     * @param logger The logging manager for error reporting and debugging.
     */
    public MotorCoupled(String name, HardwareMap hwMap, LogManager logger)
    {
        mLogger                 = logger;
        mName                   = name;
        mFirstHwName            = "";
        mSecondHwName           = "";
        mConfigurationValid     = false;

        mMap                    = hwMap;

        mDirection              = DcMotor.Direction.FORWARD;
        mFirst                  = null;
        mSecond                 = null;
        mFirstInvertPosition    = 1;
        mSecondInvertPosition   = 1;

    }

    /* --------------------- Custom functions ---------------------- */

    /**
            * Retrieves the name of the coupled motor component.
            *
            * @return The name of the component.
     */
    @Override
    public String                       name() { return mName; }

    /**
     * Determines if encoder correction is required.
     *
     * @return True if at least one motor has inverted encoder behavior, false otherwise.
     */
    @Override
    public boolean                      encoderCorrection() { return ((mFirstInvertPosition == -1) || (mSecondInvertPosition == -1)); }

    /**
     * Enables or disables encoder correction.
     *
     * @param shallCorrect True to enable encoder correction, false to disable.
     */
    @Override
    public void                         encoderCorrection(boolean shallCorrect) {
        if (mConfigurationValid) {
            if (shallCorrect) {
                mFirstInvertPosition = -1;
                mSecondInvertPosition = -1;
            } else {
                mFirstInvertPosition = 1;
                mSecondInvertPosition = 1;
            }
        }
    }

    /**
     * Logs the current motor positions, velocities, and power levels.
     *
     * @return A formatted string containing motor telemetry data.
     */
    @Override
    public String                       log() {
        String result = "";
        if (mConfigurationValid) {
            result += "\n  First : P : " + mFirst.getCurrentPosition() + " V : " + mFirst.getVelocity() + " P : " + mFirst.getPower();
            result += "\n  Second : P : " + mSecond.getCurrentPosition() + " V : " + mSecond.getVelocity() + " P : " + mSecond.getPower();
        }
        return result;
    }

    /* ------------------ Configurable functions ------------------- */

    /**
     * Determines if the coupled motor component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    /**
     * Reads and applies the motor configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    @Override
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;
        mFirst = null;
        mSecond = null;

        if(!reader.has(sFirstKey))       { mLogger.error("Missing first DC motor for coupled motor"); }
        else if(!reader.has(sSecondKey)) { mLogger.error("Missing second DC motor for coupled motor"); }
        else {
            try {
                JSONObject first = reader.getJSONObject(sFirstKey);
                JSONObject second = reader.getJSONObject(sSecondKey);

                if(mMap != null && first.has(sHwMapKey)) {
                    mFirstHwName = first.getString(sHwMapKey);
                    mFirst = mMap.tryGet(DcMotorEx.class, mFirstHwName);
                }
                if(mMap != null && second.has(sHwMapKey)) {
                    mSecondHwName = second.getString(sHwMapKey);
                    mSecond = mMap.tryGet(DcMotorEx.class, mSecondHwName);
                }

                if(mFirst != null && first.has(sDirectionKey)) {
                    DcMotor.Direction direction = sString2Direction.get(first.getString(sDirectionKey));
                    mFirst.setDirection(direction);
                }
                else if(mFirst != null) {
                    mFirst.setDirection(DcMotor.Direction.FORWARD);
                }
                if(mSecond != null && second.has(sDirectionKey)) {
                    DcMotor.Direction direction = sString2Direction.get(second.getString(sDirectionKey));
                    mSecond.setDirection(direction);
                }
                else if(mSecond != null) {
                    mSecond.setDirection(DcMotor.Direction.FORWARD);
                }

                if(mFirst != null && first.has(sEncoderReverseKey)) {
                    boolean shallReverse = first.getBoolean(sEncoderReverseKey);
                    if(shallReverse) { mFirstInvertPosition = -1; }
                    else { mFirstInvertPosition = 1; }
                }
                else { mFirstInvertPosition = 1; }
                if(mSecond != null && second.has(sEncoderReverseKey)) {
                    boolean shallReverse = second.getBoolean(sEncoderReverseKey);
                    if(shallReverse) { mSecondInvertPosition = -1; }
                    else { mSecondInvertPosition = 1; }
                }
                else { mSecondInvertPosition = 1; }

                if(mFirst != null) { mFirst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }
                if(mSecond != null) { mSecond.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }

            }
            catch(JSONException e) { mLogger.error(e.getMessage()); }
        }

        if (mFirst == null) { mConfigurationValid = false; }
        if (mSecond == null) { mConfigurationValid = false; }

    }

    /**
     * Writes the current motor configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) {

        if(mConfigurationValid) {

            JSONObject first = new JSONObject();
            JSONObject second = new JSONObject();

            try {
                if (mFirst != null) {
                    String direction = sDirection2String.get(mFirst.getDirection());
                    first.put(sHwMapKey, mFirstHwName);
                    first.put(sDirectionKey, direction);
                    first.put(sEncoderReverseKey, mFirstInvertPosition == -1);
                }
                if (mSecond != null) {
                    String direction = sDirection2String.get(mSecond.getDirection());
                    second.put(sHwMapKey, mSecondHwName);
                    second.put(sDirectionKey, direction);
                    second.put(sEncoderReverseKey, mSecondInvertPosition == -1);
                }

                writer.put(sFirstKey, first);
                writer.put(sSecondKey, second);
            } catch (JSONException e) { mLogger.error(e.getMessage()); }
        }

    }

    /**
     * Generates an HTML representation of the motor configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted motor configuration.
     */
    @Override
    public String                       logConfigurationHTML() {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {

            if (mFirst != null) {
                result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                        .append("ID : ")
                        .append(sFirstKey)
                        .append(" - HW : ")
                        .append(mFirstHwName)
                        .append(" - DIR : ")
                        .append(sDirection2String.get(mFirst.getDirection()))
                        .append(" - ENC : ")
                        .append(mFirstInvertPosition == -1)
                        .append("</li>\n");
            }
            if (mSecond != null) {
                result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                        .append("ID : ")
                        .append(sSecondKey)
                        .append(" - HW : ")
                        .append(mSecondHwName)
                        .append(" - DIR : ")
                        .append(sDirection2String.get(mSecond.getDirection()))
                        .append(" - ENC : ")
                        .append(mSecondInvertPosition == -1)
                        .append("</li>\n");
            }
        }

        return result.toString();

    }

    /**
     * Generates a text-based representation of the motor configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted motor configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {
            if (mFirst != null) {
                result.append(header)
                        .append("> ")
                        .append(sFirstKey)
                        .append(" HW :")
                        .append(mFirstHwName)
                        .append(" - DIR : ")
                        .append(sDirection2String.get(mFirst.getDirection()))
                        .append(" - ENC : ")
                        .append(mFirstInvertPosition == -1)
                        .append("\n");
            }
            if (mSecond != null) {
                result.append(header)
                        .append("> ")
                        .append(sSecondKey)
                        .append(" HW : ")
                        .append(mSecondHwName)
                        .append(" - DIR : ")
                        .append(sDirection2String.get(mSecond.getDirection()))
                        .append(" - ENC : ")
                        .append(mSecondInvertPosition == -1)
                        .append("\n");
            }
        }

        return result.toString();

    }

    /* --------------------- DcMotor functions --------------------- */

    @Override
    public int	                        currentPosition()
    {
        int result = -1;
        if(mConfigurationValid) {
            result = (int) (0.5 * mFirstInvertPosition * mFirst.getCurrentPosition() +
                    mSecondInvertPosition * 0.5 * mSecond.getCurrentPosition());
        }
        return result;
    }

    @Override
    public DcMotorSimple.Direction      direction()
    {
        return mDirection;
    }


    @Override
    public DcMotor.RunMode	            mode()
    {
        DcMotor.RunMode result =  DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        if (mConfigurationValid) { result = mFirst.getMode(); }
        return result;
    }

    @Override
    public int	                        targetPosition()
    {
        int result = -1;
        if(mConfigurationValid) {
            result = (int) (0.5 * mFirstInvertPosition * mFirst.getTargetPosition() +
                    0.5 * mSecondInvertPosition * mSecond.getTargetPosition());
        }
        return result;
    }

    @Override
    public double	                    power()
    {
        double result = -1;
        if(mConfigurationValid) {
            result = (int) (0.5 * mFirst.getPower() + 0.5 * mSecond.getPower());
        }
        return result;
    }

    @Override
    public DcMotor.ZeroPowerBehavior	zeroPowerBehavior()
    {
        DcMotor.ZeroPowerBehavior result = DcMotor.ZeroPowerBehavior.UNKNOWN;
        if(mConfigurationValid) { result = mFirst.getZeroPowerBehavior(); }
        return result;
    }

    @Override
    public boolean	                    isBusy()
    {
        boolean result = false;
        if(mConfigurationValid) { result = (mFirst.isBusy() || mSecond.isBusy()); }
        return result;
    }

    @Override
    public void	                        mode(DcMotor.RunMode mode)
    {
        if(mConfigurationValid) {
            mFirst.setMode(mode);
            mSecond.setMode(mode);
        }
    }

    @Override
    public void	                        direction(DcMotorSimple.Direction direction)
    {
        if(direction != mDirection && mConfigurationValid) {

            if(     mFirst.getDirection()  == DcMotor.Direction.FORWARD) { mFirst.setDirection(DcMotor.Direction.REVERSE);  }
            else if(mFirst.getDirection()  == DcMotor.Direction.REVERSE) { mFirst.setDirection(DcMotor.Direction.FORWARD);  }

            if(     mSecond.getDirection() == DcMotor.Direction.FORWARD) { mSecond.setDirection(DcMotor.Direction.REVERSE); }
            else if(mSecond.getDirection() == DcMotor.Direction.REVERSE) { mSecond.setDirection(DcMotor.Direction.FORWARD); }

            mDirection = direction;

        }
    }

    @Override
    public void	                        targetPosition(int position)
    {
        if(mConfigurationValid) {
            mFirst.setTargetPosition(mFirstInvertPosition * position);
            mSecond.setTargetPosition(mSecondInvertPosition * position);
        }
    }

    @Override
    public void	                        zeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        if(mConfigurationValid) {
            mFirst.setZeroPowerBehavior(zeroPowerBehavior);
            mSecond.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    @Override
    public void	                        power(double power)
    {
        if(mConfigurationValid) {
            mFirst.setPower(power);
            mSecond.setPower(power);
        }
    }

    /* -------------------- DcMotorEx functions -------------------- */

    @Override
    public PIDFCoefficients             PIDFCoefficients(DcMotor.RunMode mode){
        PIDFCoefficients result = null;
        if(mConfigurationValid) {
            result = mSecond.getPIDFCoefficients(mode);
        }
        return result;
    }

    @Override
    public void                        PIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients){
        if(mConfigurationValid) {
            mFirst.setPIDFCoefficients(mode, pidfCoefficients);
            mSecond.setPIDFCoefficients(mode, pidfCoefficients);
        }
    }

    @Override
    public void                        targetPositionTolerance(int tolerance)
    {
        if(mConfigurationValid) {
            mFirst.setTargetPositionTolerance(tolerance);
            mSecond.setTargetPositionTolerance(tolerance);
        }
    }

    @Override
    public int                         targetPositionTolerance()
    {
        int result = -1;
        if(mConfigurationValid) {
            result = mSecond.getTargetPositionTolerance();
        }
        return result;

    }

    @Override
    public double                       velocity()
    {
        double result = 0;
        if(mConfigurationValid) {
            result = 0.5 * mSecond.getVelocity() + 0.5 * mFirst.getVelocity();
        }
        return result;

    }

}
