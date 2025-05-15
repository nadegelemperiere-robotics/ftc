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
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/* FTC controller includes */
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/**
 * Overloading of DcMotorEx to manage coupled motors
 */
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
    MotorControllerComponent    mController;


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
        mController             = null;
        mFirstInvertPosition    = 1;
        mSecondInvertPosition   = 1;

    }

    /* --------------------- Custom functions ---------------------- */

    /**
     * Retrieves the name of the coupled motor component.
     * @return The name of the component.
     */
    @Override
    public String                       getName() { return mName; }

    /**
     * Determines if encoder correction is required.
     *
     * @return True if at least one motor has inverted encoder behavior, false otherwise.
     */
    @Override
    public boolean                      getEncoderCorrection() { return ((mFirstInvertPosition == -1) || (mSecondInvertPosition == -1)); }

    /**
     * Enables or disables encoder correction.
     *
     * @param shallCorrect True to enable encoder correction, false to disable.
     */
    @Override
    public void                         setEncoderCorrection(boolean shallCorrect) {
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
     * Return the coupled encoder for this coupled motor
     * @return The coupled encoder
     */
    @Override
    public EncoderComponent             getEncoder() {
        return new EncoderCoupled(mFirst, mSecond,mName, mLogger);
    }

    /**
     * Sets the fraction of the motor power accessible
     * @param rate the power fraction
     */
    @Override
    public void                         setAchieveableMaxRPMFraction(double rate){
        if(mConfigurationValid) {
            MotorConfigurationType motorConfigurationType = mFirst.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(rate);
            mFirst.setMotorType(motorConfigurationType);
            motorConfigurationType = mSecond.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(rate);
            mSecond.setMotorType(motorConfigurationType);
        }
    }
    
    /**
     * Logs the current motor positions, velocities, and power levels.
     */
    @Override
    public void                         log() {

        if (mConfigurationValid) {
            mLogger.metric(LogManager.Target.DASHBOARD, mName+"-1-pos","" + mFirst.getCurrentPosition());
            mLogger.metric(LogManager.Target.DASHBOARD, mName+"-1-spd","" + mFirst.getVelocity());
            mLogger.metric(LogManager.Target.DASHBOARD, mName+"-1-pwr","" + mFirst.getPower());
            mLogger.metric(LogManager.Target.DASHBOARD, mName+"-2-pos","" + mSecond.getCurrentPosition());
            mLogger.metric(LogManager.Target.DASHBOARD, mName+"-2-spd","" + mSecond.getVelocity());
            mLogger.metric(LogManager.Target.DASHBOARD, mName+"-2-pwr","" + mSecond.getPower());
        }
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
        mController = null;

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

                if(mFirst != null) {
                    mFirst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mFirst.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                if(mSecond != null) {
                    mSecond.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mSecond.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

            }
            catch(JSONException e) { mLogger.error(e.getMessage()); }
        }

        if (mFirst == null) { mConfigurationValid = false; }
        if (mSecond == null) { mConfigurationValid = false; }

        if(mConfigurationValid) {
            mController = new MotorControllerCoupled(mFirst.getController(), mSecond.getController(), mName, mLogger);

            if(!mFirst.getManufacturer().equals(mSecond.getManufacturer())) {
                mLogger.warning("Coupled motor does not have the same manufacturers : "+  mFirst.getManufacturer() + " and " + mSecond.getManufacturer());
            }
        }

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
                        .append(" HW : ")
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

    /* ------------------ HardwareDevice functions ----------------- */

    /**
     * Returns an indication of the manufacturer of this device.
     * @return the manufacturer
     */
    @Override
    public Manufacturer                 getManufacturer()
    {
        Manufacturer result = Manufacturer.Unknown;
        if(mConfigurationValid) {
            result = mFirst.getManufacturer();
        }
        return result;
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.Note that this is a device-type-specific name; it has nothing to do with the name by which a user might have configured the device in a robot configuration.
     * @return the device name
     */
    @Override
    public String                       getDeviceName()
    {
        String result = "";
        if(mConfigurationValid) {
            result = mFirst.getDeviceName() + " coupled with " + mSecond.getDeviceName();
        }
        return result;
    }

    /**
     * Get connection information about this device in a human readable format
     * @return connection information
     */
    @Override
    public String                       getConnectionInfo() {
        String result = "";
        if(mConfigurationValid) {
            result = "First : " + mFirst.getConnectionInfo();
            result += "\nSecond : " + mSecond.getConnectionInfo();
        }
        return result;
    }

    /**
     * Version
     */
    @Override
    public int                          getVersion() {
        int result = -1;
        if(mConfigurationValid) {
            result = mFirst.getVersion();
        }
        return result;
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void                         resetDeviceConfigurationForOpMode() {
        if(mConfigurationValid) {
            mFirst.resetDeviceConfigurationForOpMode();
            mSecond.resetDeviceConfigurationForOpMode();
        }
    }

    /**
     * Closes this device
     */
    @Override
    public void                         close()
    {
        if(mConfigurationValid) {
            mFirst.close();
            mSecond.close();
        }
    }

    /* --------------------- DcMotor functions --------------------- */

    /**
     * Returns the current reading of the encoder for this motor. The units for this reading, that is, the number of ticks per revolution, are specific to the motor/encoder in question, and thus are not specified here.
     * @return the current reading of the encoder for this motor
     */
    @Override
    public int	                        getCurrentPosition()
    {
        int result = -1;
        if(mConfigurationValid) {
            result = (int) (0.5 * mFirstInvertPosition * mFirst.getCurrentPosition() +
                    mSecondInvertPosition * 0.5 * mSecond.getCurrentPosition());
        }
        return result;
    }

    /**
     * Returns the current logical direction in which this motor is set as operating.
     * @return the current logical direction in which this motor is set as operating.
     */
    @Override
    public DcMotorSimple.Direction      getDirection()
    {
        return mDirection;
    }

    /**
     * Returns the current run mode for this motor
     * @return the current run mode for this motor
     */
    @Override
    public DcMotor.RunMode	            getMode()
    {
        DcMotor.RunMode result =  DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        if (mConfigurationValid) { result = mFirst.getMode(); }
        return result;
    }

    /**
     * Returns the current target encoder position for this motor.
     * @return the current target encoder position for this motor.
     */
    @Override
    public int	                        getTargetPosition()
    {
        int result = -1;
        if(mConfigurationValid) {
            result = (int) (0.5 * mFirstInvertPosition * mFirst.getTargetPosition() +
                    0.5 * mSecondInvertPosition * mSecond.getTargetPosition());
        }
        return result;
    }

    /**
     * Returns the current behavior of the motor were a power level of zero to be applied.
     * @return the current behavior of the motor were a power level of zero to be applied.
     */
    @Override
    public DcMotor.ZeroPowerBehavior	getZeroPowerBehavior()
    {
        DcMotor.ZeroPowerBehavior result = DcMotor.ZeroPowerBehavior.UNKNOWN;
        if(mConfigurationValid) { result = mFirst.getZeroPowerBehavior(); }
        return result;
    }

    /**
     * Returns the current configured power level of the motor.
     * @return the current level of the motor, a value in the interval [0.0, 1.0]
     */
    @Override
    public double	                    getPower()
    {
        double result = -1;
        if(mConfigurationValid) {
            result = (0.5 * mFirst.getPower() + 0.5 * mSecond.getPower());
        }
        return result;
    }

    /**
     * Returns whether the motor is currently in a float power level.
     */
    @Override
    @Deprecated
    public boolean	                    getPowerFloat()
    {
        boolean result = false;
        if(mConfigurationValid) {
            result = mFirst.getPowerFloat() && mSecond.getPowerFloat();
        }
        return result;
    }

    /**
     * Returns true if the motor is currently advancing or retreating to a target position.
     * @return true if the motor is currently advancing or retreating to a target position.
     */
    @Override
    public boolean	                    isBusy()
    {
        boolean result = false;
        if(mConfigurationValid) { result = (mFirst.isBusy() || mSecond.isBusy()); }
        return result;
    }

    /**
     * Returns the assigned type for this motor. If no particular motor type has been configured, then getUnspecifiedMotorType will be returned.Note that the motor type for a given motor is initially assigned in the robot configuration user interface, though it may subsequently be modified using methods herein.
     * @return motor type
     */
    @Override
    public MotorConfigurationType       getMotorType() {
        MotorConfigurationType result = null;
        if(mConfigurationValid) {
            result = mFirst.getMotorType();
        }
        return result;
    }

    /**
     * Returns the underlying motor controller on which this motor is situated.
     * @return the underlying motor controller on which this motor is situated.f
     */
    @Override
    public DcMotorController            getController() {
        return mController;
    }

    /**
     * Unable to provide this method since each motor has a difference port
     * @return -1
     */
    @Override
    public int                          getPortNumber() {
        return -1;
    }

    /**
     * Sets the current run mode for this motor
     * @param mode the new current run mode for this motor
     */
    @Override
    public void	                        setMode(DcMotor.RunMode mode)
    {
        if(mConfigurationValid) {
            mFirst.setMode(mode);
            mSecond.setMode(mode);
        }
    }

    /**
     * Sets the logical direction in which this motor operates.
     * @param direction the direction to set for this motor
     */
    @Override
    public void	                        setDirection(DcMotorSimple.Direction direction)
    {
        if(direction != mDirection && mConfigurationValid) {

            if(     mFirst.getDirection()  == DcMotor.Direction.FORWARD) { mFirst.setDirection(DcMotor.Direction.REVERSE);  }
            else if(mFirst.getDirection()  == DcMotor.Direction.REVERSE) { mFirst.setDirection(DcMotor.Direction.FORWARD);  }

            if(     mSecond.getDirection() == DcMotor.Direction.FORWARD) { mSecond.setDirection(DcMotor.Direction.REVERSE); }
            else if(mSecond.getDirection() == DcMotor.Direction.REVERSE) { mSecond.setDirection(DcMotor.Direction.FORWARD); }

            mDirection = direction;

        }
    }

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat and then actively hold thereat. This behavior is similar to the operation of a servo. The maximum speed at which this advance or retreat occurs is governed by the power level currently set on the motor. While the motor is advancing or retreating to the desired target position, isBusy() will return true.
     * Note that adjustment to a target position is only effective when the motor is in RUN_TO_POSITION RunMode. Note further that, clearly, the motor must be equipped with an encoder in order for this mode to function properly.
     * @param position the desired encoder target position
     */
    @Override
    public void	                        setTargetPosition(int position)
    {
        if(mConfigurationValid) {
            mFirst.setTargetPosition(mFirstInvertPosition * position);
            mSecond.setTargetPosition(mSecondInvertPosition * position);
        }
    }

    /**
     * Sets the behavior of the motor when a power level of zero is applied.
     * @param zeroPowerBehavior the new behavior of the motor when a power level of zero is applied.
     */
    @Override
    public void	                        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        if(mConfigurationValid) {
            mFirst.setZeroPowerBehavior(zeroPowerBehavior);
            mSecond.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    /**
     * Sets the power level of the motor, expressed as a fraction of the maximum possible power / speed supported according to the run mode in which the motor is operating.
     * Setting a power level of zero will brake the motor
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     */
    @Override
    public void	                        setPower(double power)
    {
        if(mConfigurationValid) {
            mFirst.setPower(power);
            mSecond.setPower(power);
        }
    }

    /**
     * Sets the zero power behavior of the motor to FLOAT, then applies zero power to that motor.
     */
    @Override
    @Deprecated
    public void	                        setPowerFloat()
    {
        if(mConfigurationValid) {
            mFirst.setPowerFloat();
            mSecond.setPowerFloat();
        }
    }

    /**
     * Sets the assigned type of this motor. Usage of this method is very rare.
     * @param type the new assigned type for this motor
     */
    @Override
    public void                         setMotorType(MotorConfigurationType type) {
        if(mConfigurationValid) {
            mFirst.setMotorType(type);
            mSecond.setMotorType(type);
        }
    }

    /* -------------------- DcMotorEx functions -------------------- */

    /**
     * Returns the current consumed by the motor
     * @param unit current units
     * @return the current consumed by the motor
     */
    @Override
    public double                       getCurrent(CurrentUnit unit) {
        double result = 0;
        if(mConfigurationValid) {
            result = 0.5 * mFirst.getCurrent(unit) + 0.5 * mSecond.getCurrent(unit);
        }
        return result;
    }

    /**
     * Returns the current alert for by the motor
     * @param unit current units
     * @return the current alert for by the motor
     */
    @Override
    public double                       getCurrentAlert(CurrentUnit unit) {
        double result = 0;
        if(mConfigurationValid) {
            result = 0.5 * mFirst.getCurrentAlert(unit) + 0.5 * mSecond.getCurrentAlert(unit);
        }
        return result;
    }

    /**
     * Returns whether the current consumption of this motor exceeds the alert threshold.
     * @return true if threshold exceeded, false otherwise
     */
    @Override
    public boolean                      isOverCurrent() {
        boolean result = false;
        if(mConfigurationValid) {
            result = mFirst.isOverCurrent() || mSecond.isOverCurrent();
        }
        return result;
    }


    /**
     * Returns the PIDF control coefficients used when running in the indicated mode on this motor.
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @return the PIDF control coefficients used when running in the indicated mode on this motor
     */
    @Override
    public PIDFCoefficients             getPIDFCoefficients(DcMotor.RunMode mode){
        PIDFCoefficients result = null;
        if(mConfigurationValid) {
            result = mSecond.getPIDFCoefficients(mode);
        }
        return result;
    }

    /**
     * Returns the PIDF control coefficients used when running in the indicated mode on this motor.
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @return the PIDF control coefficients used when running in the indicated mode on this motor
     */
    @Override
    public PIDCoefficients              getPIDCoefficients(DcMotor.RunMode mode){
        PIDCoefficients result = null;
        if(mConfigurationValid) {
            result = mSecond.getPIDCoefficients(mode);
        }
        return result;
    }

    /**
     * Returns the current target positioning tolerance of this motor
     * @return the current target positioning tolerance of this motor
     */
    @Override
    public int                         getTargetPositionTolerance()
    {
        int result = -1;
        if(mConfigurationValid) {
            result = mSecond.getTargetPositionTolerance();
        }
        return result;

    }

    /**
     * Returns the current velocity of the motor, in ticks per second
     * @return the current velocity of the motor
     */
    @Override
    public double                       getVelocity()
    {
        double result = 0;
        if(mConfigurationValid) {
            result = 0.5 * mSecond.getVelocity() + 0.5 * mFirst.getVelocity();
        }
        return result;

    }

    /**
     * Returns the current velocity of the motor, in angular unit per second
     * @return the current velocity of the motor
     */
    @Override
    public double                       getVelocity(AngleUnit unit)
    {
        double result = 0;
        if(mConfigurationValid) {
            result = 0.5 * mSecond.getVelocity(unit) + 0.5 * mFirst.getVelocity(unit);
        }
        return result;

    }

    /**
     * Returns whether this motor is energized
     */
    @Override
    public boolean                      isMotorEnabled() {
        boolean result = false;
        if(mConfigurationValid) {
            result = mFirst.isMotorEnabled() && mSecond.isMotorEnabled();
        }
        return result;
    }

    /**
     * Sets the current alert for by the motor
     * @param unit current units
     * @param alert the alert threshold for by the motor
     */
    @Override
    public void                         setCurrentAlert(double alert, CurrentUnit unit) {
        if(mConfigurationValid) {
            mFirst.setCurrentAlert(alert, unit);
            mSecond.setCurrentAlert(alert, unit);
        }
    }

    /**
     * Sets the PIDF control coefficients for one of the PID modes of this motor. 
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @param pidfCoefficients the new coefficients to use when in that mode on this motor
     */
    @Override
    public void                        setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients){
        if(mConfigurationValid) {
            mFirst.setPIDFCoefficients(mode, pidfCoefficients);
            mSecond.setPIDFCoefficients(mode, pidfCoefficients);
        }
    }

    /**
     * Sets the PID control coefficients for one of the PID modes of this motor.
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @param pidCoefficients the new coefficients to use when in that mode on this motor
     */
    @Override
    public void                        setPIDCoefficients(DcMotor.RunMode mode, PIDCoefficients pidCoefficients){
        if(mConfigurationValid) {
            mFirst.setPIDCoefficients(mode, pidCoefficients);
            mSecond.setPIDCoefficients(mode, pidCoefficients);
        }
    }
    /**
     * A shorthand for setting the PIDF coefficients for the DcMotor.RunMode.RUN_USING_ENCODER mode.
     */
    @Override
    public void                        setVelocityPIDFCoefficients(double p, double i, double d, double f){
        if(mConfigurationValid) {
            mFirst.setVelocityPIDFCoefficients(p,i,d,f);
            mSecond.setVelocityPIDFCoefficients(p,i,d,f);
        }
    }

    /**
     * A shorthand for setting the PIDF coefficients for the DcMotor.RunMode.RUN_TO_POSITION mode. MotorControlAlgorithm.PIDF is used. Readers are reminded that DcMotor.RunMode.RUN_TO_POSITION mode makes use of both the coefficients set for RUN_TO_POSITION and the coefficients set for RUN_WITH_ENCODER, due to the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal on each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double- layering, only the proportional ('p') coefficient makes logical sense for use in the RUN_TO_POSITION coefficients.
     */
    @Override
    public void                        setPositionPIDFCoefficients(double p){
        if(mConfigurationValid) {
            mFirst.setPositionPIDFCoefficients(p);
            mSecond.setPositionPIDFCoefficients(p);
        }
    }

    /**
     * Sets the target positioning tolerance of this motor
     * @param tolerance the desired tolerance, in encoder ticks
     */
    @Override
    public void                         setTargetPositionTolerance(int tolerance)
    {
        if(mConfigurationValid) {
            mFirst.setTargetPositionTolerance(tolerance);
            mSecond.setTargetPositionTolerance(tolerance);
        }
    }


    /**
     * Individually energizes this particular motor
     */
    @Override
    public void                         setMotorEnable() {
        if(mConfigurationValid) {
            mFirst.setMotorEnable();
            mSecond.setMotorEnable();
        }
    }

    /**
     * Individually de-energizes this particular motor
     */
    @Override
    public void                         setMotorDisable() {
        if(mConfigurationValid) {
            mFirst.setMotorDisable();
            mSecond.setMotorDisable();
        }
    }

    /**
     * Sets the velocity of the motor
     * @param ticks  the desired ticks per second
     */
    @Override
    public void                         setVelocity(double ticks) {
        if(mConfigurationValid) {
            mFirst.setVelocity(ticks);
            mSecond.setVelocity(ticks);
        }
    }

    /**
     * Sets the velocity of the motor
     * @param angularRate   the desired angular rate, in units per second
     * @param unit          the units in which angularRate is expressed
     *
     */
    @Override
    public void                         setVelocity(double angularRate, AngleUnit unit){
        if(mConfigurationValid) {
            mFirst.setVelocity(angularRate, unit);
            mSecond.setVelocity(angularRate, unit);
        }
    }

}
