/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   CoupledMotor class overloads the FTC motor class to manage
   A couple of motors both turning the same hardware.

   Note that this is a dangerous situation which can result in
   motor destruction if not correctly tuned. The coupled motors
   shall be the same model
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.motors;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class MotorSingle implements MotorComponent {

    final LogManager            mLogger;

    boolean                     mConfigurationValid;
    final String                mName;
    String                      mHwName;

    final HardwareMap           mMap;
    DcMotorEx                   mMotor;
    MotorControllerComponent    mController;

    int                         mInvertPosition;

    /* ----------------------- Constructors ------------------------ */
    /**
     * Constructs a MotorSingle instance.
     *
     * @param name   The name of the single motor component.
     * @param hwMap  The FTC HardwareMap to retrieve motor hardware.
     * @param logger The logging manager for error reporting and debugging.
     */
    public MotorSingle(String name, HardwareMap hwMap, LogManager logger)
    {
        mLogger             = logger;
        mName               = name;
        mHwName             = "";
        mConfigurationValid = false;

        mMap                = hwMap;

        mMotor              = null;
        mController         = null;
        mInvertPosition     = 1;
        
    }

    /* --------------------- Custom functions ---------------------- */

    /**
     * Retrieves the name of the single motor component
     * @return The name of the component.
     */
    @Override
    public String                       getName() { return mName; }

    /**
     * Determines if encoder correction is required.
     * @return True if at least one motor has inverted encoder behavior, false otherwise.
     */
    @Override
    public boolean                      getEncoderCorrection() { return (mInvertPosition == -1);}

    /**
     * Enables or disables encoder correction.
     * @param shallCorrect True to enable encoder correction, false to disable.
     */
    @Override
    public void                         setEncoderCorrection( boolean shallCorrect) {
        if (shallCorrect) { mInvertPosition = -1; }
        else {              mInvertPosition = 1;  }
    }

    /**
     * Return the encoder for this motor
     * @return The encoder
     */
    @Override
    public EncoderComponent             getEncoder() {
        return new EncoderSingle(mMotor,mName, mLogger);
    }

    /**
     * Sets the fraction of the motor power accessible
     * @param rate the power fraction
     */
    @Override
    public void                         setAchieveableMaxRPMFraction(double rate){
        if(mConfigurationValid) {
            MotorConfigurationType motorConfigurationType = mMotor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(rate);
            mMotor.setMotorType(motorConfigurationType);
        }
    }

    /**
     * Logs the current motor positions, velocities, and power levels.
     */
    @Override
    public void                         log()
    {
        if(mConfigurationValid) {
            mLogger.metric(LogManager.Target.DASHBOARD, mName+"-pos","" + mMotor.getCurrentPosition());
            mLogger.metric(LogManager.Target.DASHBOARD, mName+"-spd","" + mMotor.getVelocity());
            mLogger.metric(LogManager.Target.DASHBOARD, mName+"-pwr","" + mMotor.getPower());
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
        mMotor = null;
        mController = null;

        try {
            if (mMap != null && reader.has(sHwMapKey)) {
                mHwName = reader.getString(sHwMapKey);
                mMotor = mMap.tryGet(DcMotorEx.class, mHwName);
            }

            if (mMotor != null && reader.has(sDirectionKey)) {
                DcMotor.Direction direction = sString2Direction.get(reader.getString(sDirectionKey));
                mMotor.setDirection(direction);
            } else if (mMotor != null) {
                mMotor.setDirection(DcMotor.Direction.FORWARD);
            }

            if (mMotor != null && reader.has(sEncoderReverseKey)) {
                boolean shallReverse = reader.getBoolean(sEncoderReverseKey);
                if (shallReverse) { mInvertPosition = -1; }
                else { mInvertPosition = 1; }
            } else { mInvertPosition = 1; }

            if (mMotor != null) {
                mMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

        } catch (JSONException e) { mLogger.error(e.getMessage()); }

        if (mMotor == null) { mConfigurationValid = false; }
        if (mConfigurationValid) {
            mController = new MotorControllerSingle(mMotor.getController(), mName, mLogger);
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

            try {
                String direction = sDirection2String.get(mMotor.getDirection());
                writer.put(sHwMapKey, mHwName);
                writer.put(sDirectionKey, direction);
                writer.put(sEncoderReverseKey, mInvertPosition == -1);
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

            if (mMotor != null) {
                result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                        .append("HW : ")
                        .append(mHwName)
                        .append(" - DIR : ")
                        .append(sDirection2String.get(mMotor.getDirection()))
                        .append(" - ENC : ")
                        .append(mInvertPosition == -1)
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

            if (mMotor != null) {
                result.append(header)
                        .append("> HW : ")
                        .append(mHwName)
                        .append(" - DIR : ")
                        .append(sDirection2String.get(mMotor.getDirection()))
                        .append(" - ENC : ")
                        .append(mInvertPosition == -1)
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
            result = mMotor.getManufacturer();
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
            result = mMotor.getDeviceName();
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
            result = mMotor.getConnectionInfo();
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
            result = mMotor.getVersion();
        }
        return result;
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void                         resetDeviceConfigurationForOpMode() {
        if(mConfigurationValid) {
            mMotor.resetDeviceConfigurationForOpMode();
        }
    }

    /**
     * Closes this device
     */
    @Override
    public void                         close()
    {
        if(mConfigurationValid) {
            mMotor.close();
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
            result = mInvertPosition * mMotor.getCurrentPosition();
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
        DcMotorSimple.Direction result = DcMotorSimple.Direction.FORWARD;
        if(mConfigurationValid) { result = mMotor.getDirection(); }
        return result;
    }

    /**
     * Returns the current run mode for this motor
     * @return the current run mode for this motor
     */
    @Override
    public DcMotor.RunMode	            getMode()
    {
        DcMotor.RunMode result =  DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        if (mConfigurationValid) { result = mMotor.getMode(); }
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
            result = mInvertPosition * mMotor.getTargetPosition();
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
        if(mConfigurationValid) { result = mMotor.getZeroPowerBehavior(); }
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
        if(mConfigurationValid) { result = mMotor.getPower(); }
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
            result = mMotor.getPowerFloat();
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
        if(mConfigurationValid) { result = mMotor.isBusy(); }
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
            result = mMotor.getMotorType();
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
        int result = -1;
        if(mConfigurationValid) {
            result = mMotor.getPortNumber();
        }
        return result;
    }

    /**
     * Sets the current run mode for this motor
     * @param mode the new current run mode for this motor
     */
    @Override
    public void	                        setMode(DcMotor.RunMode mode)
    {
        if(mConfigurationValid) {
            mMotor.setMode(mode);
        }
    }

    /**
     * Sets the logical direction in which this motor operates.
     * @param direction the direction to set for this motor
     */
    @Override
    public void	                        setDirection(DcMotorSimple.Direction direction)
    {
        if(mConfigurationValid) {
            mMotor.setDirection(direction);
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
            mMotor.setTargetPosition(mInvertPosition * position);
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
            mMotor.setZeroPowerBehavior(zeroPowerBehavior);
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
            mMotor.setPower(power);
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
            mMotor.setPowerFloat();
        }
    }

    /**
     * Sets the assigned type of this motor. Usage of this method is very rare.
     * @param type the new assigned type for this motor
     */
    @Override
    public void                         setMotorType(MotorConfigurationType type) {
        if(mConfigurationValid) {
            mMotor.setMotorType(type);
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
            result = mMotor.getCurrent(unit);
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
            result = mMotor.getCurrentAlert(unit);
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
            result = mMotor.isOverCurrent();
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
            result = mMotor.getPIDFCoefficients(mode);
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
            result = mMotor.getPIDCoefficients(mode);
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
            result = mMotor.getTargetPositionTolerance();
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
            result = mMotor.getVelocity();
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
            result = mMotor.getVelocity(unit);
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
            result = mMotor.isMotorEnabled();
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
            mMotor.setCurrentAlert(alert, unit);
        }
    }

    /**
     * Sets the PIDF control coefficients for one of the PID modes of this motor.
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @param pidfCoefficients the new coefficients to use when in that mode on this motor
     */
    @Override
    public void                         setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients){
        if(mConfigurationValid) {
            mMotor.setPIDFCoefficients(mode, pidfCoefficients);
        }
    }

    /**
     * Sets the PID control coefficients for one of the PID modes of this motor.
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @param pidCoefficients the new coefficients to use when in that mode on this motor
     */
    @Override
    public void                         setPIDCoefficients(DcMotor.RunMode mode, PIDCoefficients pidCoefficients){
        if(mConfigurationValid) {
            mMotor.setPIDCoefficients(mode, pidCoefficients);
        }
    }

    /**
     * A shorthand for setting the PIDF coefficients for the DcMotor.RunMode.RUN_USING_ENCODER mode.
     */
    @Override
    public void                        setVelocityPIDFCoefficients(double p, double i, double d, double f){
        if(mConfigurationValid) {
            mMotor.setVelocityPIDFCoefficients(p,i,d,f);
        }
    }

    /**
     * A shorthand for setting the PIDF coefficients for the DcMotor.RunMode.RUN_TO_POSITION mode. MotorControlAlgorithm.PIDF is used. Readers are reminded that DcMotor.RunMode.RUN_TO_POSITION mode makes use of both the coefficients set for RUN_TO_POSITION and the coefficients set for RUN_WITH_ENCODER, due to the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal on each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double- layering, only the proportional ('p') coefficient makes logical sense for use in the RUN_TO_POSITION coefficients.
     */
    @Override
    public void                        setPositionPIDFCoefficients(double p){
        if(mConfigurationValid) {
            mMotor.setPositionPIDFCoefficients(p);
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
            mMotor.setTargetPositionTolerance(tolerance);
        }
    }

    /**
     * Individually energizes this particular motor
     */
    @Override
    public void                         setMotorEnable() {
        if(mConfigurationValid) {
            mMotor.setMotorEnable();
        }
    }

    /**
     * Individually de-energizes this particular motor
     */
    @Override
    public void                         setMotorDisable() {
        if(mConfigurationValid) {
            mMotor.setMotorDisable();
        }
    }

    /**
     * Sets the velocity of the motor
     * @param ticks  the desired ticks per second
     */
    @Override
    public void                         setVelocity(double ticks) {
        if(mConfigurationValid) {
            mMotor.setVelocity(ticks);
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
            mMotor.setVelocity(angularRate, unit);
        }
    }

}
