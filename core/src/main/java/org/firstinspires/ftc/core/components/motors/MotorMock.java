/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   CoupledServo class overloads the FTC servo class to manage
   A couple of servos both turning the same hardware.

   Note that this is a dangerous situation which can result in
   servo destruction if not correctly tuned. The coupled servos
   shall be tuned so that each orientation of the hardware they
   both support correspond to the same position on the 2 servos.
   If wrongly tuned, each of the 2 coupled servos may end up
   each forcing into a position they can not reach without the
   other failing.

   This means for example that the 2 servos are the same model
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.motors;

/* JSON includes */
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.InterOpMode;

public class MotorMock implements MotorComponent {

    final LogManager            mLogger;

    final boolean               mConfigurationValid;

    final String                mName;

    DcMotor.Direction           mDirection;

    DcMotor.RunMode             mMode;
    int                         mPosition;
    DcMotor.ZeroPowerBehavior   mBehavior;
    double                      mPower;
    int                         mTolerance;

    MotorControllerComponent    mController;
    
    /* ----------------------- Constructors ------------------------ */

    public MotorMock(String name, LogManager logger)
    {
        mLogger             = logger;
        mName               = name;
        mConfigurationValid = true;
        mPosition           = 0;

        mDirection          = DcMotorSimple.Direction.FORWARD;
        mMode               = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        mBehavior           = DcMotor.ZeroPowerBehavior.UNKNOWN;
        
        mController         = new MotorControllerMock(mLogger);
        
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
    public boolean                      getEncoderCorrection() { return false; }

    /**
     * Enables or disables encoder correction.
     * @param shallCorrect True to enable encoder correction, false to disable.
     */
    @Override
    public void                         setEncoderCorrection(boolean shallCorrect) {  }

    /**
     * Return the encoder for this motor
     * @return The encoder
     */
    @Override
    public EncoderComponent             getEncoder() {
        return new EncoderMock(this, mName, mLogger);
    }

    /**
     * Sets the fraction of the motor power accessible
     * @param rate the power fraction
     */
    @Override
    public void                         setAchieveableMaxRPMFraction(double rate){}

    /**
     * Logs the current motor positions, velocities, and power levels.
     */
    @Override
    public void                         log()
    {
        mLogger.metric( LogManager.Target.DASHBOARD, mName+"-pos","" + mPosition);
        mLogger.metric( LogManager.Target.DASHBOARD, mName+"-spd","" + 0);
        mLogger.metric( LogManager.Target.DASHBOARD, mName+"-pwr","" + mPower);
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
    public void                         read(JSONObject reader) { }

    /**
     * Writes the current motor configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) { }

    /**
     * Generates an HTML representation of the motor configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted motor configuration.
     */
    @Override
    public String                       logConfigurationHTML() { return "<p style=\"padding-left:10px; font-size: 11px\">Mock</p>\n"; }

    /**
     * Generates a text-based representation of the motor configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted motor configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {   return header + "> Mock\n"; }

    /* ------------------ HardwareDevice functions ----------------- */

    /**
     * Returns an indication of the manufacturer of this device.
     * @return the manufacturer
     */
    @Override
    public Manufacturer                 getManufacturer() { return Manufacturer.Other; }

    /**
     * Returns a string suitable for display to the user as to the type of device.Note that this is a device-type-specific name; it has nothing to do with the name by which a user might have configured the device in a robot configuration.
     * @return the device name
     */
    @Override
    public String                       getDeviceName() { return "Motor Mock"; }

    /**
     * Get connection information about this device in a human readable format
     * @return connection information
     */
    @Override
    public String                       getConnectionInfo() { return ""; }

    /**
     * Version
     */
    @Override
    public int                          getVersion() { return 0; }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void                         resetDeviceConfigurationForOpMode() {}

    /**
     * Closes this device
     */
    @Override
    public void                         close() {}

    /* --------------------- DcMotor functions --------------------- */

    /**
     * Returns the current reading of the encoder for this motor. The units for this reading, that is, the number of ticks per revolution, are specific to the motor/encoder in question, and thus are not specified here.
     * @return the current reading of the encoder for this motor
     */
    @Override
    public int	                        getCurrentPosition() { return mPosition; }

    /**
     * Returns the current logical direction in which this motor is set as operating.
     * @return the current logical direction in which this motor is set as operating.
     */
    @Override
    public DcMotorSimple.Direction      getDirection() { return mDirection; }

    /**
     * Returns the current run mode for this motor
     * @return the current run mode for this motor
     */
    @Override
    public DcMotor.RunMode	            getMode() { return mMode; }

    /**
     * Returns the current target encoder position for this motor.
     * @return the current target encoder position for this motor.
     */
    @Override
    public int	                        getTargetPosition() { return mPosition; }

    /**
     * Returns the current behavior of the motor were a power level of zero to be applied.
     * @return the current behavior of the motor were a power level of zero to be applied.
     */
    @Override
    public DcMotor.ZeroPowerBehavior	getZeroPowerBehavior() { return mBehavior; }

    /**
     * Returns the current configured power level of the motor.
     * @return the current level of the motor, a value in the interval [0.0, 1.0]
     */
    @Override
    public double	                    getPower() { return mPower; }

    /**
     * Returns whether the motor is currently in a float power level.
     */
    @Override
    @Deprecated
    public boolean	                    getPowerFloat() { return mPower < 0.00000001; }

    /**
     * Returns true if the motor is currently advancing or retreating to a target position.
     * @return true if the motor is currently advancing or retreating to a target position.
     */
    @Override
    public boolean	                    isBusy() { return false; }

    /**
     * Returns the assigned type for this motor. If no particular motor type has been configured, then getUnspecifiedMotorType will be returned.Note that the motor type for a given motor is initially assigned in the robot configuration user interface, though it may subsequently be modified using methods herein.
     * @return motor type
     */
    @Override
    public MotorConfigurationType       getMotorType() {
        MotorConfigurationType result = new MotorConfigurationType();
        return result;
    }

    /**
     * Returns the underlying motor controller on which this motor is situated.
     * @return the underlying motor controller on which this motor is situated.f
     */
    @Override
    public DcMotorController            getController() { return mController; }

    /**
     * Unable to provide this method since each motor has a difference port
     * @return -1
     */
    @Override
    public int                          getPortNumber() { return -1; }

    /**
     * Sets the current run mode for this motor
     * @param mode the new current run mode for this motor
     */
    @Override
    public void	                        setMode(DcMotor.RunMode mode) {
        InterOpMode.instance().add(mName + "-mode", mode);
        mMode = mode;
    }

    /**
     * Sets the logical direction in which this motor operates.
     * @param direction the direction to set for this motor
     */
    @Override
    public void	                        setDirection(DcMotorSimple.Direction direction) { mDirection = direction; }

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat and then actively hold thereat. This behavior is similar to the operation of a servo. The maximum speed at which this advance or retreat occurs is governed by the power level currently set on the motor. While the motor is advancing or retreating to the desired target position, isBusy() will return true.
     * Note that adjustment to a target position is only effective when the motor is in RUN_TO_POSITION RunMode. Note further that, clearly, the motor must be equipped with an encoder in order for this mode to function properly.
     * @param position the desired encoder target position
     */
    @Override
    public void	                        setTargetPosition(int position) {
        InterOpMode.instance().add(mName + "-position",position);
        mPosition = position;
    }

    /**
     * Sets the behavior of the motor when a power level of zero is applied.
     * @param zeroPowerBehavior the new behavior of the motor when a power level of zero is applied.
     */
    @Override
    public void	                        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) { mBehavior = zeroPowerBehavior; }

    /**
     * Sets the power level of the motor, expressed as a fraction of the maximum possible power / speed supported according to the run mode in which the motor is operating.
     * Setting a power level of zero will brake the motor
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     */
    @Override
    public void	                        setPower(double power) {
        InterOpMode.instance().add(mName + "-power",power);
        mPower = power;
    }

    /**
     * Sets the zero power behavior of the motor to FLOAT, then applies zero power to that motor.
     */
    @Override
    @Deprecated
    public void	                        setPowerFloat() { mPower = 0; mBehavior = ZeroPowerBehavior.FLOAT; }

    /**
     * Sets the assigned type of this motor. Usage of this method is very rare.
     * @param type the new assigned type for this motor
     */
    @Override
    public void                         setMotorType(MotorConfigurationType type) { }

    /* -------------------- DcMotorEx functions -------------------- */

    /**
     * Returns the current consumed by the motor
     * @param unit current units
     * @return the current consumed by the motor
     */
    @Override
    public double                       getCurrent(CurrentUnit unit) { return 0; }

    /**
     * Returns the current alert for by the motor
     * @param unit current units
     * @return the current alert for by the motor
     */
    @Override
    public double                       getCurrentAlert(CurrentUnit unit) { return 1.0; }

    /**
     * Returns whether the current consumption of this motor exceeds the alert threshold.
     * @return true if threshold exceeded, false otherwise
     */
    @Override
    public boolean                      isOverCurrent() { return false; }

    /**
     * Returns the PIDF control coefficients used when running in the indicated mode on this motor.
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @return the PIDF control coefficients used when running in the indicated mode on this motor
     */
    @Override
    public PIDFCoefficients             getPIDFCoefficients(DcMotor.RunMode mode) { return null; }

    /**
     * Returns the PIDF control coefficients used when running in the indicated mode on this motor.
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @return the PIDF control coefficients used when running in the indicated mode on this motor
     */
    @Override
    public PIDCoefficients              getPIDCoefficients(DcMotor.RunMode mode){ return null; }

    /**
     * Returns the current target positioning tolerance of this motor
     * @return the current target positioning tolerance of this motor
     */
    @Override
    public int                          getTargetPositionTolerance()  { return mTolerance; }

    /**
     * Returns the current velocity of the motor, in ticks per second
     * @return the current velocity of the motor
     */
    @Override
    public double                       getVelocity() { return 0.0; }

    /**
     * Returns the current velocity of the motor, in angular unit per second
     * @return the current velocity of the motor
     */
    @Override
    public double                       getVelocity(AngleUnit unit) { return 0.0; }

    /**
     * Returns whether this motor is energized
     */
    @Override
    public boolean                      isMotorEnabled() { return true; }

    /**
     * Sets the current alert for by the motor
     * @param unit current units
     * @param alert the alert threshold for by the motor
     */
    @Override
    public void                         setCurrentAlert(double alert, CurrentUnit unit) { }

    /**
     * Sets the PIDF control coefficients for one of the PID modes of this motor.
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @param pidfCoefficients the new coefficients to use when in that mode on this motor
     */
    @Override
    public void                         setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients){ }

    /**
     * Sets the PID control coefficients for one of the PID modes of this motor.
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @param pidCoefficients the new coefficients to use when in that mode on this motor
     */
    @Override
    public void                         setPIDCoefficients(DcMotor.RunMode mode, PIDCoefficients pidCoefficients){ }

    /**
     * A shorthand for setting the PIDF coefficients for the DcMotor.RunMode.RUN_USING_ENCODER mode.
     */
    @Override
    public void                         setVelocityPIDFCoefficients(double p, double i, double d, double f) { }

    /**
     * A shorthand for setting the PIDF coefficients for the DcMotor.RunMode.RUN_TO_POSITION mode. MotorControlAlgorithm.PIDF is used. Readers are reminded that DcMotor.RunMode.RUN_TO_POSITION mode makes use of both the coefficients set for RUN_TO_POSITION and the coefficients set for RUN_WITH_ENCODER, due to the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal on each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double- layering, only the proportional ('p') coefficient makes logical sense for use in the RUN_TO_POSITION coefficients.
     */
    @Override
    public void                         setPositionPIDFCoefficients(double p){ }

    /**
     * Sets the target positioning tolerance of this motor
     * @param tolerance the desired tolerance, in encoder ticks
     */
    @Override
    public void                         setTargetPositionTolerance(int tolerance) { mTolerance = tolerance; }

    /**
     * Individually energizes this particular motor
     */
    @Override
    public void                         setMotorEnable() { }

    /**
     * Individually de-energizes this particular motor
     */
    @Override
    public void                         setMotorDisable() {  }

    /**
     * Sets the velocity of the motor
     * @param ticks  the desired ticks per second
     */
    @Override
    public void                         setVelocity(double ticks) { }

    /**
     * Sets the velocity of the motor
     * @param angularRate   the desired angular rate, in units per second
     * @param unit          the units in which angularRate is expressed
     *
     */
    @Override
    public void                         setVelocity(double angularRate, AngleUnit unit){   }

}
