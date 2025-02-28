/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
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
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    }

    /* --------------------- Custom functions ---------------------- */

    @Override
    public String                       name() { return mName; }

    @Override
    public boolean                      encoderCorrection() { return false; }

    @Override
    public void                         encoderCorrection(boolean ShallCorrect) {  }

    /**
     * Return the encoder for this motor
     * @return The encoder
     */
    @Override
    public EncoderComponent             encoder() {
        return new EncoderMock(this, mName, mLogger);
    }

    /**
     * Logs the current motor positions, velocities, and power levels.
     *
     * @return A formatted string containing motor telemetry data.
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

    /* --------------------- DcMotor functions --------------------- */

    @Override
    public int	                        currentPosition() { return mPosition; }

    @Override
    public DcMotorSimple.Direction      direction() { return mDirection; }

    @Override
    public DcMotor.RunMode	            mode() { return mMode; }

    @Override
    public int	                        targetPosition() { return mPosition; }

    @Override
    public DcMotor.ZeroPowerBehavior	zeroPowerBehavior() { return mBehavior; }

    @Override
    public double                   	power() { return mPower; }

    @Override
    public boolean	                    isBusy() { return false; }

    @Override
    public void	                        mode(DcMotor.RunMode mode) {
        InterOpMode.instance().add(mName + "-mode", mode);
        mMode = mode;
    }

    @Override
    public void	                        direction(DcMotorSimple.Direction direction) { mDirection = direction; }

    @Override
    public void	                        targetPosition(int position) {
        InterOpMode.instance().add(mName + "-position",position);
        mPosition = position;
    }

    @Override
    public void	                        zeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) { mBehavior = zeroPowerBehavior; }

    @Override
    public void	                        power(double power) {
        InterOpMode.instance().add(mName + "-power",power);
        mPower = power;
    }

    /* -------------------- DcMotorEx functions -------------------- */

    @Override
    public PIDFCoefficients             PIDFCoefficients(DcMotor.RunMode mode) { return null; }

    @Override
    public void                         PIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients){
    }

    @Override
    public void                         targetPositionTolerance(int tolerance) { mTolerance = tolerance; }

    @Override
    public int                          targetPositionTolerance()  { return mTolerance; }

    @Override
    public double                       velocity()  { return 0.0; }

    @Override
    public void                         achieveableMaxRPMFraction(double value) {}

}
