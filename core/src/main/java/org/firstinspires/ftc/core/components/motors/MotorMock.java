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

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.Mock;

public class MotorMock implements MotorComponent {

    LogManager                  mLogger;

    boolean                     mConfigurationValid;

    String                      mName;

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

    @Override
    public String                       logPositions()
    {
        return "\n  P : " + mPosition + " V : " + 0.0 + " P : " + mPower;
    }

    /* ------------------ Configurable functions ------------------- */

    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    @Override
    public void                         read(JSONObject reader) { }

    @Override
    public void                         write(JSONObject writer) { }

    @Override
    public String                       logConfigurationHTML() { return "<p>Mock</p>\n"; }

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
        switch(mode) {
            case RUN_TO_POSITION: Mock.instance().mMockedComponents.put(mName + "-mode", 0.0); break;
            case RUN_USING_ENCODER: Mock.instance().mMockedComponents.put(mName + "-mode", 1.0); break;
            case RUN_WITHOUT_ENCODER: Mock.instance().mMockedComponents.put(mName + "-mode", 2.0); break;
        }

        mMode = mode;
    }

    @Override
    public void	                        direction(DcMotorSimple.Direction direction) { mDirection = direction; }

    @Override
    public void	                        targetPosition(int position) {
        Mock.instance().mMockedComponents.put(mName + "-position",(double)position);
        mPosition = position;
    }

    @Override
    public void	                        zeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) { mBehavior = zeroPowerBehavior; }

    @Override
    public void	                        power(double power) {
        Mock.instance().mMockedComponents.put(mName + "-power",power);
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

}
