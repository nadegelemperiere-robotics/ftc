/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   EncoderMock is a the encoder implementation for
   mocked motors
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.motors;

/* PedroPathing includes */
import com.pedropathing.localization.Encoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class EncoderMock extends EncoderComponent {

    final LogManager        mLogger;

    boolean                 mConfigurationValid;

    final String            mName;

    final MotorComponent    mEncoder;
    double                  mPreviousPosition;
    double                  mCurrentPosition;
    double                  mMultiplier;

    /* -------------- Constructors --------------- */
    public EncoderMock(MotorComponent motor, String name, LogManager logger)
    {
        mConfigurationValid  = true;

        mLogger = logger;

        mName   = name;

        mEncoder  = motor;
        mMultiplier = 1.0;

        if(mEncoder == null) { mConfigurationValid = false; }
        else {
            mPreviousPosition = mEncoder.currentPosition();
            mCurrentPosition = mEncoder.currentPosition();
        }

    }


    /* --------------------- Custom functions ---------------------- */

    /**
     * Configuration status accessor
     * @return true if the encoder is ready, false otherwise
     */
    public boolean isConfigured() { return mConfigurationValid; }

    /* -------------- Encoder methods override ------------- */
    /**
     * This sets the direction/multiplier of the Encoder. Setting 1 or -1 will make the Encoder track
     * forward or in reverse, respectively. Any multiple of either one will scale the Encoder's output
     * by that amount.
     *
     * @param multiplier the multiplier/direction to set
     */
    public void    setDirection(double multiplier) {
        if(mConfigurationValid) {
            mMultiplier = multiplier;
        }
    }

    /**
     * This resets the Encoder's position and the current and previous position in the code.
     */
    public void    reset() {
    }

    /**
     * This updates the Encoder's tracked current position and previous position.
     */
    public void    update() {
        if(mConfigurationValid) {
            mPreviousPosition = mCurrentPosition;
            mCurrentPosition = mEncoder.currentPosition();
        }
    }

    /**
     * This returns the multiplier/direction of the Encoder.
     *
     * @return returns the multiplier
     */
    public double  getMultiplier() {
        double result = 0;
        if(mConfigurationValid) { result = mMultiplier; }
        return result;
    }

    /**
     * This returns the change in position from the previous position to the current position. One
     * important thing to note is that this encoder does not track velocity, only change in position.
     * This is because I am using a pose exponential method of localization, which doesn't need the
     * velocity of the encoders. Velocity of the robot is calculated in the localizer using an elapsed
     * time timer there.
     *
     * @return returns the change in position of the Encoder
     */
    public double  getDeltaPosition() {
        double result = 0;
        if(mConfigurationValid) { result = mMultiplier * (mCurrentPosition - mPreviousPosition); }
        return result;
    }


}