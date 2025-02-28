/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   EncoderCoupled is a the encoder implementation for
   Coupled motors
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.motors;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotorEx;

/* PedroPathing includes */
import com.pedropathing.localization.Encoder;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class EncoderCoupled extends EncoderComponent {

    final LogManager    mLogger;

    boolean             mConfigurationValid;

    final String        mName;

    final Encoder       mFirst;
    final Encoder       mSecond;

    /* -------------- Constructors --------------- */

    /**
     * Constructor from MotorComponent
     * @param first First coupled motor
     * @param second Second coupled motor
     * @param name Name of the encoder
     * @param logger Logger
     */
    public EncoderCoupled(DcMotorEx first, DcMotorEx second, String name, LogManager logger)
    {
        mConfigurationValid  = true;

        mLogger = logger;

        mName   = name;

        mFirst  = new Encoder(first);
        mSecond = new Encoder(second);

        if(first == null) { mConfigurationValid = false; }
        if(second == null) { mConfigurationValid = false; }
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
            mFirst.setDirection(multiplier);
            mSecond.setDirection(multiplier);
        }
    }

    /**
     * This resets the Encoder's position and the current and previous position in the code.
     */
    public void    reset() {
        if(mConfigurationValid) {
            mFirst.reset();
            mSecond.reset();
        }
    }

    /**
     * This updates the Encoder's tracked current position and previous position.
     */
    public void    update() {
        if(mConfigurationValid) {
            mFirst.update();
            mSecond.update();
        }
    }

    /**
     * This returns the multiplier/direction of the Encoder.
     *
     * @return returns the multiplier
     */
    public double  getMultiplier() {
        double result = 0;
        if(mConfigurationValid) { result = 0.5 * mFirst.getMultiplier() + 0.5 * mSecond.getMultiplier(); }
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
        if(mConfigurationValid) { result = 0.5 * mFirst.getDeltaPosition() + 0.5 * mSecond.getDeltaPosition(); }
        return result;
    }

}