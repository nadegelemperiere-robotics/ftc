/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   EncoderSingle is a the encoder implementation for
   single motors
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.motors;

/* PedroPathing includes */
import com.pedropathing.localization.Encoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.core.tools.LogManager;


public class EncoderSingle extends EncoderComponent {

    final LogManager    mLogger;

    boolean             mConfigurationValid;

    final String        mName;

    final Encoder       mEncoder;

    /* -------------- Constructors --------------- */
    public EncoderSingle(DcMotorEx motor, String name, LogManager logger)
    {
        mConfigurationValid  = true;

        mLogger     = logger;

        mName       = name;

        mEncoder    = new Encoder(motor);

        if(motor == null) { mConfigurationValid = false; }
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
            mEncoder.setDirection(multiplier);
        }
    }

    /**
     * This resets the Encoder's position and the current and previous position in the code.
     */
    public void    reset() {
        if(mConfigurationValid) { mEncoder.reset(); }
    }

    /**
     * This updates the Encoder's tracked current position and previous position.
     */
    public void    update() {
        if(mConfigurationValid) { mEncoder.update(); }
    }

    /**
     * This returns the multiplier/direction of the Encoder.
     *
     * @return returns the multiplier
     */
    public double  getMultiplier() {
        double result = 0;
        if(mConfigurationValid) { result = mEncoder.getMultiplier(); }
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
        if(mConfigurationValid) { result = mEncoder.getDeltaPosition(); }
        return result;
    }

}