/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Encoder filtering class
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.odometers;

/* Qualcomm includes */
import com.qualcomm.robotcore.util.ElapsedTime;

/* ACME robotics includes */
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Components includes */
import org.firstinspires.ftc.core.components.motors.MotorComponent;

public class Encoder {

    final LogManager                    mLogger;

    boolean                             mConfigurationValid;
    boolean                             mIsFirstTime;

    final MotorComponent                mMotor;

    final ElapsedTime                   mTimer;
    
    double[]                            mHistory;
    double                              mLastPosition;
    int                                 mCurrentIndex;

    public  Encoder(MotorComponent motor,LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = true;
        mIsFirstTime        = true;

        mMotor = motor;
        if(mMotor == null) { mConfigurationValid = false; }
        
        mTimer = new ElapsedTime();
        mHistory = new double[3];
        mLastPosition = 0;
        mCurrentIndex = 0;

    }

    public boolean                      isConfigured() { return mConfigurationValid;}


    public PositionVelocityPair update() {

        PositionVelocityPair result = null;
        if (mConfigurationValid) {


            int currentPosition = mMotor.currentPosition();
            double velocity = mMotor.velocity();
            double deltaTime = mTimer.seconds();
            mTimer.reset();

            if (mIsFirstTime) {
                result = new PositionVelocityPair(currentPosition, 0, currentPosition, 0);
                mLastPosition = currentPosition;
                mHistory[0] = velocity;
                mHistory[1] = velocity;
                mHistory[2] = velocity;
                mIsFirstTime = false;
            } else {
                double velComputed = (currentPosition - mLastPosition) / deltaTime;

                mHistory[mCurrentIndex] = velComputed;
                mCurrentIndex = (mCurrentIndex + 1) % 3;

                double median = Math.max(
                        Math.min(mHistory[0], mHistory[1]),
                        Math.min(Math.max(mHistory[0], mHistory[1]), mHistory[2]));

                // convert to uint16
                int real = (int) velocity & 0xFFFF;
                real += real % 20 / 4 * 0x10000;
                // estimate-based correction: it finds the nearest multiple of 5 to correct the upper bits by
                real += (int) (Math.round((median - real) / (int) (5 * 0x10000)) * 5 * 0x10000);

                result = new PositionVelocityPair(currentPosition, real, currentPosition, velocity);
            }


        }
        return result;
    }

}
