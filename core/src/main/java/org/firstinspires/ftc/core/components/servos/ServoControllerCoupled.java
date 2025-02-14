/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Coupled Controller managing coupled servos together
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.servos;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.ServoController;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class ServoControllerCoupled implements ServoControllerComponent {

    LogManager              mLogger;

    boolean                 mReady;

    String                  mName;

    ServoController         mFirst;
    ServoController         mSecond;

    /* -------------- Constructors --------------- */
    public ServoControllerCoupled(ServoController first, ServoController second, String name, LogManager logger)
    {
        mReady  = true;

        mLogger = logger;

        mName   = name;

        mFirst  = first;
        mSecond = second;

        if(mFirst  == null) { mReady = false; }
        if(mSecond == null) { mReady = false; }

        if(mReady && mFirst.equals(mSecond)) {
            // If coupled servos have the same controller, it won't be possible to power one
            // without powering the other. It won't be possible to pilot them separately and
            // check if coupling won't destroy them.
            mLogger.warning("Coupled servos " + mName + " have same controller.");
        }
    }

    /* --------------------- Custom functions ---------------------- */

    @Override
    public boolean                      isReady() { return mReady;}


    /* ----------------- ServoController functions ----------------- */

    @Override
    public void	                        pwmEnable(){
        if(mReady) {
            mFirst.pwmEnable();
            mSecond.pwmDisable();
        }
    }

    @Override
    public void	                        pwmDisable(){
        if(mReady) {
            mFirst.pwmDisable();
            mSecond.pwmDisable();
        }
    }
    @Override
    public ServoController.PwmStatus	getPwmStatus(){
        ServoController.PwmStatus result = ServoController.PwmStatus.DISABLED;
        if(mReady) {
            result = mFirst.getPwmStatus();
        }
        return result;
    }

}
