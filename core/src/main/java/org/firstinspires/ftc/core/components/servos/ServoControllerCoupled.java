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

    final LogManager        mLogger;

    boolean                 mConfigurationValid;

    final String            mName;

    final ServoController   mFirst;
    final ServoController   mSecond;

    /* -------------- Constructors --------------- */
    public ServoControllerCoupled(ServoController first, ServoController second, String name, LogManager logger)
    {
        mConfigurationValid  = true;

        mLogger = logger;

        mName   = name;

        mFirst  = first;
        mSecond = second;

        if(mFirst  == null) { mConfigurationValid = false; }
        if(mSecond == null) { mConfigurationValid = false; }

        if(mConfigurationValid && mFirst.equals(mSecond)) {
            // If coupled servos have the same controller, it won't be possible to power one
            // without powering the other. It won't be possible to pilot them separately and
            // check if coupling won't destroy them.
            mLogger.warning("Coupled servos " + mName + " have same controller.");
        }
    }

    /* --------------------- Custom functions ---------------------- */

    /**
     * Determines if the coupled servo controller component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}


    /* ----------------- ServoController functions ----------------- */

    @Override
    public void	                        pwmEnable(){
        if(mConfigurationValid) {
            mFirst.pwmEnable();
            mSecond.pwmDisable();
        }
    }

    @Override
    public void	                        pwmDisable(){
        if(mConfigurationValid) {
            mFirst.pwmDisable();
            mSecond.pwmDisable();
        }
    }
    
    @Override
    public ServoController.PwmStatus	pwmStatus(){
        ServoController.PwmStatus result = ServoController.PwmStatus.DISABLED;
        if(mConfigurationValid) {
            result = mFirst.getPwmStatus();
        }
        return result;
    }

}
