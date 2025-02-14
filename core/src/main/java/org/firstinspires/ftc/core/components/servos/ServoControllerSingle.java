/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Controller managing  single component servo
   ------------------------------------------------------- */
package org.firstinspires.ftc.core.components.servos;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.ServoController;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class ServoControllerSingle implements ServoControllerComponent {

    LogManager              mLogger;

    boolean                 mConfigurationValid;

    String                  mName;

    ServoController         mController;

    /* -------------- Constructors --------------- */
    public ServoControllerSingle(ServoController controller, String name, LogManager logger)
    {
        mConfigurationValid      = true;

        mLogger     = logger;

        mName       = name;

        mController = controller;

        if(mController == null) { mConfigurationValid = false; }
    }

    /* --------------------- Custom functions ---------------------- */

    /**
     * Determines if the servo controller component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}


    /* ----------------- ServoController functions ----------------- */

    @Override
    public void	                        pwmEnable(){
        if(mConfigurationValid) {
            mController.pwmEnable();
        }
    }

    @Override
    public void	                        pwmDisable(){
        if(mConfigurationValid) {
            mController.pwmDisable();
        }
    }
    @Override
    public ServoController.PwmStatus	pwmStatus(){
        ServoController.PwmStatus result = ServoController.PwmStatus.DISABLED;
        if(mConfigurationValid) {
            result = mController.getPwmStatus();
        }
        return result;
    }

}
