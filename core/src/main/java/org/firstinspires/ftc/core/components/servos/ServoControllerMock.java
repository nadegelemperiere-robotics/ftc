/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Controller managing mock servos
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.servos;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.ServoController;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class ServoControllerMock implements ServoControllerComponent {

    LogManager                  mLogger;

    boolean                     mConfigurationValid;

    ServoController.PwmStatus   mStatus;


    /* -------------- Constructors --------------- */
    public ServoControllerMock( LogManager logger)
    {
        mConfigurationValid = true;
        mLogger             = logger;
        mStatus             = ServoController.PwmStatus.DISABLED;
    }

    /* --------------------- Custom functions ---------------------- */

    /**
     * Determines if the mock servo controller component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    /* ----------------- ServoController functions ----------------- */

    @Override
    public void	                        pwmEnable(){
        mStatus = ServoController.PwmStatus.ENABLED;
    }

    @Override
    public void	                        pwmDisable(){
        mStatus = ServoController.PwmStatus.DISABLED;
    }

    @Override
    public ServoController.PwmStatus	pwmStatus() { return mStatus; }

}
