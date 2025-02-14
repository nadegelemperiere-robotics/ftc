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

    boolean                 mReady;

    String                  mName;

    ServoController         mController;

    /* -------------- Constructors --------------- */
    public ServoControllerSingle(ServoController controller, String name, LogManager logger)
    {
        mReady      = true;

        mLogger     = logger;

        mName       = name;

        mController = controller;

        if(mController == null) { mReady = false; }
    }

    /* --------------------- Custom functions ---------------------- */

    @Override
    public boolean                      isReady() { return mReady;}


    /* ----------------- ServoController functions ----------------- */

    @Override
    public void	                        pwmEnable(){
        if(mReady) {
            mController.pwmEnable();
        }
    }

    @Override
    public void	                        pwmDisable(){
        if(mReady) {
            mController.pwmDisable();
        }
    }
    @Override
    public ServoController.PwmStatus	getPwmStatus(){
        ServoController.PwmStatus result = ServoController.PwmStatus.DISABLED;
        if(mReady) {
            result = mController.getPwmStatus();
        }
        return result;
    }

}
