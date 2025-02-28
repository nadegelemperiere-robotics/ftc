/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   ServoComponent is an interface for servo management
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.servos;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.ServoController;


public interface ServoControllerComponent {

    /* --------------------- Custom functions ---------------------- */

    boolean                     isConfigured();

    /* -------------- ServoController methods override ------------- */

    void	                    pwmEnable()	;
    void	                    pwmDisable();
    ServoController.PwmStatus	pwmStatus();


}
