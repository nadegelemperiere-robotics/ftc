/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   ServoComponent is an interface for servo management
   It supersedes Servo and provides additional capabilities
   such as :
   - Synchronizing 2 coupled servos
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.servos;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.ServoController;


public interface ServoControllerComponent {

    /* --------------------- Custom functions ---------------------- */

    boolean                     isReady();

    /* -------------- ServoController methods override ------------- */

    void	                    pwmEnable()	;
    void	                    pwmDisable();
    ServoController.PwmStatus	getPwmStatus();


}
