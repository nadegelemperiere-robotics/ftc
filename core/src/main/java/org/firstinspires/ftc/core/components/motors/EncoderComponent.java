/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   EncoderComponent is an interface for motor encoders
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.motors;

/* PedroPathing includes */
import com.pedropathing.localization.Encoder;


public abstract class EncoderComponent extends Encoder {
    
    public EncoderComponent() {
        super(null);
    }
    
    /* --------------------- Custom functions ---------------------- */

    public abstract boolean isConfigured();

    /* -------------- Encoder methods override ------------- */

    public abstract void    setDirection(double setMultiplier);
    public abstract void    reset();
    public abstract void    update();
    public abstract double  getMultiplier();
    public abstract double  getDeltaPosition();

}
