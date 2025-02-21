/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   InPerTick tuning tool - common to many odometers tuning
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.tuning;

/* System includes */
import java.util.List;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Components includes */
import org.firstinspires.ftc.core.components.odometers.Encoder;


public class InPerTick {

    /* ---------------- Members ---------------- */
    private List<Encoder>       mEncoders;

    private double              mTicks;
    private double              mInitialTicks;

    public InPerTick(List<Encoder> encoders)
    {
        mEncoders   = encoders;
        mTicks = 0;
    }

    public void init() {
        mTicks = 0;
        mInitialTicks = 0;

        for(int i_encoder = 0; i_encoder < mEncoders.size(); i_encoder ++){
            mInitialTicks += mEncoders.get(i_encoder).update().rawPosition;
        }
        mInitialTicks /= mEncoders.size();
    }

    public void update() {
        double current = 0;

        for(int i_encoder = 0; i_encoder < mEncoders.size(); i_encoder ++){
            current += mEncoders.get(i_encoder).update().rawPosition;
        }
        current /= mEncoders.size();

        mTicks = current - mInitialTicks;
    }

    public double ticks() { return mTicks; }


}