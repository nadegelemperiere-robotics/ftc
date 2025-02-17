/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   State context for state machine
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.orchestration.engine;

/* System includes */
import java.util.Map;
import java.util.LinkedHashMap;

/* Tools includes */
import org.firstinspires.ftc.core.components.servos.ServoComponent;
import org.firstinspires.ftc.core.tools.LogManager;

public class Mock {

    static Mock sInstance = null;

    /**
     * Static function to access mock from anywhere
     *
     * @return the current mock
     */
    public static Mock instance() {
        if(sInstance == null) { sInstance = new Mock(); }
        return sInstance;
    }

    public Map<String,Double>  mMockedComponents;

    private Mock() {
        mMockedComponents = new LinkedHashMap<>();
    }

    public void                         log(LogManager logger) {
        for (Map.Entry<String, Double> values : mMockedComponents.entrySet()) {
            logger.trace(values.getKey().toUpperCase() + " : " + values.getValue());
        }
    }

    public void                         clear() { mMockedComponents.clear(); }

}