/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Shared data for interopmodes exchange
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.orchestration.engine;

/* System includes */
import java.util.Map;
import java.util.LinkedHashMap;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class InterOpMode {

    static InterOpMode sInstance = null;

    /**
     * Static function to access interops data from anywhere
     *
     * @return the current mock
     */
    public static InterOpMode instance() {
        if(sInstance == null) { sInstance = new InterOpMode(); }
        return sInstance;
    }

    final private Map<String,Object>  mData;

    /**
     * Private constructor : only Singleton can use it
     */
    private InterOpMode() {
        mData = new LinkedHashMap<>();
    }

    /**
     * Interopmodes data storage function
     *
     * @param key Key to store data under
     * @param data Data to store
     */
    public void                         add(String key, Object data) {
        mData.put(key,data);
    }

    /**
     * Interopmodes data retrieval function
     *
     * @param key Key to store data under
     * @return Data stored under key
     */
    public Object                        get(String key) {
        Object result = null;
        if(mData.containsKey(key)) { result = mData.get(key); }
        return result;
    }

    /**
     * Interopmodes data logging function
     *
     * @param logger Logger to use for mock data logging
     */
    public void                         log(LogManager logger) {
        for (Map.Entry<String, Object> values : mData.entrySet()) {
            logger.trace(values.getKey().toUpperCase() + " : " + values.getValue().toString());
        }
    }

    /**
     * Clear inter opmodes data
     */
    public void                         clear() { mData.clear(); }



}