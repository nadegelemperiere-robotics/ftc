/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Hardware manager
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.tuning;

/* System includes */
import java.util.Map;

/* Json includes */
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Processors includes */
import org.firstinspires.ftc.core.processing.Processor;

/* Subsystems includes */
import org.firstinspires.ftc.core.subsystems.Subsystem;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

/* Tuning includes */
import org.firstinspires.ftc.core.tuning.Tuning;

public class Robot extends org.firstinspires.ftc.intothedeep.v1.robot.Robot {

    /**
     * Constructor
     *
     * @param asker Tuning object to make usre this is only called from a Tuning component
     * @param map hardware map to retrieve hardware from
     * @param logger logger to use for trace
     */
    public Robot(Tuning asker, HardwareMap map, LogManager logger) {
        super(map, logger);
    }

    /**
     * Read configuration from file, but consider it valid even if missing subsystems
     * @param reader The JSON object containing configuration settings.
     */
    public void                         read(JSONObject reader) {
        super.read(reader);
        mConfigurationValid = mHardware.isConfigured();
        for (Map.Entry<String, Subsystem> subsystem : mSubsystems.entrySet()) {
            if (subsystem.getValue() == null || !subsystem.getValue().isConfigured()) {
                mConfigurationValid = false;
            }
        }
        for (Map.Entry<String, Processor> processor : mProcessors.entrySet()) {
            if (processor.getValue() == null || !processor.getValue().isConfigured()) {
                mConfigurationValid = false;
            }
        }
    }

    public Subsystem                    subsystem(Tuning asker, String name) {
        Subsystem result = null;
        if(mConfigurationValid && mSubsystems.containsKey(name)) {
            result = mSubsystems.get(name);
        }
        return result;
    }

    public Processor                    processor(Tuning asker, String name) {
        Processor result = null;
        if(mConfigurationValid && mProcessors.containsKey(name)) {
            result = mProcessors.get(name);
        }
        return result;
    }

    public Hardware                      hardware(Tuning asker) {
        Hardware result = null;
        if(mConfigurationValid) {
            result = mHardware;
        }
        return result;
    }

}