/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   ServoComponent is an interface for servo management
   It supersedes Servo and provides additional capabilities
   such as :
   - Synchronizing 2 coupled servos
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.voltage;

/* JSON includes */
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

public interface VoltageSensorComponent extends Configurable, VoltageSensor {

    String sHwMapKey      = "hwmap";

    static VoltageSensorComponent factory(String name, JSONObject reader, HardwareMap map, LogManager logger) {

        VoltageSensorComponent result = null;

        switch (name) {
            case VoltageSensorBuiltIn.sTypeKey:
                result = new VoltageSensorBuiltIn(name, map, logger);
                result.read(reader);
                break;
            case VoltageSensorMock.sTypeKey:
                result = new VoltageSensorMock(name, logger);
                result.read(reader);
                break;
        }

        return result;

    }


    /* --------------------- Custom functions ---------------------- */

    String                      getName();
    String                      log();

    /* ------------------ Configurable functions ------------------- */

    void                        read(JSONObject reader);
    void                        write(JSONObject writer);
    boolean                     isConfigured();
    String                      logConfigurationHTML();
    String                      logConfigurationText(String header);

    /* ------------------ HardwareDevice functions ----------------- */

    Manufacturer                getManufacturer();
    String                      getDeviceName();
    String                      getConnectionInfo();
    int                         getVersion();
    void                        resetDeviceConfigurationForOpMode();
    void                        close();

    /* ------------------- Voltage Sensor methods override ------------------ */

    double                      getVoltage();

}
