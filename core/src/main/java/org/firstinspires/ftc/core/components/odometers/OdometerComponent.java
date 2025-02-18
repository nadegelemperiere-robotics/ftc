/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localization interface providing robot precise position
   and orientation on the mat
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.odometers;

/* JSON includes */
import org.json.JSONObject;

/* Qualcomm includes */
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

public interface OdometerComponent extends Configurable {

    static OdometerComponent factory(String name, JSONObject reader, HardwareMap map, LogManager logger) {

        OdometerComponent result = null;

        switch (name) {
            case "otos":
                result = new OpticalTrackingOdometer(name, map, logger);
                result.read(reader);
                break;
            case "2deadwheels":
                result = new TwoDeadWheelsOdometer(name, map, logger);
                result.read(reader); break;
            case "3deadwheels":
                result = new ThreeDeadWheelsOdometer(name, map, logger);
                result.read(reader); break;
            case "driveencoders":
                result = new DriveEncodersOdometer(name, map, logger);
                result.read(reader);
                break;
            case "pinpoint":
                result = new PinPointOdometer(name, map, logger);
                result.read(reader);
                break;
            case "mock":
                result = new MockOdometer(name, logger);
                result.read(reader);
                break;
        }

        return result;

    }

    /* ------------------------- Accessors ------------------------- */
    void            pose(Pose2d current);
    Pose2d          pose();
    PoseVelocity2d  velocity();
    void            log();

    /* ------------------ Configurable functions ------------------- */

    void            read(JSONObject reader);
    void            write(JSONObject writer);
    boolean         isConfigured();
    String          logConfigurationHTML();
    String          logConfigurationText(String header);

    /* ------------------------ Localization ----------------------- */
    void            update();

}