/**
 * -------------------------------------------------------
 * Copyright (c) 2025 Nadege LEMPERIERE
 * All rights reserved
 * -------------------------------------------------------
 * CamerComponent provides centralized initialization and
 * configuration management for the cameras in FTC robots.
 * -------------------------------------------------------
 * This class facilitates the initialization and handling
 * of the camera component within the FTC Control Hub,
 * allowing for easy configuration using JSON input. It
 * provides methods for video stream display in dashboard,
 * managing configuration states, and logging system
 * parameters.
 * <p>
 * Features:
 * <p>
 */

package org.firstinspires.ftc.core.components.cameras;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* OpenCV includes */
import org.opencv.core.Mat;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;


public interface CameraComponent extends Configurable {

    static final String sTypeKey = "type";

    /**
     * Factory method to create and configure an CameraComponent from a JSON object.
     *
     * @param name The name of the camera component.
     * @param reader A JSON object containing the configuration parameters.
     * @param map The FTC HardwareMap to retrieve the camera hardware.
     * @param logger The logging manager to handle system logs.
     * @return A fully configured CameraComponent instance.
     */
    static CameraComponent factory(String name, JSONObject reader, HardwareMap map, LogManager logger) {

        CameraComponent result = null;

        String type = "";
        if (reader.has(sTypeKey)) {
            try {
                type = reader.getString(sTypeKey);
            } catch (JSONException ignored) {}
        }

        switch (type) {
            case CameraLimelight.sTypeKey:
                result = new CameraLimelight(name, map, logger);
                result.read(reader);
                break;
            case CameraDefault.sTypeKey:
                result = new CameraDefault(name, map, logger);
                result.read(reader);
                break;
            case CameraMock.sTypeKey:
                result = new CameraMock(name, logger);
                result.read(reader);
                break;
        }

        return result;
    }

    /* --------------------- Custom functions ---------------------- */

    String                      name();
    void                        update();

    /* ------------------ Configurable functions ------------------- */

    void                        read(JSONObject reader);
    void                        write(JSONObject writer);
    boolean                     isConfigured();
    String                      logConfigurationHTML();
    String                      logConfigurationText(String header);

    /* ---------------------- Cameras function --------------------- */

    Mat                         current();


}