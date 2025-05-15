/**
 * -------------------------------------------------------
 * Copyright (c) 2025 Nadege LEMPERIERE
 * All rights reserved
 * -------------------------------------------------------
 * CameraComponent Interface
 * -------------------------------------------------------
 * The CameraComponent interface provides centralized
 * initialization and configuration management for cameras
 * in FTC robots. It facilitates the handling of camera
 * components, allowing for easy configuration using JSON
 * input and integration with the FTC Control Hub.
 * -------------------------------------------------------
 * Features:
 * - Factory method to create and configure camera
 *   components (e.g., Limelight, Default, or Mock cameras).
 * - Provides methods for video stream display in the
 *   dashboard.
 * - Manages configuration states and logs configuration
 *   details in HTML or text format.
 * - Supports retrieving the current camera frame as an
 *   OpenCV Mat object.
 * -------------------------------------------------------
 */

package org.firstinspires.ftc.core.components.cameras;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* OpenCV includes */
import org.opencv.core.Mat;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;


public interface CameraComponent extends Configurable {

    String sTypeKey = "type";

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
            case CameraLimelight.sTypeValue:
                result = new CameraLimelight(name, map, logger);
                result.read(reader);
                break;
            case CameraDefault.sTypeValue:
                result = new CameraDefault(name, map, logger);
                result.read(reader);
                break;
            case CameraMock.sTypeValue:
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
    Limelight3A                 limelight();


}