/**
 * -------------------------------------------------------
 * Copyright (c) 2025 Nadege LEMPERIERE
 * All rights reserved
 * -------------------------------------------------------
 * Processor provides smart function to support robot control
 * -------------------------------------------------------
 */
package org.firstinspires.ftc.core.processing;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;
import org.opencv.core.Mat;

public interface Processor {

    final String sTypeKey = "type";

    /**
     * Factory method to create and configure an CameraComponent from a JSON object.
     *
     * @param name The name of the camera component.
     * @param reader A JSON object containing the configuration parameters.
     * @param hardware The FTC Hardware to retrieve the sensors.
     * @param logger The logging manager to handle system logs.
     * @return A fully configured CameraComponent instance.
     */
    static Processor factory(String name, JSONObject reader, Hardware hardware, LogManager logger) {

        Processor result = null;

        String type = "";
        if (reader.has(sTypeKey)) {
            try {
                type = reader.getString(sTypeKey);
            } catch (JSONException ignored) {}
        }

        switch (type) {
        }

        return result;
    }

    /* --------------------- Custom functions ---------------------- */

    String                      name();
    void                        update();
    void                        log(String header);

    Mat                         draw(Mat raw);

    /* ------------------ Configurable functions ------------------- */

    void                        read(JSONObject reader);
    void                        write(JSONObject writer);
    boolean                     isConfigured();
    String                      logConfigurationHTML();
    String                      logConfigurationText(String header);


}
