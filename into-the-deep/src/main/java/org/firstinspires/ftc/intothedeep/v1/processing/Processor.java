/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Robot subsystem interface
   ------------------------------------------------------- */
package org.firstinspires.ftc.intothedeep.v1.processing;

/* Json includes */

import org.firstinspires.ftc.core.robot.Hardware;
import org.firstinspires.ftc.core.tools.LogManager;
import org.json.JSONException;
import org.json.JSONObject;

public interface Processor extends org.firstinspires.ftc.core.processing.Processor {

    static org.firstinspires.ftc.core.processing.Processor   factory(String name, JSONObject reader, Hardware hardware, LogManager logger) {

        org.firstinspires.ftc.core.processing.Processor result;

        result = org.firstinspires.ftc.core.processing.Processor.factory(name, reader, hardware, logger);
        if(result == null) {
            try {
                if (reader.has(sTypeKey)) {
                    String type = reader.getString(sTypeKey);
                    switch (type) {
                        case SamplesDetection.sTypeValue :
                            result = new SamplesDetection(name, hardware, logger);
                            result.read(reader);
                            break;
                        case SamplesDetectionMock.sTypeValue :
                            result = new SamplesDetectionMock(name, logger);
                            result.read(reader);
                            break;
                    }
                }
            } catch (JSONException e) { logger.error(e.getMessage()); }
        }
        return result;
    }
}