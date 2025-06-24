/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Configuration management test class
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.test.tools;

/* Junit 5 includes */
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/* Mockito includes */
import org.mockito.junit.jupiter.MockitoExtension;

/* Component Under Test includes */
import org.firstinspires.ftc.core.tools.Calibration;

@ExtendWith(MockitoExtension.class)
public class CalibrationTest {

    private Calibration       mCalibration;

    static {
        System.load("/usr/local/share/java/opencv4/libopencv_java4110.dylib");
    }

    @Test
    public void evaluateOnReferencePoints() {

        mCalibration = new Calibration();
        mCalibration.initialize();

        float [] result = mCalibration.computeGroundPosition(160,57);

        assertTrue(Math.abs(result[0])<0.1);
        assertTrue(Math.abs(result[1] - 4)<0.1);
    }
}


