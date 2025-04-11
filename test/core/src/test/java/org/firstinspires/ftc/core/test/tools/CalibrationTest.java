/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Configuration management test class
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.test.tools;

/* Junit 5 includes */
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/* Mockito includes */
import org.mockito.junit.jupiter.MockitoExtension;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.Gamepad;

/* Component includes */
import org.firstinspires.ftc.core.components.controllers.Controller;

/* Component Under Test includes */
import org.firstinspires.ftc.core.tools.Calibration;

@ExtendWith(MockitoExtension.class)
public class CalibrationTest {

    private Calibration       mCalibration;

    @Test
    public void evaluateOnReferencePoints() {

        mCalibration = new Calibration();
        mCalibration.initialize();

        float [] result = mCalibration.computeGroundPosition(160,57);

        assertEquals(result[0],0);
        assertEquals(result[1],4);
    }
}


