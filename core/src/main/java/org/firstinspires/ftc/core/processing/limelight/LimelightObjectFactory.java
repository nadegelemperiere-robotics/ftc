/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Limelight I/O objects formatting factory
   ------------------------------------------------------- */
package org.firstinspires.ftc.core.processing.limelight;

/* Qualcomm includes */
import com.qualcomm.hardware.limelightvision.LLResultTypes;

public interface LimelightObjectFactory<T extends LimelightObject, U extends LimelightObject> {

    /**
     * Build LimelightObject from a neural network detector result
     * @param pipeline Name of the pipeline issuing the object
     * @param result The pipeline result
     * @return A valid LimelightObject to use as interface
     */
    T        build(String pipeline, LLResultTypes.DetectorResult result);

    /**
     * Format limelight object into a valid input for Limelight Snapscript pipeline
     * @param pipeline Name of the pipeline processing the object
     * @param input The object to format
     * @return An llrobot ready array
     */
    double[] format(String pipeline, T input);

    /**
     * Build LimelightObject from a python snapscript result
     * @param pipeline Name of the pipeline issuing the object
     * @param data The pipeline result (llpython)
     * @param index : The position to start reading from
     * @return A valid LimelightObject to use as interface
     */
    U        build(String pipeline, double[] data, int index);

    /**
     * return increment from one object to another in a python snapscript result
     * @param pipeline Name of the pipeline issuing the object
     * @return The number of data describing an object
     */
    int      increment(String pipeline);

}
