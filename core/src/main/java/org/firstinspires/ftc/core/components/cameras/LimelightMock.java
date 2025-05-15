/**
 * -------------------------------------------------------
 * Copyright (c) 2025 Nadege LEMPERIERE
 * All rights reserved
 * -------------------------------------------------------
 * LimelightMock Class
 * -------------------------------------------------------
 * A mock limelight object, to enable robot logic to be
 * tested with mocked component without the vision processor
 * update failing
 * -------------------------------------------------------
 * Features:
 * - Simulates limelight behavior for testing purposes.
 * -------------------------------------------------------
 */
package org.firstinspires.ftc.core.components.cameras;

/* System includes */
import java.net.InetAddress;
import java.net.UnknownHostException;

/* Qualcomm includes */
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.internal.usb.EmbeddedSerialNumber;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class LimelightMock  extends Limelight3A {

    final LogManager mLogger;

    public LimelightMock(LogManager logger) throws UnknownHostException {
        super(EmbeddedSerialNumber.fromString("eth0.172.29.0.1"),"liemlight", InetAddress.getLocalHost());
        mLogger             = logger;
    }

    @Override
    public boolean                      pipelineSwitch(int id) { return true;}
    @Override
    public void                         start() { }
    @Override
    public boolean                      updatePythonInputs( double[] data) { return true; }
    @Override
    public LLResult                     getLatestResult() { return null; }



}
