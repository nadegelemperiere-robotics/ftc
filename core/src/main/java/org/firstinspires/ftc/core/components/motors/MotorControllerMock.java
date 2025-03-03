/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Controller managing mock servos
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.motors;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class MotorControllerMock implements MotorControllerComponent {

    final LogManager            mLogger;

    final boolean               mConfigurationValid;



    /* -------------- Constructors --------------- */
    public MotorControllerMock( LogManager logger)
    {
        mConfigurationValid = true;
        mLogger             = logger;
    }

    /* --------------------- Custom functions ---------------------- */

    /**
     * Determines if the mock servo controller component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    /* ------------------ HardwareDevice functions ----------------- */

    @Override
    public Manufacturer                 getManufacturer() { return Manufacturer.Unknown; }
    @Override
    public String                       getDeviceName() { return "MotorController"; }
    @Override
    public String                       getConnectionInfo() { return ""; }
    @Override
    public int                          getVersion() { return -1; }
    @Override
    public void                         resetDeviceConfigurationForOpMode() {}
    @Override
    public void                         close(){}


    /* ---------------- DcMotorController functions ---------------- */

    @Override
    public MotorConfigurationType       getMotorType(int motor) { return null; }
    @Override
    public DcMotor.RunMode	            getMotorMode(int motor) { return DcMotor.RunMode.RUN_WITHOUT_ENCODER; }
    @Override
    public double	                    getMotorPower(int motor) { return 0; }
    @Override
    public boolean	                    isBusy(int motor) { return false; }
    @Override
    public DcMotor.ZeroPowerBehavior	getMotorZeroPowerBehavior(int motor) { return DcMotor.ZeroPowerBehavior.UNKNOWN; }
    @Override
    public boolean	                    getMotorPowerFloat(int motor) { return false; }
    @Override
    public int                          getMotorTargetPosition(int motor) { return 0; }
    @Override
    public int	                        getMotorCurrentPosition(int motor) { return 0; }

    @Override
    public void                         setMotorType(int motor, MotorConfigurationType motorType) {}
    @Override
    public void	                        setMotorMode(int motor, DcMotor.RunMode mode) {}
    @Override
    public void	                        setMotorPower(int motor, double power) {}
    @Override
    public void	                        setMotorZeroPowerBehavior(int motor, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {}
    @Override
    public void	                        setMotorTargetPosition(int motor, int position) {}
    @Override
    public void	                        resetDeviceConfigurationForOpMode(int motor) {}

}
