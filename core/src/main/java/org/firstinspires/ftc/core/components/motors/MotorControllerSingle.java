/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Controller managing  single component servo
   ------------------------------------------------------- */
package org.firstinspires.ftc.core.components.motors;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class MotorControllerSingle implements MotorControllerComponent {

    final LogManager        mLogger;

    boolean                 mConfigurationValid;

    final String            mName;

    final DcMotorController mController;

    /* -------------- Constructors --------------- */
    public MotorControllerSingle(DcMotorController controller, String name, LogManager logger)
    {
        mConfigurationValid      = true;

        mLogger     = logger;

        mName       = name;

        mController = controller;

        if(mController == null) { mConfigurationValid = false; }
    }

    /* --------------------- Custom functions ---------------------- */

    /**
     * Determines if the servo controller component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    /* ------------------ HardwareDevice functions ----------------- */

    @Override
    public Manufacturer                 getManufacturer() {
        Manufacturer result = Manufacturer.Unknown;
        if(mConfigurationValid) {
            result = mController.getManufacturer();
        }
        return result;
    }

    @Override
    public String                       getDeviceName() {
        String result = "";
        if(mConfigurationValid) {
            result = mController.getDeviceName();
        }
        return result;
    }

    @Override
    public String                       getConnectionInfo() {
        String result = "";
        if(mConfigurationValid) {
            result = mController.getConnectionInfo();
        }
        return result;
    }

    @Override
    public int                          getVersion() {
        int result = -1;
        if(mConfigurationValid) {
            result = mController.getVersion();
        }
        return result;
    }

    @Override
    public void                         resetDeviceConfigurationForOpMode() {
        if(mConfigurationValid) {
            mController.resetDeviceConfigurationForOpMode();
        }
    }

    /**
     * Closes this device
     */
    @Override
    public void                         close(){
        if(mConfigurationValid) {
            mController.close();
        }
    }

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
