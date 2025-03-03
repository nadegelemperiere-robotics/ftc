/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   MotorComponent is an interface for motor controllers management
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.motors;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


public interface MotorControllerComponent extends DcMotorController {

    /* --------------------- Custom functions ---------------------- */

    boolean                     isConfigured();

    /* ------------------ HardwareDevice functions ----------------- */

    Manufacturer                getManufacturer();
    String                      getDeviceName();
    String                      getConnectionInfo();
    int                         getVersion();
    void                        resetDeviceConfigurationForOpMode();
    void                        close();

    /* ------------- DcMotorController methods override ------------ */

    MotorConfigurationType	    getMotorType(int motor);
    DcMotor.RunMode	            getMotorMode(int motor);
    double	                    getMotorPower(int motor);
    boolean	                    isBusy(int motor);
    DcMotor.ZeroPowerBehavior	getMotorZeroPowerBehavior(int motor);
    boolean	                    getMotorPowerFloat(int motor);
    int                         getMotorTargetPosition(int motor);
    int	                        getMotorCurrentPosition(int motor);

    void                        setMotorType(int motor, MotorConfigurationType motorType);
    void	                    setMotorMode(int motor, DcMotor.RunMode mode);
    void	                    setMotorPower(int motor, double power);
    void	                    setMotorZeroPowerBehavior(int motor, DcMotor.ZeroPowerBehavior zeroPowerBehavior);
    void	                    setMotorTargetPosition(int motor, int position);
    void	                    resetDeviceConfigurationForOpMode(int motor);



}
