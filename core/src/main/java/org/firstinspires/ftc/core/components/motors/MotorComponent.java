/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   MotorComponent is an interface for motor management
   It supersedes DcMotorEx and provides additional capabilities
   such as :
   - Correcting orientation error on encoder
   - Synchronizing 2 coupled motors
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.motors;

/* System includes */
import java.util.Map;

/* JSON includes */
import org.json.JSONObject;
import org.json.JSONArray;
import org.json.JSONException;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/* FTC controller includes */
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

/**
 * Motor component wrapper to handle motor mocking and motor coupling
 */
public interface MotorComponent extends Configurable, DcMotorEx {

    // Configuration parsng keys
    String sHwMapKey          = "hwmap";
    String sDirectionKey      = "direction";
    String sEncoderReverseKey = "encoder-reverse";

    Map<String, DcMotor.Direction> sString2Direction = Map.of(
            "reverse", DcMotor.Direction.REVERSE,
            "forward",DcMotor.Direction.FORWARD
    );

    Map<DcMotor.Direction, String> sDirection2String = Map.of(
            DcMotor.Direction.REVERSE,"reverse",
            DcMotor.Direction.FORWARD,"forward"
    );

    /**
     * Motor building factory.
     * @param name Name to give to the motor
     * @param reader JSON object to read configuration from
     * @param map Raw hardware map to retrieve motor info from
     * @param logger Logger
     * @return The created motor - null if creation failed.
     */
    static MotorComponent factory(String name, JSONArray reader, HardwareMap map, LogManager logger) {

        MotorComponent result = null;

        // Configure motor
        try {
            if (reader.length() == 0) {
                result = new MotorMock(name, logger);
            } else if (reader.length() == 1) {
                result = new MotorSingle(name, map, logger);
                result.read(reader.getJSONObject(0));
            } else if (reader.length() == 2) {
                JSONObject configuration = new JSONObject();
                configuration.put(MotorCoupled.sFirstKey, reader.getJSONObject(0));
                configuration.put(MotorCoupled.sSecondKey, reader.getJSONObject(1));
                result = new MotorCoupled(name, map, logger);
                result.read(configuration);
            } else {
                logger.error("Can not managed more than 3 coupled DcMotors");
            }
        } catch (JSONException e) { logger.error(e.getMessage()); }

        return result;

    }

    /* --------------------- Custom functions ---------------------- */

    String                      getName();
    void                        log();
    boolean                     getEncoderCorrection();
    void                        setEncoderCorrection(boolean value);
    EncoderComponent            getEncoder();
    void                        setAchieveableMaxRPMFraction(double rate);

    /* ------------------ Configurable functions ------------------- */

    void                        read(JSONObject reader);
    void                        write(JSONObject writer);
    boolean                     isConfigured();
    String                      logConfigurationHTML();
    String                      logConfigurationText(String header);

    /* ------------------ HardwareDevice functions ----------------- */

    Manufacturer                getManufacturer();
    String                      getDeviceName();
    String                      getConnectionInfo();
    int                         getVersion();
    void                        resetDeviceConfigurationForOpMode();
    void                        close();

    /* --------------------- DcMotor functions --------------------- */

    boolean	                    isBusy();

    int	                        getCurrentPosition();
    DcMotor.RunMode	            getMode();
    int	                        getTargetPosition();
    DcMotorSimple.Direction     getDirection();
    DcMotor.ZeroPowerBehavior	getZeroPowerBehavior();
    double                      getPower();
    boolean                     getPowerFloat();
    DcMotorController           getController();
    int                         getPortNumber();

    void	                    setMode(DcMotor.RunMode mode);
    void	                    setDirection(DcMotorSimple.Direction direction);
    void	                    setTargetPosition(int position);
    void	                    setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior);
    void                        setPower(double power);
    void                        setPowerFloat();

    /* -------------------- DcMotorEx functions -------------------- */

    double                      getCurrent(CurrentUnit unit);
    double                      getCurrentAlert(CurrentUnit unit);
    boolean                     isOverCurrent();
    PIDFCoefficients            getPIDFCoefficients(DcMotor.RunMode mode);
    PIDCoefficients             getPIDCoefficients(DcMotor.RunMode mode);
    int                         getTargetPositionTolerance();
    double                      getVelocity();
    double                      getVelocity(AngleUnit unit);
    boolean                     isMotorEnabled();

    void                        setCurrentAlert(double alert, CurrentUnit unit);
    void                        setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients);
    void                        setPIDCoefficients(DcMotor.RunMode mode, PIDCoefficients pidCoefficients);
    void                        setPositionPIDFCoefficients(double p);
    void                        setVelocityPIDFCoefficients(double p, double i, double d, double f);
    void                        setTargetPositionTolerance(int tolerance);
    void                        setVelocity(double ticks);
    void                        setVelocity(double angularRate, AngleUnit unit);
    void                        setMotorEnable();
    void                        setMotorDisable();


}
