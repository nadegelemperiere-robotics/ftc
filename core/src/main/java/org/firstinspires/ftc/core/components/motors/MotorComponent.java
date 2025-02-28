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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

public interface MotorComponent extends Configurable {

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

    String                      name();
    void                        log();
    boolean                     encoderCorrection();
    void                        encoderCorrection(boolean value);
    EncoderComponent            encoder();

    /* ------------------ Configurable functions ------------------- */

    void                        read(JSONObject reader);
    void                        write(JSONObject writer);
    boolean                     isConfigured();
    String                      logConfigurationHTML();
    String                      logConfigurationText(String header);

    /* --------------------- DcMotor functions --------------------- */

    boolean	                    isBusy();

    int	                        currentPosition();
    DcMotor.RunMode	            mode();
    int	                        targetPosition();
    DcMotorSimple.Direction     direction();
    DcMotor.ZeroPowerBehavior	zeroPowerBehavior();
    double                      power();

    void	                    mode(DcMotor.RunMode mode);
    void	                    direction(DcMotorSimple.Direction direction);
    void	                    targetPosition(int position);
    void	                    zeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior);
    void                        power(double power);

    /* -------------------- DcMotorEx functions -------------------- */

    void                        PIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients);
    PIDFCoefficients            PIDFCoefficients(DcMotor.RunMode mode);
    void                        targetPositionTolerance(int tolerance);
    int                         targetPositionTolerance();
    double                      velocity();
    void                        achieveableMaxRPMFraction(double value);


}
