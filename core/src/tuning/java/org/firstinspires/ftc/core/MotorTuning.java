/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Motor tuning tool
   ------------------------------------------------------- */

package org.firstinspires.ftc.core;

/* System includes */
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/* Android includes */
import android.os.Environment;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/* ACME robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

/* Components includes */
import org.firstinspires.ftc.core.components.motors.MotorComponent;
import org.firstinspires.ftc.core.components.controllers.Controller;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Tuning;


@Config
@TeleOp(name = "MotorTuning", group = "Tuning")
public class MotorTuning extends LinearOpMode{

    public enum Mode {
        FIRST,
        SECOND,
        BOTH
    }

    public enum Direction {
        REVERSE,
        FORWARD
    }

    /* -------- Configuration variables -------- */
    public static long                  SLEEP_MS        = 200;
    public static String                CONFIGURATION   = "test";
    public static int                   TARGET_POSITION = 0;

    /* ---------------- Members ---------------- */
    private LogManager                  mLogger;

    private Configuration               mConfiguration;
    private String                      mConfigurationName;
    private Tuning                      mHardware;

    private ModeProvider                mMode;
    private int                         mTargetPosition;


    private Controller                  mController;


    /* ------- Preload for all motor data  ------ */

    // Each motors are manipulated as single motors, to enable motors couple to be moved
    // independantly while tuning their positions.

    // The motor selection config variables that can be updated by the dashboard
    private Map<String, Boolean>                mMotorSelection;
    // Link between motor name and the associated harwareMap names (2 for coupled motors)
    private Map<String, List<String>>           mMotorsHw;
    private String                              mCurrentMotor;

    // Link between hardwareMap name and the corresponding single motors
    private Map<String, MotorComponent>         mMotors;

    // Current motors hw component for easy access
    private List<MotorComponent>                mCurrentMotorHw;



    @Override
    public void runOpMode() {

        try {

            mLogger = new LogManager(null,FtcDashboard.getInstance(),"motor-tuning");
            mLogger.level(LogManager.Severity.TRACE);

            mController = new Controller(gamepad1, mLogger);

            mHardware = new Tuning(hardwareMap, mLogger);

            mMode = new ModeProvider();
            mMode.set(Mode.FIRST);
            mTargetPosition = TARGET_POSITION;

            mConfigurationName = CONFIGURATION;
            mConfiguration = new Configuration(mLogger);
            mConfiguration.register("robot.hardware", mHardware);
            mConfiguration.read(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/" + mConfigurationName + ".json");
            mConfiguration.log();

            mMotorsHw = mHardware.mappingMotors();
            mMotors = mHardware.singleMotors();
            // Single and coupled motors
            Map<String,MotorComponent> motors = mHardware.motors();

            mCurrentMotorHw = new ArrayList<>();
            mMotorSelection = new LinkedHashMap<>();
            for (Map.Entry<String, MotorComponent> motor : motors.entrySet()) {
                mMotorSelection.put(motor.getKey(),false);
            }

            // Add the motor selection variables on the dashboard
            for (Map.Entry<String, Boolean> selected : mMotorSelection.entrySet()) {
                SelectedProvider provider = new SelectedProvider(mMotorSelection, selected.getKey());
                FtcDashboard.getInstance().addConfigVariable(MotorTuning.class.getSimpleName(),selected.getKey(),provider);
            }

            FtcDashboard.getInstance().updateConfig();
            mLogger.update();

            waitForStart();

            mLogger.clear();

            while(opModeIsActive()) {

                /* Find current selected motor */
                String currentMotor = this.findSelectedMotor();
                mLogger.metric("Current Motor",currentMotor);

                /* Manage configuration change */
                if(!Objects.equals(currentMotor, mCurrentMotor))  {

                    // Stop current motors
                    this.powerMotors(0);

                    // Now we can forget the previously selected motors sice we no longer need them
                    mCurrentMotor = currentMotor;
                    // Select hardwaremotors and conf for current motor
                    this.updateCurrentMotors();

                    // Initialize motor processing
                    mMode.set(Mode.FIRST);

                    // Adapt configuration
                    if(!mCurrentMotorHw.isEmpty()) {
                        ReverseProvider reverse = new ReverseProvider(mCurrentMotorHw.get(0));
                        FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"REVERSE_1",reverse);

                        DirectionProvider direction = new DirectionProvider(mCurrentMotorHw.get(0));
                        FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"DIRECTION_1",direction);


                    }
                    else {
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"REVERSE_1");
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"DIRECTION_1");
                    }
                    if(mCurrentMotorHw.size() >= 2) {

                        ReverseProvider reverse = new ReverseProvider(mCurrentMotorHw.get(1));
                        FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"REVERSE_2",reverse);

                        DirectionProvider direction = new DirectionProvider(mCurrentMotorHw.get(1));
                        FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"DIRECTION_2",direction);

                        FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"MODE",mMode);
                    }
                    else {
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"REVERSE_2");
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"DIRECTION_2");
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"MODE");
                    }

                    // Start motors

                    FtcDashboard.getInstance().updateConfig();

                }

                if(mTargetPosition != TARGET_POSITION) {
                    mTargetPosition = TARGET_POSITION;
                    this.reachTargetPosition();
                }

                // Manage controls
                this.powerMotors(mController.axes.left_stick_y.value());

                // Log motors state and updated configuration
                mLogger.metric("Mode",""+mMode.get());
                this.logMotorsState(mLogger);
                mConfiguration.log();

                mLogger.update();

                // Give time for motor position change to occur
                sleep(SLEEP_MS);


            }

            mConfiguration.write(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/motor-tuning.json");
            mLogger.info("Updated configuration saved. You may retrieve it using <b>adb pull /sdcard/FIRST/motor-tuning.json</b>");
            mLogger.update();
        }
        catch(Exception e) {
            mLogger.error(e.toString());
            mLogger.update();
        }
    }

    private String findSelectedMotor()
    {
        String result = "";
        for (Map.Entry<String, Boolean> selected : mMotorSelection.entrySet()) {
            if(selected.getValue()) { result = selected.getKey(); }
        }
        return result;
    }

    private void   updateCurrentMotors() {

        mCurrentMotorHw.clear();

        if (mMotorsHw.containsKey(mCurrentMotor)) {
            // Retrieve the list of hardware motor associated to current selected motor
            // (may be coupled, so there may be up to 2 of them)
            List<String> hwMotors = mMotorsHw.get(mCurrentMotor);
            if(hwMotors != null) {
                for (int i_motor = 0; i_motor < hwMotors.size(); i_motor++) {
                    String motorHwName = hwMotors.get(i_motor);

                    // Get current motor and its associated configuration
                    // which might have been changed through the dashboard
                    MotorComponent motor = null;

                    // Find the single motor associated to the hardware name
                    if (mMotors.containsKey(motorHwName)) {
                        motor = mMotors.get(motorHwName);
                    }

                    if (motor != null) {
                        mCurrentMotorHw.add(motor);
                    }
                }
            }
        }
    }


    private void powerMotors(double Value) {
        mLogger.metric("Power", ""+Value);
        for (int i_motor = 0; i_motor < mCurrentMotorHw.size(); i_motor++) {
            MotorComponent hwMotor = mCurrentMotorHw.get(i_motor);
            if (hwMotor != null && !hwMotor.isBusy()) {

                if (i_motor == 0) {
                    if (mMode.get() == Mode.FIRST || mMode.get() == Mode.BOTH) {
                        hwMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        hwMotor.setPower(Value);
                    }
                }
                if (i_motor == 1) {
                    if (mMode.get() == Mode.SECOND || mMode.get() == Mode.BOTH) {
                        hwMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        hwMotor.setPower(Value);
                    }
                }


            }
        }
    }


    void reachTargetPosition(){
        for (int i_motor = 0; i_motor < mCurrentMotorHw.size(); i_motor++) {
            MotorComponent hwMotor = mCurrentMotorHw.get(i_motor);
            if (hwMotor != null) {

                if (i_motor == 0) {
                    if (mMode.get() == Mode.FIRST || mMode.get() == Mode.BOTH) {
                        hwMotor.setTargetPosition(mTargetPosition);
                        hwMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hwMotor.setPower(1.0);
                    }
                }
                if (i_motor == 1) {
                    if (mMode.get() == Mode.SECOND || mMode.get() == Mode.BOTH) {
                        hwMotor.setTargetPosition(mTargetPosition);
                        hwMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hwMotor.setPower(1.0);
                    }
                }


            }
        }
    }


    private void logMotorsState(LogManager logger) {
        logger.info("CURRENT MOTORS");

        for (int i_motor = 0; i_motor < mCurrentMotorHw.size(); i_motor++) {
            MotorComponent hwMotor = mCurrentMotorHw.get(i_motor);
            if (hwMotor != null) {

                // Log motor state
                logger.info("--> Motor " + i_motor);
                logger.info("-----> HwMap : " + hwMotor.getName());
                logger.info("-----> Direction : " + hwMotor.getDirection());
                logger.info("-----> Position : " + hwMotor.getCurrentPosition());
                logger.info("-----> Target : " + hwMotor.getTargetPosition());
                logger.info("-----> Power : " + hwMotor.getPower());
                logger.info("-----> Mode : " + hwMotor.getMode());
                logger.info("-----> Busy : " + hwMotor.isBusy());
            }
        }
    }



    // SelectedProvider updates the motors selection states
    // Since Map<String, Boolean> is not a simple type, it's managed as
    // pointer, when we change it in the provider, it's changed in the
    // global class.
    // When we select a new motor, we make sure to deselect all the others
    static class SelectedProvider implements ValueProvider<Boolean> {
        Map<String, Boolean> mAllSelection;
        String mCurrentSelection;

        public SelectedProvider(Map<String, Boolean> selection, String current) {
            mAllSelection = selection;
            mCurrentSelection = current;
        }

        @Override
        public Boolean get() {
            return mAllSelection.get(mCurrentSelection);
        }

        @Override
        public void set(Boolean Value) {

            if (Value) {
                for (Map.Entry<String, Boolean> selected : mAllSelection.entrySet()) {
                    selected.setValue(false);
                }
            }
            mAllSelection.put(mCurrentSelection, Value);
        }
    }


    // ReverseProvider updates the controller reverse configuration
    // Since ConfMotor.Controller is not a simple type, it's managed as
    // pointer, when we change it in the provider, it's changed in the
    // global configuration
    static class ReverseProvider implements ValueProvider<Boolean> {
        MotorComponent mMotor;
        public ReverseProvider( MotorComponent motor) {
            mMotor = motor;
        }
        @Override
        public Boolean get()           { return mMotor.getEncoderCorrection(); }
        @Override
        public void set(Boolean Value) { mMotor.setEncoderCorrection(Value);   }
    }

    // DirectionProvider updates the controller reverse configuration
    // Since ConfMotor.Controller is not a simple type, it's managed as
    // pointer, when we change it in the provider, it's changed in the
    // global configuration
    static class DirectionProvider implements ValueProvider<Direction> {
        MotorComponent mMotor;
        public DirectionProvider( MotorComponent motor) {
            mMotor = motor;
        }
        @Override
        public Direction get()           {
            Direction result = Direction.FORWARD;

            DcMotor.Direction direction = mMotor.getDirection();
            if(direction == DcMotor.Direction.REVERSE) { result = Direction.REVERSE; }

            return result;
        }
        @Override
        public void set(Direction Value) {
            if (Value == Direction.REVERSE) { mMotor.setDirection(DcMotor.Direction.REVERSE); }
            if (Value == Direction.FORWARD) { mMotor.setDirection(DcMotor.Direction.FORWARD); }
        }
    }


    // Since mode is a simple type, even with the appropriate constructor,
    // mMode will only be updated locally by the dashboard.
    // We'll have to make sure the code access the mode from the provider using the get
    // Method, since it's the only place the pdated information can be found
    static class ModeProvider implements ValueProvider<Mode> {
        Mode mMode;
        @Override
        public Mode get()              { return mMode;  }
        @Override
        public void set(Mode Value)    { mMode = Value; }
    }



}