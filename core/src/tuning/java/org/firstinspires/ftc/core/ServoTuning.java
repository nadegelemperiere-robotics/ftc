/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Servo tuning tool
   ------------------------------------------------------- */

package org.firstinspires.ftc.core;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.List;
import java.util.ArrayList;
import java.util.Objects;

/* Android includes */
import android.os.Environment;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/* ACME robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

/* Components includes */
import org.firstinspires.ftc.core.components.servos.ServoComponent;
import org.firstinspires.ftc.core.components.controllers.Controller;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Tuning;

@Config
@TeleOp(name = "ServoTuning", group = "Tuning")
public class ServoTuning extends LinearOpMode {

    public enum Mode {
        FIRST,
        SECOND,
        BOTH
    }

    /* -------- Configuration variables -------- */
    public static double                INCREMENT_STEP  = 0.01;
    public static long                  SLEEP_MS        = 200;
    public static double                TARGET_POS      = 0.0;
    public static boolean               HOLD_POSITION   = false;
    public static String                CONFIGURATION   = "test";

    /* ---------------- Members ---------------- */
    private LogManager                  mLogger;

    private Configuration               mConfiguration;
    private String                      mConfigurationName;
    private Tuning                      mHardware;

    private ModeProvider                mMode;

    private Controller                  mController;


    /* ------- Preload for all servo data  ------ */

    // Each servos are manipulated as single servos, to enable servos couple to be moved
    // independantly while tuning their positions.

    // The servo selection config variables that can be updated by the dashboard
    private Map<String, Boolean>                mServoSelection;
    // Link between servo name and the associated harwareMap names (2 for coupled servos)
    private Map<String, List<String>>           mServosHw;
    private String                              mCurrentServo;

    // Link between hardwareMap name and the corresponding single servos
    private Map<String, ServoComponent>         mServos;

    // Current servos hw component for easy access
    private List<ServoComponent>                mCurrentServoHw;



    @Override
    public void runOpMode() {

        try {

            mLogger = new LogManager(null,FtcDashboard.getInstance(),"servo-tuning");
            mLogger.level(LogManager.Severity.TRACE);

            mController = new Controller(gamepad1, mLogger);

            mHardware = new Tuning(hardwareMap, mLogger);

            mMode = new ModeProvider();
            mMode.set(Mode.FIRST);

            mConfigurationName = CONFIGURATION;
            mConfiguration = new Configuration(mLogger);
            mConfiguration.register("robot.hardware", mHardware);
            mConfiguration.read(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/" + mConfigurationName + ".json");
            mConfiguration.log();

            mServosHw = mHardware.mappingServos();
            mServos = mHardware.singleServos();


            // Single and coupled motors
            Map<String, ServoComponent> servos = mHardware.servos();

            mCurrentServoHw = new ArrayList<>();
            mServoSelection = new LinkedHashMap<>();
            for (Map.Entry<String, ServoComponent> servo : servos.entrySet()) {
                mServoSelection.put(servo.getKey(),false);
            }

            // Add the servo selection variables on the dashboard
            for (Map.Entry<String, Boolean> selected : mServoSelection.entrySet()) {
                SelectedProvider provider = new SelectedProvider(mServoSelection, selected.getKey());
                FtcDashboard.getInstance().addConfigVariable(ServoTuning.class.getSimpleName(),selected.getKey(),provider);
            }

            FtcDashboard.getInstance().updateConfig();
            mLogger.update();

            waitForStart();

            mLogger.clear();

            while(opModeIsActive()) {

                /* Find current selected servo */
                String currentServo = this.findSelectedServo();
                mLogger.metric("Current Servo",currentServo);

                /* Manage configuration change */
                if(!Objects.equals(currentServo, mCurrentServo))  {

                    // Stop servos if they don't need to hold
                    if (!HOLD_POSITION) { this.stopServos(); }

                    // Now we can forget the previously selected servos sice we no longer need them
                    mCurrentServo = currentServo;
                    // Select hardwareservos and conf for current servo
                    this.updateCurrentServos();
                    
                    // Initialize servo processing
                    mMode.set(Mode.FIRST);

                    // Adapt configuration
                    if(!mCurrentServoHw.isEmpty()) {
                        ReverseProvider reverse = new ReverseProvider(mCurrentServoHw.get(0));
                        FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"REVERSE_1",reverse);
                    }
                    else {
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"REVERSE_1");
                    }
                    if(mCurrentServoHw.size() >= 2) {
                        ReverseProvider reverse = new ReverseProvider(mCurrentServoHw.get(1));
                        FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"REVERSE_2",reverse);

                        FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"MODE",mMode);
                    }
                    else {
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"REVERSE_2");
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"MODE");
                    }

                    // Start servos
                    this.startServos();
                    TARGET_POS = this.getPosition();

                    FtcDashboard.getInstance().updateConfig();

                }

                // Manage controls
                if (mController.buttons.left_bumper.pressedOnce()) {
                    TARGET_POS = Math.max(0.00, TARGET_POS - INCREMENT_STEP); // Decrease position but don't go below 0
                    FtcDashboard.getInstance().updateConfig();
                } else if (mController.buttons.right_bumper.pressedOnce()) {
                    TARGET_POS = Math.min(1.00, TARGET_POS + INCREMENT_STEP); // Increase position but don't exceed 1
                    FtcDashboard.getInstance().updateConfig();
                }

                this.setPosition(TARGET_POS);

                // Log servos state and updated configuration
                mLogger.metric("Mode",""+mMode.get());
                this.logServosState(mLogger);
                mConfiguration.log();

                mLogger.update();

                // Give time for servo position change to occur
                sleep(SLEEP_MS);


            }

            mConfiguration.write(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/servo-tuning.json");
            mLogger.info("Updated configuration saved. You may retrieve it using <b>adb pull /sdcard/FIRST/servo-tuning.json</b>");
            mLogger.update();
        }
        catch(Exception e) {
            mLogger.error(e.toString());
            mLogger.update();
        }
    }

    private String findSelectedServo()
    {
        String result = "";
        for (Map.Entry<String, Boolean> selected : mServoSelection.entrySet()) {
            if(selected.getValue()) { result = selected.getKey(); }
        }
        return result;
    }

    private void   updateCurrentServos() {

        mCurrentServoHw.clear();

        if (mServosHw.containsKey(mCurrentServo)) {
            // Retrieve the list of hardware servo associated to current selected servo
            // (may be coupled, so there may be up to 2 of them)
            List<String> hwServos = mServosHw.get(mCurrentServo);
            if(hwServos != null) {
                for (int i_servo = 0; i_servo < hwServos.size(); i_servo++) {
                    String servoHwName = hwServos.get(i_servo);

                    // Get current servo and its associated configuration
                    // which might have been changed through the dashboard
                    ServoComponent servo = null;

                    // Find the single servo associated to the hardware name
                    if (mServos.containsKey(servoHwName)) {
                        servo = mServos.get(servoHwName);
                    }

                    if (servo != null) {
                        mCurrentServoHw.add(servo);
                    }
                }
            }
        }
    }

    private void stopServos() {
        for (int i_servo = 0; i_servo < mCurrentServoHw.size(); i_servo++) {
            ServoComponent hwServo = mCurrentServoHw.get(i_servo);
            if (hwServo != null) {
                hwServo.getController().pwmDisable();
            }
        }
    }

    private void startServos()
    {
        for (int i_servo = 0; i_servo < mCurrentServoHw.size(); i_servo++) {
            ServoComponent hwServo = mCurrentServoHw.get(i_servo);
            if (hwServo != null) {
                hwServo.getController().pwmEnable();
            }
        }
    }


    private void setPosition(double position) {

        for (int i_servo = 0; i_servo < mCurrentServoHw.size(); i_servo++) {
            ServoComponent hwServo = mCurrentServoHw.get(i_servo);
            if (hwServo != null) {
                // Depending on the coupled servo management mode, pilot the required servos
                if(i_servo == 0) {
                    if (mMode.get() == Mode.FIRST || mMode.get() == Mode.BOTH){
                        hwServo.setPosition(position);
                    }
                    else{
                        hwServo.getController().pwmDisable();
                    }
                }
                if(i_servo == 1) {
                    if (mMode.get() == Mode.SECOND || mMode.get() == Mode.BOTH){
                        hwServo.setPosition(position);
                    }
                    else{
                        hwServo.getController().pwmDisable();
                    }
                }
            }
        }
    }

    private double getPosition()
    {
        double result = -1.0;

        for (int i_servo = 0; i_servo < mCurrentServoHw.size(); i_servo++) {
            ServoComponent hwServo = mCurrentServoHw.get(i_servo);
            if (hwServo != null) {

                // Depending on the coupled servo management mode, pilot the required servos
                if (i_servo == 0) {
                    if (mMode.get() == Mode.FIRST || mMode.get() == Mode.BOTH) {
                        result = hwServo.getPosition();
                    }
                }
                if (i_servo == 1) {
                    if (mMode.get() == Mode.SECOND || mMode.get() == Mode.BOTH) {
                        result = hwServo.getPosition();
                    }
                }
            }
        }

        return result;
    }

    private void logServosState(LogManager logger) {
        logger.info("CURRENT SERVOS");

        for (int i_servo = 0; i_servo < mCurrentServoHw.size(); i_servo++) {
            ServoComponent hwServo = mCurrentServoHw.get(i_servo);
            if (hwServo != null) {

                // Log servo state
                logger.info("--> Servo " + i_servo);
                logger.info("-----> HwMap : " + hwServo.getName());
                logger.info("-----> Direction : " + hwServo.getDirection());
                logger.info("-----> Position : " + hwServo.getPosition());
                logger.info("-----> Power : " + hwServo.getController().getPwmStatus());
            }
        }
    }




    // SelectedProvider updates the servos selection states
    // Since Map<String, Boolean> is not a simple type, it's managed as
    // pointer, when we change it in the provider, it's changed in the
    // global class.
    // When we select a new servo, we make sure to deselect all the others
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
    // Since ConfServo.Controller is not a simple type, it's managed as
    // pointer, when we change it in the provider, it's changed in the
    // global configuration
    static class ReverseProvider implements ValueProvider<Boolean> {
        ServoComponent mServo;
        public ReverseProvider( ServoComponent servo) {
            mServo = servo;
        }
        @Override
        public Boolean get()           { return mServo.getDirection() == Servo.Direction.REVERSE; }
        @Override
        public void set(Boolean Value) {
            if (Value) { mServo.setDirection(Servo.Direction.REVERSE); }
            else { mServo.setDirection(Servo.Direction.FORWARD);       }
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