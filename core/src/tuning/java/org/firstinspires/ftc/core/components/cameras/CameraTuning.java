/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   camera tuning tool
   ------------------------------------------------------- */

package components.cameras;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

/* Android includes */
import android.graphics.Bitmap;
import android.os.Environment;

/* Acmerobotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* OpenCV includes */
import org.opencv.android.Utils;
import org.opencv.core.Mat;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

/* Components includes */
import org.firstinspires.ftc.core.components.cameras.CameraComponent;

/* Tuning includes */
import org.firstinspires.ftc.core.tuning.Hardware;
import org.firstinspires.ftc.core.tuning.Tuning;

@Config
@TeleOp(name = "CameraTuning", group = "Tuning")
public class CameraTuning extends LinearOpMode implements Tuning {

    /* ---------------- Members ---------------- */
    private LogManager                  mLogger;

    private Configuration               mConfiguration;
    private Hardware                    mHardware;


    /* ------- Preload for all camera data  ------ */

    // Each cameras are manipulated as single cameras, to enable cameras couple to be moved
    // independantly while tuning their positions.

    // The camera selection config variables that can be updated by the dashboard
    private Map<String, Boolean>                mCameraSelection;
    // Link between camera name and the associated harwareMap names (2 for coupled cameras)
    private String                              mCurrentCamera;

    // Link between hardwareMap name and the corresponding single cameras
    private Map<String, CameraComponent>        mCameras;

    private CameraComponent                     mCamera;
    
    @Override
    public void runOpMode() {

        try {

            mLogger = new LogManager(null,FtcDashboard.getInstance(),"camera-tuning");
            mLogger.level(LogManager.Severity.TRACE);

            mHardware = new Hardware(this, hardwareMap, mLogger);
            
            mConfiguration = new Configuration(mLogger);
            mConfiguration.register("robot.hardware", mHardware);
            mConfiguration.read();
            mConfiguration.log();
            
            // All cameras
            mCameras = mHardware.cameras();

            mCameraSelection = new LinkedHashMap<>();
            for (Map.Entry<String, CameraComponent> camera : mCameras.entrySet()) {
                mCameraSelection.put(camera.getKey(),false);
            }

            // Add the camera selection variables on the dashboard
            for (Map.Entry<String, Boolean> selected : mCameraSelection.entrySet()) {
                SelectedProvider provider = new SelectedProvider(mCameraSelection, selected.getKey());
                FtcDashboard.getInstance().addConfigVariable(CameraTuning.class.getSimpleName(),selected.getKey(),provider);
            }

            FtcDashboard.getInstance().updateConfig();
            mLogger.update();

            waitForStart();

            mLogger.clear();

            while(opModeIsActive()) {

                /* Find current selected camera */
                String currentCamera = this.findSelectedCamera();
                mLogger.metric("Current camera",currentCamera);

                /* Manage configuration change */
                if(!Objects.equals(currentCamera, mCurrentCamera))  {
                    // Now we can forget the previously selected cameras sice we no longer need them
                    mCurrentCamera = currentCamera;
                    mCamera = mCameras.get(mCurrentCamera);
                }

                if(mCamera != null) {
                    Mat frame = mCamera.current();
                    if (frame != null) {
                        mLogger.debug("" + frame.cols());
                        mLogger.debug("" + frame.rows());
                        Bitmap bitmap = Bitmap.createBitmap(frame.cols(), frame.rows(), Bitmap.Config.ARGB_8888);
                        Utils.matToBitmap(frame, bitmap);
                        FtcDashboard.getInstance().sendImage(bitmap);
                    }
                    else {
                        mLogger.warning("Received null frame");
                    }
                }

                // Log cameras state and updated configuration
                this.logCamerasState(mLogger);
                mConfiguration.log();
                mHardware.update();

                mLogger.update();

                // Give time for camera change to occur
                sleep(10);
            }

            mConfiguration.write(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/camera-tuning.json");
            mLogger.info("Updated configuration saved. You may retrieve it using <b>adb pull /sdcard/FIRST/camera-tuning.json</b>");
            mLogger.update();
        }
        catch(Exception e) {
            mLogger.error(e.toString());
            mLogger.update();
        }
    }

    private String findSelectedCamera()
    {
        String result = "";
        for (Map.Entry<String, Boolean> selected : mCameraSelection.entrySet()) {
            if(selected.getValue()) { result = selected.getKey(); }
        }
        return result;
    }
    
    private void logCamerasState(LogManager logger) {
        logger.info("CURRENT CAMERA");

        if(mCamera != null) {
            logger.info("-----> HwMap : " + mCamera.name());
        }
    }

    // SelectedProvider updates the cameras selection states
    // Since Map<String, Boolean> is not a simple type, it's managed as
    // pointer, when we change it in the provider, it's changed in the
    // global class.
    // When we select a new camera, we make sure to deselect all the others
    static class SelectedProvider implements ValueProvider<Boolean> {
        final Map<String, Boolean> mAllSelection;
        final String mCurrentSelection;

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
}