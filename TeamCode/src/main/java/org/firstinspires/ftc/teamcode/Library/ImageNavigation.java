package org.firstinspires.ftc.teamcode.Library;

import android.sax.StartElementListener;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class ImageNavigation
{
    public VuforiaTrackable stoneTarget;
    public VuforiaTrackable redRearBridge;
    public VuforiaTrackable blueRearBridge;
    public VuforiaTrackable redFrontBridge;
    public VuforiaTrackable blueFrontBridge;
    public VuforiaTrackable red1;
    public VuforiaTrackable red2;
    public VuforiaTrackable front1;
    public VuforiaTrackable front2;
    public VuforiaTrackable blue1;
    public VuforiaTrackable blue2;
    public VuforiaTrackable rear1;
    public VuforiaTrackable rear2;

    public VuforiaLocalizer vuforia;

    VuforiaTrackables targetsUltimateGoal;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_QUAD_ELEMENT = "Quad";
    private static final String LABEL_SINGLE_ELEMENT = "Single";

    public TFObjectDetector tfod;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    OpenGLMatrix camLocationOnRobot;

    HardwareMap hardwareMap;
    LinearOpMode opMode;

    WebcamName webcamName;

    public ImageNavigation(HardwareMap h, LinearOpMode op)
    {
        hardwareMap = h;
        opMode = op;
    }

    public void init()
    {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        com.vuforia.Vuforia VU = new com.vuforia.Vuforia();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //com.vuforia.Vuforia.setInitParameters(null, 3, "");
        //CameraDevice.getInstance().setField("iso", "100");
        param.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        //param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        param.cameraName = webcamName; // set camera to webcam
        param.useExtendedTracking = false;
        com.vuforia.Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 5);

        vuforia = ClassFactory.getInstance().createVuforia(param);

        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");



        camLocationOnRobot = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, -90, 0, 0));

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        targetsUltimateGoal.activate();
        //tensorflow
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD_ELEMENT, LABEL_SINGLE_ELEMENT);
    }

    public OpenGLMatrix getRobotLocation()
    {
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);
        OpenGLMatrix lastLocation = null;

        if (opMode.opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {

                    ((VuforiaTrackableDefaultListener)trackable.getListener()).setCameraLocationOnRobot(webcamName, camLocationOnRobot);

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getRobotLocation();

                    if (robotLocationTransform != null) {
                        opMode.telemetry.addData("Phoenix XY: ", robotLocationTransform.getColumn(3).get(0) / 25.4f + " " + robotLocationTransform.getColumn(3).get(1) / 25.4f);
                        Log.i("Phoenix XY:", robotLocationTransform.getColumn(3).get(0) / 25.4f + "  " + robotLocationTransform.getColumn(3).get(1) / 25.4f);
                        return robotLocationTransform;
                    }
                }
            }

            opMode.telemetry.update();
        }
        return null;
    }

    public int getRingStack(OpMode opMode) {
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getRecognitions();
            if (recognitions != null) {
                int i = 0;
                for (Recognition recognition : recognitions) {
                    float width = recognition.getRight() - recognition.getLeft();
                    opMode.telemetry.addData(String.format("  left,right,width (%d)", i), "%f , %f, %f",
                            recognition.getLeft(), recognition.getRight(), width);

                    float height = recognition.getTop() - recognition.getBottom();
                    opMode.telemetry.addData(String.format("  top,bottom,height (%d)", i), "%f , %f, %f",
                            recognition.getTop(), recognition.getBottom(), height);

                    if (width >= 110 && width <= 160) {
                        if (recognition.getLabel().equals(LABEL_QUAD_ELEMENT)) {
                            opMode.telemetry.addData(String.format("label (%d)", i), "Quad");
                            Log.i("[Phoenix]:", "Quad");
                            return 4;
                        } else if (recognition.getLabel().equals(LABEL_SINGLE_ELEMENT)) {
                            opMode.telemetry.addData(String.format("label (%d)", i), "Single");
                            Log.i("[Phoenix]:", "Single");
                            return 1;
                        }
                    }

                    opMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
                if (recognitions.size() != 0)
                    opMode.telemetry.addData("[Phoenix]:","Unrecognized label");
                else
                    opMode.telemetry.addData("[Phoenix]:","no label detected");
                return 0;
            }

            opMode.telemetry.addData("[Phoenix]:","no recognition object");
            return 0;
        }
        opMode.telemetry.addData("[Phoenix]:","no tfod");
        return 0;
    }
}