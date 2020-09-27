package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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

    VuforiaTrackables targetsSkyStone;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private TFObjectDetector tfod;

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

    OpenGLMatrix phoneLocationOnRobot;

    HardwareMap hardwareMap;
    LinearOpMode opMode;

    public ImageNavigation(HardwareMap h, LinearOpMode op)
    {
        hardwareMap = h;
        opMode = op;

        init();
    }

    public void init()
    {
        com.vuforia.Vuforia VU = new com.vuforia.Vuforia();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        com.vuforia.Vuforia.setInitParameters(null, 3, "");
        CameraDevice.getInstance().setField("iso", "100");
        param.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        //param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        com.vuforia.Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 13);

        vuforia = ClassFactory.getInstance().createVuforia(param);

        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        targetsSkyStone.activate();

        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("skystone");
        redRearBridge = targetsSkyStone.get(1);
        redRearBridge.setName("redRearBridge");
        blueRearBridge = targetsSkyStone.get(2);
        blueRearBridge.setName("blueRearBridge");
        redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("redFrontBridge");
        blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("blueFrontBridge");
        red1 = targetsSkyStone.get(5);
        red1.setName("red1");
        red2 = targetsSkyStone.get(6);
        red2.setName("red2");
        front1 = targetsSkyStone.get(7);
        front1.setName("front1");
        front2 = targetsSkyStone.get(8);
        front2.setName("front2");
        blue1 = targetsSkyStone.get(9);
        blue1.setName("blue1");
        blue2 = targetsSkyStone.get(10);
        blue2.setName("blue2");
        rear1 = targetsSkyStone.get(11);
        rear1.setName("rear1");
        rear2 = targetsSkyStone.get(12);
        rear2.setName("rear2");



        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        phoneLocationOnRobot = OpenGLMatrix
                .translation(142 / 2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));

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
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public float getSkystoneLocationByTf() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                        opMode.telemetry.addData("Location: ", String.format("left=%10.2f;  right=%10.2f", recognition.getLeft(), recognition.getRight()));
                        return recognition.getLeft();
                    }
                }
            }
        }
        return -99999;
    }

    public VectorF getSkyStoneLocation()
    {
        VuforiaTrackableDefaultListener stoneListener = (VuforiaTrackableDefaultListener) stoneTarget.getListener();

        if(stoneListener.isVisible())
        {
            OpenGLMatrix stonePosition = stoneListener.getPose();

            return stonePosition.getTranslation();
        }
        return null;
    }

    public OpenGLMatrix getRobotLocation()
    {
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);
        OpenGLMatrix lastLocation = null;

        while (opMode.opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {

                    ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, VuforiaLocalizer.CameraDirection.BACK);

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getRobotLocation();

                    if (robotLocationTransform != null) {
                        opMode.telemetry.addData("Phoenix XY: ", robotLocationTransform.getColumn(3).get(0) / 25.4f + " " + robotLocationTransform.getColumn(3).get(1) / 25.4f);

                        return robotLocationTransform;
                    }
                }
            }

            opMode.telemetry.update();
        }
        return null;
    }
}