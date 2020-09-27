package org.firstinspires.ftc.teamcode;

import android.provider.ContactsContract;
import android.service.dreams.DreamService;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.Library.ImageNavigation;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;
import org.firstinspires.ftc.teamcode.MyClass.PositionToImage;
import org.firstinspires.ftc.teamcode.MyClass.SkystonePosition;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public abstract class AutoBase extends LinearOpMode {
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;

    public Servo twister;

    Servo pullerLeft;
    Servo pullerRight;

    public DistanceSensor distanceSensor;
    public DistanceSensor backLeftDistanceSensor;
    public ColorSensor colorSensor;

    public float PPR = 560F; //changed ppr for test robot
    public float diameter = 3F;
    public MyBoschIMU imu;

    DcMotor intakeMotorLeft;
    DcMotor intakeMotorRight;

    DcMotor slideMotorLeft;
    DcMotor slideMotorRight;

    protected PositionToImage lastKnownPosition;

    public ImageNavigation imageNavigation;

    public int primaryAngle;

    public int skystonePosition = 0;

    public void initialize() {
        lastKnownPosition = new PositionToImage(); //instantiate this first

        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        imageNavigation = new ImageNavigation(hardwareMap, this);

        imu = new MyBoschIMU(hardwareMap);
        imu.initialize(new BNO055IMU.Parameters());

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        backLeftDistanceSensor = hardwareMap.get(DistanceSensor.class, "backLeftDistanceSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        intakeMotorLeft = hardwareMap.dcMotor.get("intaketh1");
        intakeMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotorRight = hardwareMap.dcMotor.get("intaketh2");
        intakeMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotorLeft = hardwareMap.dcMotor.get("slideMotorLeft");
        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorRight = hardwareMap.dcMotor.get("slideMotorRight");
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        primaryAngle = (int)imu.myIMU.getAngularOrientation().firstAngle;

        pullerLeft = hardwareMap.servo.get("pullerLeft");
        ServoControllerEx pullerLeftController = (ServoControllerEx) pullerLeft.getController();
        int pullerLeftServoPort = pullerLeft.getPortNumber();
        PwmControl.PwmRange pullerLeftPwmRange = new PwmControl.PwmRange(899, 1300);
        pullerLeftController.setServoPwmRange(pullerLeftServoPort, pullerLeftPwmRange);
        pullerLeft.setPosition(0);

        pullerRight = hardwareMap.servo.get("pullerRight");
        ServoControllerEx pullerRightController = (ServoControllerEx) pullerRight.getController();
        int pullerRightServoPort = pullerRight.getPortNumber();
        PwmControl.PwmRange pullerRightPwmRange = new PwmControl.PwmRange(1680, 2105);
        pullerRightController.setServoPwmRange(pullerRightServoPort, pullerRightPwmRange);
        pullerRight.setPosition(1);
    }

    private float Max(float x1, float x2, float x3, float x4) {

        x1 = Math.abs(x1);
        x2 = Math.abs(x2);
        x3 = Math.abs(x3);
        x4 = Math.abs(x4);
        float m = x1;

        if (x2 > m)
            m = x2;
        if (x3 > m)
            m = x3;
        if (x4 > m)
            m = x4;

        return m;
    }

    public float aquireStoneTarget() {
        VuforiaTrackableDefaultListener imageListener = (VuforiaTrackableDefaultListener) imageNavigation.stoneTarget.getListener();


        if (imageListener.isVisible()) {
            OpenGLMatrix pos = imageListener.getPose();
            float d = pos.getColumn(3).get(2);
            telemetry.addData("Distance:", "%f", d);
            Orientation orientation = Orientation.getOrientation(pos, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            OpenGLMatrix rotationMatrix = OpenGLMatrix.rotation(AxesReference.EXTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES, orientation.thirdAngle * -1, orientation.firstAngle * -1, 0);
            OpenGLMatrix adjustedPose = pos.multiplied(rotationMatrix);
            Orientation adjustedOrientation = Orientation.getOrientation(adjustedPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            telemetry.addData("Angle: ", "x = %f, y = %f, z = %f", adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation.thirdAngle);

            return d;
        }
        return 0;
    }

    public double getObstacleDistance() {
        if (distanceSensor != null) {
            double d = distanceSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Obstacle Distance: ", "%f", d);
            return d;
        }
        else
            return -1;
    }

    public double StrafeUntilDistance(float power, Direction direction, float safetyDistance, float robotStartingAngle, MyBoschIMU imu) {
        float robotCurrentAngle;
        double d = distanceSensor.getDistance(DistanceUnit.INCH);

        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        float flPower, frPower, blPower, brPower;
        float angleModify = Math.abs(power);
        float actualPower = Math.abs(power);
        if (direction == Direction.LEFT)
            actualPower = -(power);


        while (d > safetyDistance) {
            robotCurrentAngle = imu.getAngularOrientation().firstAngle;
            if (direction == Direction.LEFT)
            {
                if (robotCurrentAngle - robotStartingAngle >= 3) // 3 degrees or more
                {
                    flPower = actualPower; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower;
                    blPower = -actualPower - angleModify;
                    brPower = actualPower + angleModify;
                }
                else if (robotCurrentAngle - robotStartingAngle <= -3) // -3 degrees or more
                {
                    flPower = actualPower + angleModify; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower - angleModify;
                    blPower = -actualPower;
                    brPower = actualPower;
                }
                else
                {
                    flPower = actualPower; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower;
                    blPower = -actualPower;
                    brPower = actualPower;
                }
            }
            else if(direction == Direction.RIGHT)
            {
                if (robotCurrentAngle - robotStartingAngle >= 3) // 3 degrees or more
                {
                    flPower = actualPower + angleModify; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower - angleModify;
                    blPower = -actualPower;
                    brPower = actualPower;
                }
                else if (robotCurrentAngle - robotStartingAngle <= -3) // -3 degrees or more
                {
                    flPower = actualPower; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower;
                    blPower = -actualPower - angleModify;
                    brPower = actualPower + angleModify;
                }
                else
                {
                    flPower = actualPower; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower;
                    blPower = -actualPower;
                    brPower = actualPower;
                }
            }
            else
            {
                flPower = 0;
                frPower = 0;
                blPower = 0;
                brPower = 0;
            }

            float max = Max(flPower, frPower, blPower, brPower);

            if (max < 1)
                max = 1; //By setting max to 1, the fl, fr, bl and br power would be the power we intended to use; and none of these are over 1 because max is less than 1

            fl.setPower(flPower / max);
            fr.setPower(frPower / max);
            bl.setPower(blPower / max);
            br.setPower(brPower / max);

            d = distanceSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Obstacle Distance: ", "%f", d);
            telemetry.update();
        }
        StopAll();
        double driveDistance = br.getCurrentPosition();
        return driveDistance;
    }

    public void StopAll() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void Strafe(float power, float distance, Direction d /*, OpMode op*/) {

        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu.resetAndStart(Direction.NONE);

        int robotStartingAngle = (int)imu.getAngularOrientation().firstAngle;

        float x = (2.0f * PPR * distance) / (diameter * (float) Math.PI); // used to be a 2 at top. tried 1.5, seems ok
        int targetEncoderValue = Math.round(x);

        float actualPower = power;
        if (d == Direction.LEFT)
            actualPower = -(power);

        int initialPosition = br.getCurrentPosition();
        int positionDiff = 0;

        while (positionDiff < targetEncoderValue && this.opModeIsActive()) {

            /* op.telemetry.addData("current:", currentPosition);
            op.telemetry.addData("target:", targetEncoderValue);
            op.telemetry.update();
            */
            int currentPosition = br.getCurrentPosition();

            positionDiff = Math.abs(currentPosition - initialPosition); //changed encoder for test bot
            //Log.i("[Phoenix]:encoder #", Integer.toString(positionDiff));

            //if(currentPosition < 200)
            //actualPower = .28F;

            float flPower, frPower, blPower, brPower;

            float robotCurrentAngle = imu.getAngularOrientation().firstAngle;

            float angleModify = power;

            if (d == Direction.LEFT)
            {
                Log.i("[phoenix:strafe]", String.format("Strafing left;startingAngle= %d; currentAngle=%f10.2", robotStartingAngle, robotCurrentAngle));

                if (robotCurrentAngle - robotStartingAngle >= 3) // 3 degrees or more
                {
                    flPower = actualPower + angleModify; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower - angleModify;
                    blPower = -actualPower;
                    brPower = actualPower;
                }
                else if (robotCurrentAngle - robotStartingAngle <= -3) // -3 degrees or more
                {
                    flPower = actualPower; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower;
                    blPower = -actualPower - angleModify;
                    brPower = actualPower + angleModify;
                }
                else
                {
                    flPower = actualPower; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower;
                    blPower = -actualPower;
                    brPower = actualPower;
                }
            }
            else if(d == Direction.RIGHT)
            {
                Log.i("[phoenix:strafe]", String.format("Strafing right;startingAngle= %d; currentAngle=%f10.2", robotStartingAngle, robotCurrentAngle));

                if (robotCurrentAngle - robotStartingAngle >= 3) // 3 degrees or more
                {
                    flPower = actualPower + angleModify; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower - angleModify;
                    blPower = -actualPower;
                    brPower = actualPower;
                }
                else if (robotCurrentAngle - robotStartingAngle <= -3) // -3 degrees or more
                {
                    flPower = actualPower; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower;
                    blPower = -actualPower - angleModify;
                    brPower = actualPower + angleModify;
                }
                else
                {
                    flPower = actualPower; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower;
                    blPower = -actualPower;
                    brPower = actualPower;
                }
            }
            else
            {
                flPower = 0;
                frPower = 0;
                blPower = 0;
                brPower = 0;
            }

            float max = Max(flPower, frPower, blPower, brPower);

            if (max < 1)
                max = 1; //By setting max to 1, the fl, fr, bl and br power would be the power we intended to use; and none of these are over 1 because max is less than 1

            Log.i("[phoenix]", String.format("flPower:%10f frPower:%10f blPower:%10f brPower:%10f angleModify:%10f currentAngle:%10f", flPower / max, frPower / max, blPower / max, brPower / max, angleModify, robotCurrentAngle));

            fl.setPower(flPower / max);
            fr.setPower(frPower / max);
            bl.setPower(blPower / max);
            br.setPower(brPower / max);

        }

        StopAll();
    }

    public void Drive(float power, float distance, Direction d) {

        float x = (PPR * distance) / (diameter * (float) Math.PI);
        int targetEncoderValue = Math.round(x);


        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        if (d == Direction.BACKWARD) {
            power = -1 * power;
        }

        while (currentPosition < targetEncoderValue && opModeIsActive()) {

            currentPosition = (Math.abs(br.getCurrentPosition()));
            //Log.i("[Phoenix]:encoder #", Integer.toString(currentPosition));
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        }
        StopAll();
    }

    public void Turn(float power, int angle, Direction d, MyBoschIMU imu, LinearOpMode opMode) {

        Orientation startOrientation = imu.resetAndStart(d);

        float targetAngle;
        float currentAngle;
        float actualPower = power;
        float stoppingAngle = 0;

        if (d == Direction.CLOCKWISE) {
            actualPower = -(power);

            targetAngle = startOrientation.firstAngle - angle;
            currentAngle = startOrientation.firstAngle;

            while ((currentAngle - stoppingAngle) > targetAngle && opMode.opModeIsActive()) {

                opMode.telemetry.addData("start:", startOrientation.firstAngle);
                opMode.telemetry.addData("current:", currentAngle);
                opMode.telemetry.addData("target:", targetAngle);
                opMode.telemetry.update();

                currentAngle = imu.getAngularOrientation().firstAngle;
                AngularVelocity v = imu.getAngularVelocity();
                float speed = Math.abs(v.xRotationRate);
                stoppingAngle = Math.abs((0.25f * speed) - 7f);
                Log.i("[phoenix:turnTest]", String.format("StartingAngle=%f, CurrentAngle=%f, AngularVelocity=%f, StoppingAngle=%f", startOrientation.firstAngle, currentAngle, speed, stoppingAngle));

                fl.setPower(-(actualPower));
                fr.setPower(actualPower);
                bl.setPower(-(actualPower));
                br.setPower(actualPower);
            }
        } else {
            actualPower = power;

            targetAngle = startOrientation.firstAngle + angle;
            currentAngle = startOrientation.firstAngle;
            while ((currentAngle + stoppingAngle) < targetAngle && opMode.opModeIsActive()) {

                opMode.telemetry.addData("start:", startOrientation.firstAngle);
                opMode.telemetry.addData("current:", currentAngle);
                opMode.telemetry.addData("target:", targetAngle);
                opMode.telemetry.update();

                currentAngle = imu.getAngularOrientation().firstAngle;
                AngularVelocity v = imu.getAngularVelocity();
                float speed = Math.abs(v.xRotationRate);
                stoppingAngle = Math.abs((0.25f * speed) - 7f); //-8.5609

                fl.setPower(-(actualPower));
                fr.setPower(actualPower);
                bl.setPower(-(actualPower));
                br.setPower(actualPower);
            }
        }
        StopAll();
    }

    public void resetAllEncoders() {
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getLateralMovement() {
        int frCount = fr.getCurrentPosition();
        int brCount = br.getCurrentPosition();
        int encoderValue = (frCount + brCount)/2;
        double distance = ((double)encoderValue * (double)6 * Math.PI) / (double) 1120;

        Log.i("[phoenix:getLateral]", String.format("br=%d; fr=%d; distance=%f10.2; encoderValue=%d", brCount, frCount, distance, encoderValue));

        return distance;
    }

    public boolean StrafeToImage(float power, VuforiaTrackable imageTarget, LinearOpMode opMode, double stopDistance) {

        stopDistance = stopDistance * 25.4;

        VuforiaTrackableDefaultListener imageListener = (VuforiaTrackableDefaultListener) imageTarget.getListener();

        float actualPower = power;

        if (imageListener.isVisible()) {
            OpenGLMatrix pos = imageListener.getPose();
            float d = pos.getColumn(3).get(2); //distance to the image in millimeter;
            float x = pos.getColumn(3).get(1);
            float additionalpower = 0;

            lastKnownPosition.translation = pos.getTranslation();

            Log.i("[phoenix]", String.format("distanceImage (before loop) = %10.2f", d));

            Boolean tooCloseToObstacle = false;
            int distanceLimit = 8;
            double distanceToObstacle = getObstacleDistance();
            if (distanceToObstacle < distanceLimit && distanceToObstacle > 0)
                tooCloseToObstacle = true;

            while ((Math.abs(d) >= stopDistance) && !tooCloseToObstacle && (imageListener.isVisible()) && opMode.opModeIsActive()) {

                pos = ((VuforiaTrackableDefaultListener) imageTarget.getListener()).getPose();
                d = pos.getColumn(3).get(2); //distance to the image in millimeter;
                x = -1 * pos.getColumn(3).get(1);

                Log.i("[phoenix]", String.format("distanceImage = %10.2f", d));

                Orientation orientation = Orientation.getOrientation(pos, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                OpenGLMatrix rotationMatrix = OpenGLMatrix.rotation(AxesReference.EXTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES, orientation.thirdAngle * -1, orientation.firstAngle * -1, 0);
                OpenGLMatrix adjustedPose = pos.multiplied(rotationMatrix);
                Orientation adjustedOrientation = Orientation.getOrientation(adjustedPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                opMode.telemetry.addData("Angle: ", "x = %f, y = %f, z = %f", adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation.thirdAngle);

                //Keep track the last known location
                lastKnownPosition.translation = pos.getTranslation();
                lastKnownPosition.orientation = adjustedOrientation;

                d = lastKnownPosition.translation.get(2);
                x = lastKnownPosition.translation.get(1) * -1;

                opMode.telemetry.addData("x: ", "x = %f", x);

                float distanceAdjustment = Math.abs(d);
                if (distanceAdjustment > 1200F)
                    distanceAdjustment = 1200F;
                else if (distanceAdjustment < 700F)
                    distanceAdjustment = 0;

                if (x > 50) {
                    additionalpower = actualPower * 0.5F * (Math.abs(x) / 150F) * ((1200F - distanceAdjustment) / 1200F);
                } else if (x < -50) {
                    additionalpower = actualPower * -0.5F * (Math.abs(x) / 150F) * ((1200F - distanceAdjustment) / 1200F);
                }

                float flTurnAdjust = 0;
                float blTurnAdjust = 0;
                float currentAngle = imu.getAngularOrientation().firstAngle;

                float angleDiff = currentAngle - primaryAngle;


                if (angleDiff < -3) {
                    flTurnAdjust = actualPower * -2F;
                } else if (angleDiff > 3) {
                    blTurnAdjust = actualPower * 2F;
                }

                float flPower, frPower, blPower, brPower;

                flPower = actualPower + additionalpower + flTurnAdjust;
                frPower = -actualPower + additionalpower;
                blPower = -actualPower + additionalpower + blTurnAdjust;
                brPower = actualPower + additionalpower;

                float max = Max(flPower, frPower, blPower, brPower);

                if (max < 1)
                    max = 1; //By setting max to 1, the fl, fr, bl and br power would be the power we intended to use; and none of these are over 1 because max is less than 1

                fl.setPower(flPower / max);
                fr.setPower(frPower / max);
                bl.setPower(blPower / max);
                br.setPower(brPower / max);
                Log.i("[phoenix:StrafeToImage]", String.format("x = %f, d = %f, addpower = %f, actpower = %f, distadj = %f", x, d, additionalpower, actualPower, distanceAdjustment));
                Log.i("[phoenix:StrafeToImage]", String.format("angleDiff = %5.2f, currentAngle = %5.2f, flTurnAdjust = %5.4f, blTurnAdjust = %5.4f", angleDiff, currentAngle, flTurnAdjust, blTurnAdjust));

                //                Log.i("[phoenix:StrafeToImage]", String.format("adj x=%f, y=%f, z=%f, flTurnAdjust=%f, blTurnAdjust=%f", adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation.thirdAngle, flTurnAdjust, blTurnAdjust));
                opMode.telemetry.update();

                if (distanceToObstacle < distanceLimit && distanceToObstacle > 0)
                    tooCloseToObstacle = true;
            }

        }
        else {
            return false;
        }
        StopAll();
        opMode.telemetry.update();
        return true;
    }

    public boolean detectSkystoneByColor() {

        if (colorSensor.green() < 400) {
            telemetry.addData("gotgreen:", "oh yes");
            return true;
        }
        telemetry.addData("gotgreen:", "not really");
        return false;
    }

    public void DriveToCoordinate(float x, float y) {
        double frPower = 0;
        double flPower = 0;
        double brPower = 0;
        double blPower = 0;

        float yDiff = -1000;
        float xDiff = 1000;

        while (yDiff < - 100 || Math.abs(xDiff) > 100)
        {
            frPower = 0;
            flPower = 0;
            brPower = 0;
            blPower = 0;

            OpenGLMatrix robotLocation = imageNavigation.getRobotLocation();

            if (robotLocation != null)
            {
                yDiff = y - robotLocation.getColumn(3).get(1);
                xDiff = x - robotLocation.getColumn(3).get(0);

                //Log.i("xydiff & xylocation", String.format("yDiff=%10.2f xDiff=%10.2f currentY=%10.2f currentX=%10.2f", yDiff/25.4, xDiff/25.4, robotLocation.getColumn(3).get(1)/25.4, robotLocation.getColumn(3).get(0)/25.4));
                Log.i("[phoenix]",  String.format("translateX = %10.2f, translateY = %10.2f, translateZ = %10.2f", robotLocation.getTranslation().get(0)/25.4, robotLocation.getTranslation().get(1)/25.4, robotLocation.getTranslation().get(2)/25.4));
                if (yDiff < -100)
                {
                    frPower = 0.3;
                    flPower = 0.3;
                    brPower = 0.3;
                    blPower = 0.3;
                }

                if (Math.abs(xDiff) > 100)
                {
                    double strafeAdjustmentPower = 0;

                    if (xDiff > 0)
                        strafeAdjustmentPower = -0.3;
                    else
                        strafeAdjustmentPower = 0.3;

                    frPower = frPower + strafeAdjustmentPower;
                    flPower = flPower - strafeAdjustmentPower;
                    brPower = brPower - strafeAdjustmentPower;
                    blPower = blPower + strafeAdjustmentPower;
                }

                fl.setPower(flPower);
                fr.setPower(frPower);
                bl.setPower(blPower);
                br.setPower(brPower);
            }

            else {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
            }

        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public double DriveUntilDistance(float power, Direction direction, float safetyDistance, float robotStartingAngle, MyBoschIMU imu) {
        float robotCurrentAngle;
        double d = backLeftDistanceSensor.getDistance(DistanceUnit.INCH);

        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        float flPower, frPower, blPower, brPower;
        float angleModify = Math.abs(power);
        float actualPower = Math.abs(power);


        while (d > safetyDistance) {
            robotCurrentAngle = imu.getAngularOrientation().firstAngle;
            if (robotCurrentAngle - robotStartingAngle >= 3) // 3 degrees or more
            {
                flPower = actualPower + angleModify; //when strafe to left, actual power is negative, but power remains positive.
                frPower = actualPower;
                blPower = actualPower;
                brPower = actualPower;
            }
            else if (robotCurrentAngle - robotStartingAngle <= -3) // -3 degrees or more
            {
                flPower = actualPower; //when strafe to left, actual power is negative, but power remains positive.
                frPower = actualPower + angleModify;
                blPower = actualPower;
                brPower = actualPower;
            }
            else
            {
                flPower = actualPower; //when strafe to left, actual power is negative, but power remains positive.
                frPower = actualPower;
                blPower = actualPower;
                brPower = actualPower;
            }
            float max = Max(flPower, frPower, blPower, brPower);

            if (max < 1)
                max = 1; //By setting max to 1, the fl, fr, bl and br power would be the power we intended to use; and none of these are over 1 because max is less than 1

            fl.setPower(flPower / max);
            fr.setPower(frPower / max);
            bl.setPower(blPower / max);
            br.setPower(brPower / max);

            d = backLeftDistanceSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Obstacle Distance: ", "%f", d);
            telemetry.update();
        }
        StopAll();
        double driveDistance = br.getCurrentPosition();
        return driveDistance;
    }

    public float getSkystoneXPosition(VuforiaTrackable imageTarget) {
        float x = 0;
        VuforiaTrackableDefaultListener imageListener = (VuforiaTrackableDefaultListener) imageTarget.getListener();
        if (imageListener.isVisible()) {
            OpenGLMatrix pos = imageListener.getPose();
            lastKnownPosition.translation = pos.getTranslation();
            x = lastKnownPosition.translation.get(1) / 25.4F;
        }
        return x;
    }

    public void releaseStone(){
        intakeMotorLeft.setPower(-1);
        intakeMotorRight.setPower(-1);

        this.sleep(300);

        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);

    }
}

