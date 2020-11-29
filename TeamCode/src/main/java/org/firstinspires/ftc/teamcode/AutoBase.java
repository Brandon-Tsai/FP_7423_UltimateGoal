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
import org.firstinspires.ftc.teamcode.MyClass.FieldDirection;
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

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_QUAD = "Quad";
    public static final String LABEL_SINGLE = "Single";

    public float PPR = 560F; //andymark gear ratio 40:1 (previously 280)
    public float diameter = 3F; //changed for 3 inch wheel testing
    public MyBoschIMU imu;

    public ImageNavigation imageNavigation;
    public int ringType;

    public void AutoBase() {

    }

    public void initialize() {
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        imageNavigation = new ImageNavigation(hardwareMap, this);

        imu = new MyBoschIMU(hardwareMap);
        imu.initialize(new BNO055IMU.Parameters());
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
    public void StrafeTo(float power, Direction d, float targetX, float targetY){
        float frontLeftPower = 0;
        float frontRightPower = 0;
        float backLeftPower = 0;
        float backRightPower = 0;

        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        float startingAngle = imu.getAngularOrientation().firstAngle;
        float angleModify = Math.abs(power) / 2f;

        float powerFactor = 0.33333333f;
        while(Math.abs(br.getCurrentPosition()) < 8000 && opModeIsActive()){
            OpenGLMatrix location = imageNavigation.getRobotLocation();

            if(location != null){
                float currentX = location.getColumn(3).get(0) / 25.4f;
                float currentY = location.getColumn(3).get(1) / 25.4f;

                float deltaX = currentX - targetX;
                float deltaY = currentY - targetY;
                float ratio = Math.abs(deltaX/deltaY);
                powerFactor = Math.abs((1f-0.5f*ratio)/(0.5f*ratio+1f));

                float angle = imu.getAngularOrientation().firstAngle;
                FieldDirection robotFacingDirection = FieldDirection.EAST;
                if (angle > -45f && angle < 45f)
                    robotFacingDirection = FieldDirection.EAST;
                else if (angle <= -45f && angle > - 135f)
                    robotFacingDirection = FieldDirection.SOUTH;
                else if (angle >= 45f && angle < 135f)
                    robotFacingDirection = FieldDirection.NORTH;
                else
                    robotFacingDirection = FieldDirection.WEST;

                Log.i("[phoenix:getDirection]", String.format("Direction:%s, FacingDirection:%s, currentX:%f, currentY:%f", d.toString(), robotFacingDirection.toString(), currentX, currentY));

                if(d == Direction.FORWARDRIGHT){

                }
                else if(d == Direction.FORWARDLEFT){
                    if(robotFacingDirection == FieldDirection.EAST && currentY >= targetY && currentX >= targetX){
                        break;
                    } else if(robotFacingDirection == FieldDirection.WEST && currentY <= targetY && currentX <= targetX){
                        break;
                    } else if(robotFacingDirection == FieldDirection.NORTH && currentY >= targetY && currentX <= targetX){
                        break;
                    } else if(robotFacingDirection == FieldDirection.SOUTH && currentY <= targetY && currentX >= targetX){
                        break;
                    }
                }
                else if(d == Direction.BACKWARDRIGHT){
                    if(currentY >= targetY && currentX <= targetX){
                        break;
                    }
                }
                else if(d == Direction.BACKWARDLEFT){
                    if(robotFacingDirection == FieldDirection.EAST && currentY >= targetY && currentX <= targetX){
                        break;
                    } else if(robotFacingDirection == FieldDirection.WEST && currentY <= targetY && currentX >= targetX){
                        break;
                    } else if(robotFacingDirection == FieldDirection.NORTH && currentY <= targetY && currentX <= targetX){
                        break;
                    } else if(robotFacingDirection == FieldDirection.SOUTH && currentY >= targetY && currentX >= targetX){
                        break;
                    }
                }


            }
            else{
                Log.i("[phoenix:stopSeeing]", "Stopped Seeing Image");
            }

            switch(d){
                case FORWARDRIGHT:
                    frontLeftPower = power;
                    backRightPower = power;
                    frontRightPower = -power * powerFactor;
                    backLeftPower = -power * powerFactor;
                    break;
                case FORWARDLEFT:
                    frontRightPower = power;
                    backLeftPower = power;
                    frontLeftPower = -power * powerFactor;
                    backRightPower = -power * powerFactor;
                    break;
                case BACKWARDRIGHT:
                    frontRightPower = -power;
                    backLeftPower = -power;
                    frontLeftPower = power * powerFactor;
                    backRightPower = power * powerFactor;
                    break;
                case BACKWARDLEFT:
                    frontLeftPower = -power;
                    backRightPower = -power;
                    frontRightPower = power * powerFactor;
                    backLeftPower = power * powerFactor;
                    break;
            }

            //Log.i("[phoenix:imuDiagonal]", String.format("StartingAngle:%f, CurrentAngle:%f, AngleDifference:%f", startingAngle, imu.getAngularOrientation().firstAngle, imu.getAngularOrientation().firstAngle - startingAngle));

            float currentAngle = imu.getAngularOrientation().firstAngle;
            float flPower = frontLeftPower;
            float frPower = frontRightPower;
            float blPower = backLeftPower;
            float brPower = backRightPower;

            if (currentAngle - startingAngle >= 2){
                flPower += angleModify;
                frPower -= angleModify;
            }
            else if(currentAngle - startingAngle <= -2){
                blPower -= angleModify;
                brPower += angleModify;
            }

            float max = Max(flPower, frPower, blPower, brPower);

            if (max < 1)
                max = 1;

            fl.setPower(flPower / max);
            fr.setPower(frPower / max);
            bl.setPower(blPower / max);
            br.setPower(brPower / max);
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
}

