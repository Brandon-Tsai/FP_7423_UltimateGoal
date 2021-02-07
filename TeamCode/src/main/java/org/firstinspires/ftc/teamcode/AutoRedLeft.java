package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Auto Red Left", group="none")

public class AutoRedLeft extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {


        int ringType = 0;

        initialize();
        imageNavigation.init(false);

        waitForStart();


        long runTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - runTime < 500){
            ringType = imageNavigation.getRingStack(this);
            telemetry.addData("stackCount:",  ringType);
            telemetry.update();
        }
        telemetry.addData("stackCount:",  ringType);
        telemetry.update();

        switch (ringType) {
            case 0:
                Strafe(0.6f, 12, Direction.RIGHT);
                Drive(0.6f, 56, Direction.FORWARD);
                break;
            case 1:
                Strafe(0.6f, 12, Direction.RIGHT);
                Drive(0.6f, 80, Direction.FORWARD);
                Strafe(0.6f, 12, Direction.LEFT);
                break;
            case 4:
                Strafe(0.6f, 12, Direction.RIGHT);
                Drive(0.6f, 105, Direction.FORWARD);
                break;
        }

        Log.i("[phoenix:arrive]", "Hola");

        imageNavigation.getRobotLocation();

        sleep(1000);
        Log.i("[phoenix:startStrafe]", "Start Strafe");

        StrafeTo(0.8f, Direction.BACKWARDLEFT, 0, 12, -20);

        telemetry.update();

        //shoot rings at powershot
        ramp(1);
        elevate(1);
        sleep(500);
        shooter.setPower(0);
        elevate(0);

        if(ringType != 0) {
            sleep(1000);
            Turn(0.5f, 90, Direction.CLOCKWISE, imu, this);
            Drive(0.6f, 12, Direction.FORWARD);
            sleep(1000);
            StrafeTo(0.8f, Direction.FORWARDRIGHT, -90, 0, -36);
            sleep(200);
            Turn(0.5f, 90, Direction.CLOCKWISE, imu, this);

            intake.setPower(1);//or -1
            transfer.setPower(1);//or -1
            Drive(0.6f, 24, Direction.FORWARD);
            //collect rigns
            sleep(1000);
            intake.setPower(0);
            transfer.setPower(0);

            Turn(0.5f, 180, Direction.COUNTERCLOCKWISE, imu, this);
            Drive(0.6f, 24, Direction.FORWARD);
            //shoot at goal
            ramp(2);
            elevate(3);
            sleep(500);
            shooter.setPower(0);
            elevate(0);

            Log.i("[phoenix:finished]", "program finished");
        }
    }
}