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

        if(ringType != 0) {
            Turn(0.5f, 90, Direction.CLOCKWISE, imu, this);
            sleep(1000);
            StrafeTo(0.8f, Direction.FORWARDRIGHT, -90, 0, -24);
            sleep(500);
            StrafeTo(0.8f, Direction.FORWARDRIGHT, -80, -24, -30);
            Log.i("[phoenix:finished]", "program finished");
        }
    }
}