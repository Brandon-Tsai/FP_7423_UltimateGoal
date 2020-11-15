package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Auto Red Left", group="none")

public class AutoRedLeft extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {


        int ringType = 0;

        initialize();
        imageNavigation.init();

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

        StrafeDiagonal(0.8f, Direction.BACKWARDLEFT);
        telemetry.update();
    }
}