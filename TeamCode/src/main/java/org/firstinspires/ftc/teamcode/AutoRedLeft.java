package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Auto Red Left", group="none")

public class AutoRedLeft extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        long runTime = System.currentTimeMillis();
        int ringType = 0;

        initialize();
        imageNavigation.init();

        waitForStart();

        while(System.currentTimeMillis() - runTime < 500){
            ringType = imageNavigation.getRingStack(this);
            telemetry.addData("stackCount:",  ringType);
            telemetry.update();
        }
        telemetry.addData("stackCount:",  ringType);
        telemetry.update();

        switch (ringType) {
            case 0:
                telemetry.addLine("Going to 0");
                Strafe(0.6f, 12, Direction.RIGHT);
                Drive(0.6f, 66, Direction.FORWARD);
                break;
            case 1:
                telemetry.addLine("Going to 1");
                Strafe(0.6f, 12, Direction.RIGHT);
                Drive(0.6f, 90, Direction.FORWARD);
                Strafe(0.6f, 12, Direction.LEFT);
                break;
            case 4:
                telemetry.addLine("Going to 4");
                Strafe(0.6f, 12, Direction.RIGHT);
                Drive(0.6f, 114, Direction.FORWARD);
                break;
        }
        telemetry.update();
    }
}