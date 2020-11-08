package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Library.ImageNavigation;

import java.util.List;

@Autonomous(name="ImageTest", group="none")

public class ImageTest extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        long runTime = System.currentTimeMillis();

        initialize();

        waitForStart();

        while(opModeIsActive()){

            if(System.currentTimeMillis() - runTime > 100){
                this.telemetry.clearAll();
                runTime = System.currentTimeMillis();
            }
//            imageNavigation.getRobotLocation();

            telemetry.addData("stackCount:",  getRingStack());
            telemetry.update();
        }
    }

}
