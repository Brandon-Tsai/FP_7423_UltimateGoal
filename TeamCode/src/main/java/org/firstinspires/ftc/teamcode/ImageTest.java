package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Library.ImageNavigation;

@Autonomous(name="ImageTest", group="none")

public class ImageTest extends LinearOpMode {

    public ImageNavigation imageNavigation;

    @Override
    public void runOpMode() throws InterruptedException {

        imageNavigation = new ImageNavigation(hardwareMap, this);

        waitForStart();

        while(opModeIsActive()){
            imageNavigation.getRobotLocation();
        }
    }

}
