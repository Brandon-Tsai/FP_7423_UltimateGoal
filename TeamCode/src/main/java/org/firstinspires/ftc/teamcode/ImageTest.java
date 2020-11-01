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

            /*
            if (imageNavigation.tfod != null) {
                List<Recognition> updatedRecognitions = imageNavigation.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        Log.i("Phoenix XY:", recognition.getLabel());
                        if (recognition.getLabel().equals(LABEL_QUAD)) {
                            telemetry.addData(String.format("label (%d)", i), "Quad");
                        } else if (recognition.getLabel().equals(LABEL_SINGLE)) {
                            telemetry.addData(String.format("label (%d)", i), "Single");
                        }
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                }
                ringType = 0;
            }
            */
            int t = imageNavigation.getRingStack(this);
            telemetry.addData("stackCount:",  t);
            telemetry.update();
        }
    }

}
