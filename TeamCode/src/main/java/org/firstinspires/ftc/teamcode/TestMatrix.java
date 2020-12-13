package org.firstinspires.ftc.teamcode;

import android.media.Image;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Library.ImageNavigation;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;
import org.firstinspires.ftc.teamcode.MyClass.FieldDirection;

@Autonomous(name="Test Matrix", group="none")
public class TestMatrix extends AutoBase {
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


    public int ringType;

    public void TestMatrix() {

    }

    public void initialize() {
        imageNavigation = new ImageNavigation(hardwareMap, this);

        imageNavigation.init(true);
    }

    public OpenGLMatrix GetTranslateMatrix(OpenGLMatrix fieldMatrix) {
        return null;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        this.waitForStart();

        OpenGLMatrix m = imageNavigation.getRobotLocation();
        Log.i("[matrix:m.3]", String.format("x: %f, y: %f, z: %f",
                m.getColumn(3).get(0), m.getColumn(3).get(1), m.getColumn(3).get(2)));
        Log.i("[matrix:m.2]", String.format("x: %f, y: %f, z: %f",
                m.getColumn(2).get(0), m.getColumn(2).get(1), m.getColumn(2).get(2)));

       // m.translate(10f, 10f, 10f);
       // Log.i("[matrix:t-after]", String.format("x: %f, y: %f, z: %f",
       //         m.getColumn(3).get(0), m.getColumn(3).get(1), m.getColumn(3).get(2)));

        OpenGLMatrix rotate45 = OpenGLMatrix.rotation(AngleUnit.DEGREES, 90f, 45f, 45f, 0f);
        Log.i("[matrix:rmatrix45.3]", String.format("x: %f, y: %f, z: %f",
                rotate45.getColumn(3).get(0), rotate45.getColumn(3).get(1), rotate45.getColumn(3).get(2)));
        Log.i("[matrix:rmatrix45.2]", String.format("x: %f, y: %f, z: %f",
                rotate45.getColumn(2).get(0), rotate45.getColumn(2).get(1), rotate45.getColumn(2).get(2)));

        m.multiply(rotate45);
        Log.i("[matrix:r-after.3]", String.format("x: %f, y: %f, z: %f",
                m.getColumn(3).get(0), m.getColumn(3).get(1), m.getColumn(3).get(2)));
        Log.i("[matrix:r-after.2]", String.format("x: %f, y: %f, z: %f",
                m.getColumn(2).get(0), m.getColumn(2).get(1), m.getColumn(2).get(2)));

        OpenGLMatrix transposed = m.transposed();
        Log.i("[matrix:transposed]", String.format("x: %f, y: %f, z: %f",
                transposed.getColumn(3).get(0), transposed.getColumn(3).get(1), transposed.getColumn(3).get(2)));
        Log.i("[matrix:transposed]", String.format("x: %f, y: %f, z: %f",
                transposed.getColumn(2).get(0), transposed.getColumn(2).get(1), transposed.getColumn(2).get(2)));

        m.rotate(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES, 0f, 45f, 45f);
        Log.i("[matrix:m-after]", String.format("x: %f, y: %f, z: %f",
                m.getColumn(3).get(0), m.getColumn(3).get(1), m.getColumn(3).get(2)));

        OpenGLMatrix targetMatrix1 = m.rotated(AngleUnit.DEGREES, 45, 0F, 0F, 0F);
        Log.i("[matrix:1]", String.format("x: %f, y: %f, z: %f",
                targetMatrix1.getColumn(3).get(0), targetMatrix1.getColumn(3).get(1), targetMatrix1.getColumn(3).get(2)));

        OpenGLMatrix transposed2 = m.transposed();
        Log.i("[matrix:transposed2]", String.format("x: %f, y: %f, z: %f",
                transposed2.getColumn(3).get(0), transposed2.getColumn(3).get(1), transposed2.getColumn(3).get(2)));
        Log.i("[matrix:transposed2]", String.format("x: %f, y: %f, z: %f",
                transposed2.getColumn(2).get(0), transposed2.getColumn(2).get(1), transposed2.getColumn(2).get(2)));

        OpenGLMatrix targetMatrix2 = m.rotated(AngleUnit.DEGREES, -45, 0F, 0F, 0F);
        Log.i("[matrix:2]", String.format("x: %f, y: %f, z: %f",
                targetMatrix2.getColumn(3).get(0), targetMatrix2.getColumn(3).get(1), targetMatrix2.getColumn(3).get(2)));

        OpenGLMatrix targetMatrix3 = m.rotated(AngleUnit.DEGREES, 135, 0F, 0F, 0F);
        Log.i("[matrix:3]", String.format("x: %f, y: %f, z: %f",
                targetMatrix3.getColumn(3).get(0), targetMatrix3.getColumn(3).get(1), targetMatrix3.getColumn(3).get(2)));

        OpenGLMatrix targetMatrix4 = m.rotated(AngleUnit.DEGREES, -135, 0F, 0F, 0F);
        Log.i("[matrix:4]", String.format("x: %f, y: %f, z: %f",
                targetMatrix4.getColumn(3).get(0), targetMatrix4.getColumn(3).get(1), targetMatrix4.getColumn(3).get(2)));

        targetMatrix1 = m.rotated(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES, 45F, 0F, 0F);
        Log.i("[matrix:1]", String.format("x: %f, y: %f, z: %f",
                targetMatrix1.getColumn(3).get(0), targetMatrix1.getColumn(3).get(1), targetMatrix1.getColumn(3).get(2)));

        targetMatrix2 = m.rotated(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES, -45F, 0F, 0F);
        Log.i("[matrix:2]", String.format("x: %f, y: %f, z: %f",
                targetMatrix2.getColumn(3).get(0), targetMatrix2.getColumn(3).get(1), targetMatrix2.getColumn(3).get(2)));

        targetMatrix3 = m.rotated(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES, 135F, 0F, 0F);
        Log.i("[matrix:3]", String.format("x: %f, y: %f, z: %f",
                targetMatrix3.getColumn(3).get(0), targetMatrix3.getColumn(3).get(1), targetMatrix3.getColumn(3).get(2)));

        targetMatrix4 = m.rotated(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES, -135F, 0F, 0F);
        Log.i("[matrix:4]", String.format("x: %f, y: %f, z: %f",
                targetMatrix4.getColumn(3).get(0), targetMatrix4.getColumn(3).get(1), targetMatrix4.getColumn(3).get(2)));
    }
}