package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="AtlasTeleopGyro", group="OpMode")
public class atlasTeleop extends OpMode {

    public DcMotor lf, rf, lb, rb;
    double rightX;
    double angle;
    double angleTest[] = new double[10];
    int count = 0;
    double sum;
    double rA;
    double speed;
    Orientation angles;
    double lets;
    private BNO055IMU imu;
    DcMotor liftM;
    DcMotor extendM;
    DcMotor collectFlipperM;
    DcMotor collectSpinnerM;
    Servo dumpS;
    double correct;
    private ElapsedTime dumping = new ElapsedTime();

    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("lf");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf = hardwareMap.dcMotor.get("rf");
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb = hardwareMap.dcMotor.get("lb");
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb = hardwareMap.dcMotor.get("rb");
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu3");
        imu.initialize(parameters_IMU);

        liftM = hardwareMap.dcMotor.get("liftM");
        liftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendM = hardwareMap.dcMotor.get("extendM");
        extendM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collectFlipperM = hardwareMap.dcMotor.get("collectFlipperM");
        collectFlipperM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collectSpinnerM = hardwareMap.dcMotor.get("collectSpinnerM");
        collectSpinnerM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dumpS = hardwareMap.servo.get("dumpS");
        telemetry.addData("", "Successfully Initialized!");
    }

    @Override
    public void loop() {
        // OPERATOR

        extendM.setPower(gamepad2.left_stick_y);
        collectFlipperM.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        if     (gamepad2.dpad_up)   collectSpinnerM.setPower(0.6);
        else if(gamepad2.dpad_down) collectSpinnerM.setPower(-0.6);
        else if(gamepad2.dpad_left) collectSpinnerM.setPower(0);

        if(gamepad2.right_bumper){
            dumpS.setPosition(1);
            dumping.reset();
        }
        else {
            dumpS.setPosition(0);
        }

        if(dumping.milliseconds() < 100){
            liftM.setPower(0.3);
        }
        else {
            liftM.setPower(-gamepad2.right_stick_y);

        }


        // DRIVER
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI / 4;
        angle = angle + angles.firstAngle;
        rightX = gamepad1.right_stick_x;

        if(rightX == 0){
            if (count < 10) {
                angleTest[count] = angle;
                count++;
            }
            else if (count >= 10){
                 sum = (angleTest[1] + angleTest[2] + angleTest[3] + angleTest[4] + angleTest[0] + angleTest[5] + angleTest[6] + angleTest[7] + angleTest[8] + angleTest[9])/10;
                 if(sum > angle){
                     correct = sum - angle;
                     angle = angle + correct;
                 }
                 else if(angle > sum){
                     correct = angle - sum;
                     angle = angle - correct;
                 }
                 count = 0;
                 telemetry.addData("correct", correct);

                telemetry.addData("angle", angle);

                telemetry.addData("sum", sum);

                telemetry.update();





            }


        }
        lf.setPower(-(speed * (Math.sin(angle)) - rightX));
        rf.setPower((speed * (Math.cos(angle))) + rightX);
        lb.setPower(-(speed * (Math.cos(angle)) - rightX));
        rb.setPower((speed * (Math.sin(angle))) + rightX);

        telemetry.addData("lb pwoer", lb.getPower());
        telemetry.addData("rb pwoer", rb.getPower());
        telemetry.addData("rf pwoer", rf.getPower());
        telemetry.addData("lf pwoer", lf.getPower());
        telemetry.update();

    }
}