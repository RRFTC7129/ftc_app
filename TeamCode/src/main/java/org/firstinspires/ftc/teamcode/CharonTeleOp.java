package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Cheerios Teleop", group="OpMode")
public class CharonTeleOp extends OpMode {

    public DcMotor lf, rf, lb, rb;
    double rightX;
    double angle;
    double angleTest[] = new double[10];
    int count = 0;
    double sum;
    double rA;
    double speed;
    int target, tar;
    Orientation angles;
    double lets;
    private BNO055IMU imu;
    DcMotor liftM;
    DcMotor extendM;
    DcMotor collectFlipperM;
    DcMotor collectSpinnerM;
    Servo dumpS, lock;
    private ColorSensor color;
    boolean first = true;
    int x = 4;
    int y = 1;
    int z = -1;
    int a = 0;
    int q = -1;
    int p = 1;
    private int redMargin = 160;
    double correct;
    boolean trys = false;
    int base;
    private ElapsedTime dumping = new ElapsedTime();
    private ElapsedTime buttoning = new ElapsedTime();
    private ElapsedTime popOFF = new ElapsedTime();
    private ElapsedTime revThat = new ElapsedTime();


    private enum LiftStage {
        IDLE, LIFTING, SLOWING, STOP
    }
    private LiftStage currentStage = LiftStage.IDLE;

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


        lock = hardwareMap.servo.get("lock");
        lock.setPosition(0);

        liftM = hardwareMap.dcMotor.get("liftM");
        liftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendM = hardwareMap.dcMotor.get("extendM");
        extendM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collectFlipperM = hardwareMap.dcMotor.get("collectFlipperM");
        collectFlipperM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collectSpinnerM = hardwareMap.dcMotor.get("collectSpinnerM");
        collectSpinnerM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dumpS = hardwareMap.servo.get("dumpS");
        liftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        color = hardwareMap.get(ColorSensor.class, "color");

        telemetry.addData("", "Successfully Initialized!");

        liftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.update();

    }

    @Override
    public void loop() {
        // OPERATOR

        extendM.setPower(gamepad2.right_stick_y);
        collectSpinnerM.setPower(q*gamepad2.right_trigger + p*gamepad2.left_trigger);

        if     (gamepad2.dpad_down)   collectFlipperM.setPower(q*0.6);
        else if(gamepad2.dpad_up) collectFlipperM.setPower(p*0.6);
        else collectFlipperM.setPower(0);

        if(gamepad2.right_bumper || gamepad1.right_bumper){
            dumpS.setPosition(1);
            dumping.reset();
        }
        else {
            dumpS.setPosition(0);
        }

        if(dumping.milliseconds() < 100 || gamepad2.left_bumper){
            liftM.setPower(0.2);
        }

        if (gamepad2.x && popOFF.milliseconds() > 300){
            trys = true;
            popOFF.reset();
        }
        else if (gamepad2.b && popOFF.milliseconds() > 300){
            trys = false;
            popOFF.reset();
        }

        if(trys) {
            switch(currentStage) {
                case IDLE:
                    currentStage = LiftStage.LIFTING;
                    break;
                case LIFTING:
                    if(color.red() < redMargin) liftM.setPower(0.9);
                    else currentStage = LiftStage.SLOWING;
                    break;
                case SLOWING:
                    if(color.red() > redMargin) liftM.setPower(0);
                    else if(color.red() > redMargin && first) liftM.setPower(0);

                    else currentStage = LiftStage.STOP;
                    break;
                case STOP:

                    if(buttoning.milliseconds() > 200 && gamepad2.a){
                        liftM.setPower(0);
                        buttoning.reset();
                    }
                    else if (buttoning.milliseconds() > 200 && gamepad2.y){
                        liftM.setPower(0.6);
                        buttoning.reset();
                    }
                    else {
                        liftM.setPower(0.2);
                    }
                    if(first){
                        first = false;
                        target = liftM.getCurrentPosition();
                    }

                    break;
            }
        } else {
            currentStage = LiftStage.IDLE;
            liftM.setPower(-gamepad2.left_stick_y);
        }
//        if(gamepad2.y && buttoning.milliseconds() > 300){
//            target = target + 50;
//            buttoning.reset();
//        }
//        else  if(gamepad2.a && buttoning.milliseconds() > 300){
//            target = target - 50;
//buttoning.reset();
//        }
        if(gamepad1.b){
            x = 4;

        }
        else if (gamepad1.x){
            x = 2;
        }

        if(gamepad1.right_bumper){
            y = 1;
        }

        else if(gamepad1.left_bumper){
            y = -1;
        }
        if (gamepad1.left_trigger > 0.5){
            z = -1;
        }
        else if (gamepad1.right_trigger > 0.5){
            z = 1;
        }
        if(gamepad1.dpad_up){
            a = 1;
        }
        else if(gamepad1.dpad_down){
            a = 0;
        }


        if (gamepad1.y && revThat.milliseconds() > 300){
            p = -1;
            q = 1;
            revThat.reset();
        }
        else if(gamepad1.y && revThat.milliseconds() < 300){
            p = 1;
            q = -1;
            revThat.reset();
        }
        // DRIVER
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (Math.PI / x)*y;
        angle = angle; //+ (angles.firstAngle*a);
        rightX = gamepad1.right_stick_x*z;

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
//                telemetry.addData("correct", correct);
//
//                telemetry.addData("angle", angle);
//
//                telemetry.addData("sum", sum);
//
//                telemetry.update();

            }
        }


        lf.setPower(z*(-(speed * (Math.sin(angle)) - rightX)));
        rf.setPower(z*((speed * (Math.cos(angle))) + rightX));
        lb.setPower(z*(-(speed * (Math.cos(angle)) - rightX)));
        rb.setPower(z*((speed * (Math.sin(angle))) + rightX));
        telemetry.addData("target", target);
        telemetry.addData("currrent pos", liftM.getCurrentPosition());
        telemetry.addData("first", first);



//        telemetry.addData("lb pwoer", lb.getPower());
//        telemetry.addData("rb pwoer", rb.getPower());
//        telemetry.addData("rf pwoer", rf.getPower());
//        telemetry.addData("lf pwoer", lf.getPower());
//        telemetry.update();
    }
}