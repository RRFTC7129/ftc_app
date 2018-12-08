package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous (name="tempAuto",group="Opmode")

public class tempAuto extends LinearOpMode {
    DcMotor collectFlipperM, lb, rf, lf, rb, liftM, extendM, collectSpinnerM;
    double angle;
    Servo dumpS, lock;
    private BNO055IMU imu;
    double speed;
    Orientation angles;
    double rightX;
    int pos = 0;
int realDist, lastDist;
    private ElapsedTime dumping = new ElapsedTime();


    public void Drop(){

        lock.setPosition(0);

        liftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftM.setPower(0.5);
//        telemetry.addData("going into loop"," ");
//        telemetry.update();
//        sleep(1000);
        while (opModeIsActive() && liftM.getCurrentPosition() < 1060) {

            //telemetry.addData("current pos", liftM.getCurrentPosition());
            //telemetry.update();
        }
//        telemetry.addData("exited loop"," ");
//        telemetry.update();
//        sleep(1000);
        liftM.setPower(0.2);

    }

    public  void  scoreMarker(){

        collectFlipperM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        collectFlipperM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        collectFlipperM.setPower(-0.5);

        while (opModeIsActive() && collectFlipperM.getCurrentPosition() > -650){

            telemetry.addData("current pos", collectFlipperM.getCurrentPosition());
            telemetry.update();



        }
        collectFlipperM.setPower(0);
        sleep(500);
        collectSpinnerM.setPower(0.5);
        collectFlipperM.setPower(0.5);
        while (opModeIsActive() && collectFlipperM.getCurrentPosition() < -30){

            telemetry.addData("current pos", collectFlipperM.getCurrentPosition());
            telemetry.update();

        }

        collectSpinnerM.setPower(0);
        collectFlipperM.setPower(0);




    }

    public void encoderDriveForward(int distance, double x, double y){
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(distance*55 > rf.getCurrentPosition() && -distance*55 < lf.getCurrentPosition() && distance*55 > rb.getCurrentPosition() && -distance*55 < lb.getCurrentPosition()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            //0, -0.5 is to drive forward
            //-0.5, 0
            speed = Math.hypot(x, y);
            angle = Math.atan2(y, x) + Math.PI / 4;
            angle = angle + angles.firstAngle;
            rightX = 0;

            lf.setPower((speed * (Math.sin(angle)) + rightX));
            rf.setPower(-(speed * (Math.cos(angle))) - rightX);
            lb.setPower((speed * (Math.cos(angle)) + rightX));
            rb.setPower(-(speed * (Math.sin(angle))) - rightX);

            telemetry.addData("current pos",rf.getCurrentPosition());
            telemetry.addData("current pos",rb.getCurrentPosition());
            telemetry.addData("current pos",lf.getCurrentPosition());
            telemetry.addData("current pos",lb.getCurrentPosition());
            telemetry.update();
        }
        rf.setPower(-0);
        lf.setPower(-0);
        rb.setPower(0);
        lb.setPower(0);

    }

    public void encoderDriveRight(int distance, double x, double y){
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(-distance*55 < rf.getCurrentPosition() && distance*55 > lf.getCurrentPosition() && -distance*55 < rb.getCurrentPosition() && distance*55 > lb.getCurrentPosition()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            //0, -0.5 is to drive forward
            //-0.5, 0
            speed = Math.hypot(x, y);
            angle = Math.atan2(y, x) + Math.PI / 4;
            angle = angle + angles.firstAngle;
            rightX = 0;

            lf.setPower((speed * (Math.sin(angle)) + rightX));
            rf.setPower(-(speed * (Math.cos(angle))) - rightX);
            lb.setPower((speed * (Math.cos(angle)) + rightX));
            rb.setPower(-(speed * (Math.sin(angle))) - rightX);

            telemetry.addData("current pos",rf.getCurrentPosition());
            telemetry.addData("current pos",rb.getCurrentPosition());
            telemetry.addData("current pos",lf.getCurrentPosition());
            telemetry.addData("current pos",lb.getCurrentPosition());
            telemetry.update();
        }
        rf.setPower(-0);
        lf.setPower(-0);
        rb.setPower(0);
        lb.setPower(0);

    }

    public void encoderDriveBack(int distance, double x, double y){

        while(-distance*55 > rf.getCurrentPosition() && distance*55 < lf.getCurrentPosition() && -distance*55 > rb.getCurrentPosition() && distance*55 < lb.getCurrentPosition()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            //0, -0.5 is to drive forward
            //-0.5, 0
            speed = Math.hypot(x, y);
            angle = Math.atan2(y, x) + Math.PI / 4;
            angle = angle + angles.firstAngle;
            rightX = 0;

            lf.setPower((speed * (Math.sin(angle)) + rightX));
            rf.setPower(-(speed * (Math.cos(angle))) - rightX);
            lb.setPower((speed * (Math.cos(angle)) + rightX));
            rb.setPower(-(speed * (Math.sin(angle))) - rightX);

            telemetry.addData("current pos",rf.getCurrentPosition());
            telemetry.addData("current pos",rb.getCurrentPosition());
            telemetry.addData("current pos",lf.getCurrentPosition());
            telemetry.addData("current pos",lb.getCurrentPosition());
            telemetry.update();
        }
        rf.setPower(-0);
        lf.setPower(-0);
        rb.setPower(0);
        lb.setPower(0);

    }

    public void imuTurn(double degreesToTurn) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = -angles.firstAngle + 180;
        double targetHeading = degreesToTurn + currentHeading;

        targetHeading += targetHeading > 360 ? -360 :
                targetHeading <   0 ?  360 : 0;

        while (Math.abs(degreesToTurn) > 2) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = -angles.firstAngle + 180;
            degreesToTurn = targetHeading - currentHeading;

            double power = Range.clip(Math.signum(degreesToTurn) * (0.25 + (Math.abs(degreesToTurn) / 360)), -1, 1);
//            op.telemetry.addData("DegreesToTurn", degreesToTurn);
//            op.telemetry.addData("currentHeading", currentHeading);
//            op.telemetry.addData("targetHeading", targetHeading);
//            op.telemetry.addData("POWA", power);
//            System.out.println("DegreesToTurn: " + degreesToTurn + " -- currentHeading: " + currentHeading + " -- POWA: " + power);
//            op.telemetry.update();
            rf.setPower(power);
            rb.setPower(power);
            lf.setPower(power);
            lb.setPower(power);
            // LEFT TURN
        }
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
    }

    public void selection(boolean side, boolean alliance) {
        boolean B;
        boolean X;
        boolean A;
        boolean Y;
        boolean rBumper;
        boolean lBumper;

        Y = gamepad1.y;
        A = gamepad1.a;
        telemetry.addData("Press B ", "for red alliance");
        telemetry.addData("Press X ", "for blue alliance");
        telemetry.update();
        while (!opModeIsActive() && !Y) {
            Y = gamepad1.y;
            B = gamepad1.b;
            X = gamepad1.x;

            if (B) {
                telemetry.addData("red ","alliance");
                telemetry.addData("press Y to ","move on");
                telemetry.update();
                alliance = true;
            }
            else if(X){
                telemetry.addData("blue ","alliance");
                telemetry.addData("press Y to ","move on");
                alliance = false;

                telemetry.update();
            }

        }

        while (!opModeIsActive() && !A){

            A = gamepad1.y;
            rBumper = gamepad1.right_bumper;
            lBumper = gamepad1.left_bumper;

            if (rBumper) {
                telemetry.addData("Crater ","side");
                telemetry.addData("press A to ","move on");
                side = false;
                telemetry.update();
            }
            else if(lBumper){
                telemetry.addData("Crater ","side");
                telemetry.addData("press A to ","move on");
                side = true;

                telemetry.update();
            }

        }


    }

    public  void toBlock(){
if (pos == 0){
    imuTurn(-25);
    encoderDriveForward(38, 0.5, 0.5);
    imuTurn(70);
    encoderDriveForward(20, -0.5, 0.5);
    imuTurn(-90);
    encoderDriveForward(24, -0.5, 0.5);

    encoderDriveForward(10, -0.5, -0.5);
    scoreMarker();
    encoderDriveForward(45, -0.5, -0.5);

    encoderDriveRight(16, 0.5, -0.5);

    encoderDriveForward(30, -0.5, -0.5);






}

else if (pos == 1){
encoderDriveForward(46, 0, 0.5);
    scoreMarker();
    imuTurn(-45);

    encoderDriveForward(24, -0.5, 0.5);

    encoderDriveForward(45, -0.5, -0.5);

    encoderDriveRight(16, 0.5, -0.5);

    imuTurn(45);

    encoderDriveRight(25, 0, -0.5);

    imuTurn(-55);

    encoderDriveForward(20, -0.5, -0.5);









}

else if(pos == 2){
    imuTurn(30);
    encoderDriveForward(50, -0.5, 0.5);
    imuTurn(-75);
    encoderDriveForward(6, -0.5, 0.5);

    encoderDriveForward(20, 0.5, 0.5);

    scoreMarker();

    encoderDriveForward(45, -0.5, -0.5);

    encoderDriveRight(16, 0.5, -0.5);

    imuTurn(45);

    encoderDriveRight(44, 0, -0.5);

    imuTurn(-55);

    encoderDriveForward(20, -0.5, -0.5);

}
        dumpS.setPosition(1);
sleep(1000);




    }

    public  void toDeopot(){




    }

    @Override
    public void runOpMode() {
pos = 0;

        rb = hardwareMap.dcMotor.get("rb");
        rb.setPower(0);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lb = hardwareMap.dcMotor.get("lb");
        lb.setPower(0);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rf = hardwareMap.dcMotor.get("rf");
        rf.setPower(0);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf = hardwareMap.dcMotor.get("lf");
        lf.setPower(0);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collectFlipperM = hardwareMap.dcMotor.get("collectFlipperM");
        collectFlipperM.setPower(0);
        collectFlipperM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftM = hardwareMap.dcMotor.get("liftM");
        liftM.setPower(0);
        liftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendM = hardwareMap.dcMotor.get("extendM");
        extendM.setPower(0);
        extendM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collectSpinnerM = hardwareMap.dcMotor.get("collectSpinnerM");
        collectSpinnerM.setPower(0);
        collectSpinnerM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dumpS = hardwareMap.servo.get("dumpS");
        dumpS.setPosition(0);

        lock = hardwareMap.servo.get("lock");
       // lock.setPosition(0);

        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu3");
        imu.initialize(parameters_IMU);


        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();



        sleep(50);
//
//        Drop();
//        encoderDriveRight(9, 0.5, 0);
//
//        liftM.setPower(0);
//
//        sleep(1000);
//encoderDriveForward(24, 0, 0.5);
        toBlock();


    }
}