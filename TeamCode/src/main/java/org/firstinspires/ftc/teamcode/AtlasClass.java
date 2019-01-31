package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.drm.DrmStore.Playback.STOP;
import static android.net.NetworkInfo.DetailedState.IDLE;


//import org.firstinspires.ftc.robotcore.external.Telemetry;
@Autonomous(name="snedHelpBOYS", group="Opmode")
@Disabled

public class AtlasClass extends LinearOpMode {

    DcMotor collectFlipperM, lb, rf, lf, rb, liftM, extendM, collectSpinnerM;
    double angle;
    Servo dumpS, lock;
    public BNO055IMU imu;
    double speed;
    Orientation angles;
    double rightX;
    boolean side, marker, two;
    boolean trys = false;
    int wait = 0;
    int pos = 0;
    String goldPosition;
    Boolean goldFound = false;
    Boolean goldAligned = false;
    private ColorSensor color;
    private int redMargin = 160;
    float rfPow, lfPow, rbPow, lbPow, liftPower, dumpPos, collectPower, collectFlipPower, extendPower;

    private ElapsedTime buttoning = new ElapsedTime();

    private ElapsedTime popOFF = new ElapsedTime();
    public GoldAlignDetector detector;

    private enum LiftStage {
        IDLE, LIFTING, SLOWING, STOP
    }

    private LiftStage currentStage = LiftStage.IDLE;


    private ElapsedTime dumping = new ElapsedTime();


    AtlasClass(HardwareMap hardwareMap) throws InterruptedException {
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
        lock.setPosition(0);

        color = hardwareMap.get(ColorSensor.class, "color");


        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu3");
        imu.initialize(parameters_IMU);


    }

    //Teleop Functions\\
    public Object[] drive() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        angle = angle + angles.firstAngle;
        rightX = gamepad1.right_stick_x;


        lfPow = (float)(-(speed * (Math.sin(angle)) + rightX));
        rfPow = (float)((speed * (Math.cos(angle))) - rightX);
        lbPow = (float)(-(speed * (Math.cos(angle)) + rightX));
        rbPow = (float)((speed * (Math.sin(angle))) - rightX);

        return new Object[] {rfPow, lfPow, rbPow, lbPow};



    }

    public Object[] dumpAndLift() { //note to self, think i've fixed up the lifting, just need to put the setting of servo position in
        if (gamepad2.right_bumper || gamepad1.right_bumper) {
            dumpPos = (float)1;
            dumping.reset();
        } else {
            dumpPos = (float)0;
        }

        if (dumping.milliseconds() < 100 || gamepad2.left_bumper) {
            liftPower = (float) 0.2;
        }

        if (gamepad2.x && popOFF.milliseconds() > 300) {
            trys = true;
            popOFF.reset();
        } else if (gamepad2.b && popOFF.milliseconds() > 300) {
            trys = false;
            popOFF.reset();
        }

        if (trys) {
            switch (currentStage) {
                case IDLE:
                    currentStage = LiftStage.LIFTING;
                    break;
                case LIFTING:
                    if (color.red() < redMargin) liftPower = (float) 0.9;
                    else currentStage = LiftStage.SLOWING;
                    break;
                case SLOWING:
                    if (color.red() > redMargin) liftPower = (float) 0.1;

                    else currentStage = LiftStage.STOP;
                    break;
                case STOP:

                    if (buttoning.milliseconds() > 200 && gamepad2.a) {
                        liftPower = (float) 0.1;
                        buttoning.reset();
                    } else if (buttoning.milliseconds() > 200 && gamepad2.y) {
                        liftPower = (float) 0.6;
                        buttoning.reset();
                    } else {
                        liftPower = (float) 0.2;
                    }


                    break;
            }
        } else {
            currentStage = LiftStage.IDLE;
            liftPower = gamepad2.left_stick_y;
        }
        return new Object[] {liftPower, dumpPos};


    }

    public float collectionSpin(){
        collectPower = gamepad2.left_trigger - gamepad2.right_trigger;
        return collectPower;
    }

    public float collectionFlip() {
        if     (gamepad2.dpad_up)   collectFlipPower = (float)-0.6;
        else if(gamepad2.dpad_down) collectFlipPower = (float)0.6;
        else if(gamepad2.dpad_left) collectFlipPower = (float)0;
        return collectFlipPower;
    }

    public float extend(){         extendPower = gamepad2.left_stick_y;
    return extendPower;}

//Autonomous Functions\\

    public void Drop(){
        liftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lock.setPosition(1);


        sleep(500);

        liftM.setPower(0.5);
        sleep(2000);

////        telemetry.addData("going into loop"," ");
////        telemetry.update();
////        sleep(1000);
//
//        while (opModeIsActive() && liftM.getCurrentPosition() < 1055) {
//
//            telemetry.addData("current pos", liftM.getCurrentPosition());
//            telemetry.update();
//        }
////        telemetry.addData("exited loop"," ");
////        telemetry.update();
////        sleep(1000);

//        liftM.setPower(0.2);

    }

    public  void  scoreMarker(){



        collectFlipperM.setPower(0.3);
        sleep(100);

        collectSpinnerM.setPower(1);

        sleep(1000);

        collectSpinnerM.setPower(0);
        collectFlipperM.setPower(0);




    }


    public void encoderDriveForward(int distance, double x, double y){ //Distance in inches, x and y as direction values, both between 1- and 1
        //reset encoder values to 0
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Run without encoders so we can get values properly
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Run the loop as long as all of the wheels are at a position less than their target position
        //55 encoder ticks per inch, so convert the inch input to encoder ticks
        while(distance*55 > rf.getCurrentPosition() && -distance*55 < lf.getCurrentPosition() && distance*55 > rb.getCurrentPosition() && -distance*55 < lb.getCurrentPosition()) {
            //intake gyro values for drift correction
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            //calculate speed based off of the values given
            speed = Math.hypot(x, y);
            //calculate direction based off of the values given
            angle = Math.atan2(y, x) + Math.PI / 4;
            //use gyro correction
            angle = angle + angles.firstAngle;
            //Spinning value.  This code drives striaght so it is always zero
            rightX = 0;
            //set motor speeds
            lf.setPower((speed * (Math.sin(angle)) + rightX));
            rf.setPower(-(speed * (Math.cos(angle))) - rightX);
            lb.setPower((speed * (Math.cos(angle)) + rightX));
            rb.setPower(-(speed * (Math.sin(angle))) - rightX);
            //telemetry data for troubleshooting
            telemetry.addData("current pos",rf.getCurrentPosition());
            telemetry.addData("current pos",rb.getCurrentPosition());
            telemetry.addData("current pos",lf.getCurrentPosition());
            telemetry.addData("current pos",lb.getCurrentPosition());
            telemetry.update();
        }
        //exit loop and stop motors
        rf.setPower(0);
        lf.setPower(0);
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

    public void selection() {

        telemetry.addData("Press B ", "for depot side");
        telemetry.addData("Press X ", "for crater side");
        telemetry.update();
        while (!opModeIsActive() && !gamepad1.y) {


            if (gamepad1.b) {
                telemetry.addData("depot ", "side");
                telemetry.addData("press Y to ", "move on");
                telemetry.update();
                side = true;
            } else if (gamepad1.x) {
                telemetry.addData("crater ", "side");
                telemetry.addData("press Y to ", "move on");
                side = false;

                telemetry.update();
            }

        }
        sleep(1000);
        if (side) {
            telemetry.addData("Press B ", "for two block");
            telemetry.addData("Press X ", "for one");
            telemetry.update();
            while (!opModeIsActive() && !gamepad1.a) {



                if (gamepad1.b) {
                    telemetry.addData("Two ", "block");
                    telemetry.addData("press A to ", "move on");
                    two = true;
                    telemetry.update();
                } else if (gamepad1.x) {
                    telemetry.addData("One ", "block");
                    telemetry.addData("press A to ", "move on");
                    two = false;

                    telemetry.update();
                }

            }



        }
        else if(!side){
            telemetry.addData("Press B ", "for marker");
            telemetry.addData("Press X ", "for no marker");
            telemetry.update();
            while (!opModeIsActive() && !gamepad1.a) {



                if (gamepad1.b) {
                    telemetry.addData("Yes ", "marker");
                    telemetry.addData("press A to ", "move on");
                    marker = true;
                    telemetry.update();
                } else if (gamepad1.x) {
                    telemetry.addData("No ", "Marker");
                    telemetry.addData("press A to ", "move on");
                    marker = false;

                    telemetry.update();
                }

            }


        }

        telemetry.addData("Press ","Dpad up for more wait");
        telemetry.addData("Press ","Dpad down for not more wait");
        telemetry.update();
        while(!opModeIsActive()){
            if(gamepad1.dpad_up){
                wait = wait + 1000;
                sleep(300);
            }

            else if (gamepad1.dpad_down){
                wait = wait - 1000;
                sleep(300);
                if(wait < 0){
                    wait = 0;
                }

            }

            telemetry.addData("Waiting for:", wait);
            telemetry.update();

        }
    }

    public  void toBlockTwo(){
        if (pos == 0){
            imuTurn(-25);
            encoderDriveForward(36, 0.5, 0.5);
            imuTurn(70);
            encoderDriveForward(20, -0.5, 0.5);
            imuTurn(-90);
            encoderDriveForward(30, -0.5, 0.5);

            encoderDriveRight(5, 0.5, -0.5);
            scoreMarker();
            encoderDriveForward(45, -0.5, -0.5);

            encoderDriveRight(16, 0.5, -0.5);

            encoderDriveForward(30, -0.5, -0.5);






        }

        else if (pos == 1){
            encoderDriveForward(46, 0, 0.5);
            imuTurn(-45);

            encoderDriveForward(36, -0.5, 0.5);
            encoderDriveRight(6, 0.5, -0.5);

            scoreMarker();



            encoderDriveForward(45, -0.5, -0.5);

            encoderDriveRight(16, 0.5, -0.5);

            imuTurn(40);

            encoderDriveRight(22, 0, -0.5);

            imuTurn(-55);

            encoderDriveForward(20, -0.5, -0.5);

        }

        else if(pos == 2){
            imuTurn(30);
            encoderDriveForward(47, -0.5, 0.5);
            imuTurn(-75);
            encoderDriveForward(10, -0.5, 0.5);

            encoderDriveRight(7, 0.5, -0.5);


            encoderDriveForward(20, 0.5, 0.5);

            scoreMarker();

            encoderDriveForward(45, -0.5, -0.5);

            encoderDriveRight(16, 0.5, -0.5);

            imuTurn(40);

            encoderDriveRight(44, 0, -0.5);

            imuTurn(-55);

            encoderDriveForward(20, -0.5, -0.5);

        }
        dumpS.setPosition(1);
        sleep(1000);




    }

    public void initAuto(){
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 200; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
    }

    public  void toBlockPark(){
        if (pos == 0){
            imuTurn(-25);
            encoderDriveForward(36, 0.5, 0.5);
            imuTurn(70);
            encoderDriveForward(20, -0.5, 0.5);
            imuTurn(-90);
            encoderDriveForward(30, -0.5, 0.5);

            encoderDriveRight(5, 0.5, -0.5);
            scoreMarker();
            encoderDriveForward(70, -0.5, -0.5);








        }
        else if (pos == 1){

            encoderDriveForward(46, 0, 0.5);
            imuTurn(-45);

            encoderDriveForward(36, -0.5, 0.5);
            encoderDriveRight(6, 0.5, -0.5);

            scoreMarker();

//            encoderDriveForward(24, -0.5, 0.5);
//            encoderDriveRight(7, 0.5, -0.5);

            encoderDriveForward(72, -0.5, -0.5);

//            encoderDriveRight(16, 0.5, -0.5);
//
//            imuTurn(45);
//
//            encoderDriveRight(25, 0, -0.5);
//
//            imuTurn(-55);
//
//            encoderDriveForward(20, -0.5, -0.5);









        }

        else if(pos == 2){
            imuTurn(30);
            encoderDriveForward(47, -0.5, 0.5);
            imuTurn(-75);
            encoderDriveForward(10, -0.5, 0.5);

            encoderDriveRight(7, 0.5, -0.5);


            encoderDriveForward(20, 0.5, 0.5);


            scoreMarker();

            encoderDriveForward(70, -0.5, -0.5);


//            encoderDriveForward(45, -0.5, -0.5);
//
//            encoderDriveRight(16, 0.5, -0.5);
//
//            imuTurn(45);
//
//            encoderDriveRight(44, 0, -0.5);
//
//            imuTurn(-55);
//
//            encoderDriveForward(20, -0.5, -0.5);

        }
        dumpS.setPosition(1);
        sleep(1000);




    }

    public  void toBlockCraterPark(){
        if (pos == 0){
            imuTurn(-30);
            encoderDriveForward(38, 0.5, 0.5);
//            imuTurn(70);
//            encoderDriveForward(20, -0.5, 0.5);
//            imuTurn(-90);
//            encoderDriveForward(24, -0.5, 0.5);
//
//            encoderDriveForward(10, -0.5, -0.5);
//            scoreMarker();
//            encoderDriveForward(45, -0.5, -0.5);
//
//            encoderDriveRight(16, 0.5, -0.5);
//
//            encoderDriveForward(30, -0.5, -0.5);






        }

        else if (pos == 1){
            imuTurn(-10);
            encoderDriveForward(46, 0, 0.5);
//            scoreMarker();
//            imuTurn(-45);
//
//            encoderDriveForward(22, -0.5, 0.5);
//
//            encoderDriveForward(45, -0.5, -0.5);
//
//            encoderDriveRight(16, 0.5, -0.5);
//
//            imuTurn(45);
//
//            encoderDriveRight(25, 0, -0.5);
//
//            imuTurn(-55);
//
//            encoderDriveForward(20, -0.5, -0.5);









        }

        else if(pos == 2){
            imuTurn(25);
            encoderDriveForward(50, -0.5, 0.5);
//            imuTurn(-75);
//            encoderDriveForward(6, -0.5, 0.5);
//
//            encoderDriveForward(20, 0.5, 0.5);
//
//            scoreMarker();
//
//            encoderDriveForward(45, -0.5, -0.5);
//
//            encoderDriveRight(16, 0.5, -0.5);
//
//            imuTurn(45);
//
//            encoderDriveRight(44, 0, -0.5);
//
//            imuTurn(-55);
//
//            encoderDriveForward(20, -0.5, -0.5);

        }
        dumpS.setPosition(1);
        sleep(1000);




    }

    public  void toBlockCraterMarker(){

        encoderDriveForward(15, 0, 0.5);
        imuTurn(-90);
        encoderDriveForward(40, 0.5, 0);
        imuTurn(-45);
        encoderDriveForward(26, 0.5, 0.5);

        encoderDriveRight(6, -0.5, -0.5);

        sleep(500);
        encoderDriveForward(44, 0.5, -0.5);

        scoreMarker();

        encoderDriveForward(44, -0.5, 0.5);

        imuTurn(55);


//        encoderDriveForward(8, 0.5, 0.5);
//        sleep(1000);
//        encoderDriveForward(8, 0.5, -0.5);
//        sleep(1000);
//        encoderDriveForward(8, -0.5, 0.5);
//        sleep(1000);
//        encoderDriveForward(8, -0.5, -0.5);





        if (pos == 0){
            encoderDriveForward(17, -0.5, 0);
//            encoderDriveForward(20, -0.5, 0.5);
//            imuTurn(-90);
//            encoderDriveForward(24, -0.5, 0.5);
//
//            encoderDriveForward(10, -0.5, -0.5);
//            scoreMarker();
//            encoderDriveForward(45, -0.5, -0.5);
//
//            encoderDriveRight(16, 0.5, -0.5);
//
//            encoderDriveForward(30, -0.5, -0.5);






        }

        else if (pos == 1){
            encoderDriveForward(42, -0.5, 0);

//            scoreMarker();
//            imuTurn(-45);
//
//            encoderDriveForward(22, -0.5, 0.5);
//
//            encoderDriveForward(45, -0.5, -0.5);
//
//            encoderDriveRight(16, 0.5, -0.5);
//
//            imuTurn(45);
//
//            encoderDriveRight(25, 0, -0.5);
//
//            imuTurn(-55);
//
//            encoderDriveForward(20, -0.5, -0.5);









        }

        else if(pos == 2){
            encoderDriveForward(58, -0.5, 0);
//            imuTurn(-75);
//            encoderDriveForward(6, -0.5, 0.5);
//
//            encoderDriveForward(20, 0.5, 0.5);
//
//            scoreMarker();
//
//            encoderDriveForward(45, -0.5, -0.5);
//
//            encoderDriveRight(16, 0.5, -0.5);
//
//            imuTurn(45);
//
//            encoderDriveRight(44, 0, -0.5);
//
//            imuTurn(-55);
//
//            encoderDriveForward(20, -0.5, -0.5);

        }

        imuTurn(-85);
        encoderDriveForward(23, 0, 0.5);
        dumpS.setPosition(1);
        sleep(1000);




    }

    public void vision(){
        detector.enable(); // Start the detector!


        sleep(2000);
        goldAligned = detector.getAligned();
        goldFound = detector.isFound();


        detector.disable();


        if (goldFound) { // gold is center or right
            if (goldAligned) { // gold is right
                goldPosition = "right";
                pos = 2;
            }
            else { // gold is center
                goldPosition = "center";
                pos = 1;
            }
        }
        else { // gold is left
            goldPosition = "left";
            pos = 0;
        }
        telemetry.addData("Gold Position: ", "" + goldPosition);
        //telemetry.update();


        telemetry.addData("pos:", pos);
        telemetry.update();

    }







    //never actually run this, just threw it in so that we can call this a linear opMode for telemetry data and controller inputs
    @Override
    public void runOpMode() {


    }
}