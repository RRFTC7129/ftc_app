package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="cf_2_balls_Worlds", group="LinearOPMode")

public class cf_2_balls_Worlds extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    private DcMotor catapult;
    //private DcMotor sweeper;
    private DcMotor lDrive1;
    private DcMotor lDrive2;
    private DcMotor rDrive1;
    private DcMotor rDrive2;
    private DcMotor sweeper;
    private Servo belt1;
    private Servo belt2;
    private Servo button;
    private Servo hopper;
    private Servo wheels;
    private TouchSensor touch;
    GyroSensor gyroSensor;
    ModernRoboticsI2cGyro gyro;
    double[]pastError = new double[5];
    double sum = 0;
    double lastTime = 0;
    double lastError = 0;
    double speed = 0;
    double time = 0;

    private void useEncoders() {
        rDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (lDrive1.isBusy()) {
            sleep(10);
        }
    }

    private void resetEncoders() {
        lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (lDrive1.isBusy()) {
            sleep(10);
        }
    }

    private void encoderDrive(double distance, double leftSpeed, double rightSpeed, int direction) throws InterruptedException {
        int ENCODER_CPR = 1120; // Encoder counts per Rev
        double gearRatio = 1.75; // [Gear Ratio]:1
        double circumference = 13.10; // Wheel circumference
        double ROTATIONS = distance / (circumference * gearRatio); // Number of rotations to drive
        double COUNTS = ENCODER_CPR * ROTATIONS; // Number of encoder counts to drive
        if (direction == 1)
            rDrive1.setTargetPosition(rDrive1.getCurrentPosition() + (int) COUNTS);
        else if (direction == -1)
            rDrive1.setTargetPosition(rDrive1.getCurrentPosition() - (int) COUNTS);
        if (direction == 1) {
            while (rDrive1.getCurrentPosition() < rDrive1.getTargetPosition() - 5 && opModeIsActive()) {
                drive(leftSpeed, rightSpeed);
                telemetry.addData("1. left speed", leftSpeed);
                telemetry.addData("2. right speed", rightSpeed);
                updateTelemetry(telemetry);
            }
            driveStop();
        } else if (direction == -1) {
            while (rDrive1.getCurrentPosition() > rDrive1.getTargetPosition() + 5 && opModeIsActive()) {
                drive(-leftSpeed, -rightSpeed);
                telemetry.addData("1. left speed", leftSpeed);
                telemetry.addData("2. right speed", rightSpeed);
                updateTelemetry(telemetry);
            }
            driveStop();
        } else {
            telemetry.addLine("Invalid direction");
            telemetry.update();
            sleep(10000);
        }
    }

    private void drive(double leftSpeed, double rightSpeed) {
        lDrive1.setPower(leftSpeed);
        lDrive2.setPower(leftSpeed);
        rDrive1.setPower(rightSpeed);
        rDrive2.setPower(rightSpeed);
    }

    private void driveStop() {
        drive(0,0);
    }

    // Function that utlizes the launchPosition, handleBall, and launch functions to fire and reload the catapult
    private void fire() throws InterruptedException {
        launchPosition();
        launchBall();
        launchPosition();
        sleep(1000);
        loadBall();
        launchBall();
        launchPosition();
    }

    // Resets catapult to the launch position
    private void launchPosition() throws InterruptedException {
        while (!touch.isPressed()) {
            catapult.setPower(0.5);
        }
        catapult.setPower(0);
    }

    // Function to load the catapult
    private void loadBall() throws InterruptedException {
        hopper.setPosition(.5);
        sleep(1000);
        hopper.setPosition(.8);
    }

    // Fires the ball
    private void launchBall() throws InterruptedException {
        catapult.setPower(1);
        sleep(800);
        catapult.setPower(0);
    }

    private void PIDGyroTurn (int targetHeading, double time){
        double error;
        double currentHeading;
        double kp = .0045;
        double ki = 0.0001;
        double kd = 0.002;
        double power;
        ElapsedTime runtime = new ElapsedTime();
        gyro.resetZAxisIntegrator();
        runtime.reset();
        sleep(250);
        lastError = targetHeading;
        lastTime = runtime.seconds();

        while (runtime.seconds() < time && opModeIsActive()){
            // positive angles are to the right, negative to the left.
            currentHeading = -gyro.getIntegratedZValue();
            // calculate error
            error = (targetHeading-currentHeading);

            // Set the power using PID control based off of the error.
            // Also uses a power offset of 0.2 to account for motor stall torque
            if (error > 0)
                power = .15+(error*kp)+(integral(error)*ki)+(derivative(error,runtime.seconds())*kd);
            else if (error < 0)
                power = -.15+(error*kp)+(integral(error)*ki)+(derivative(error,runtime.seconds())*kd);
            else
                power = 0;

            power = Range.clip(power, -1, 1);
            drive(0+power, 0-power);

            telemetry.addData("error", error);
            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("targetHeading", targetHeading);
            telemetry.addData("proportional",(error*kp));
            telemetry.addData("integral",(integral(error)*ki));
            telemetry.addData("derivative",(derivative(error,runtime.seconds())*kd));
            telemetry.addData("power", power);
            telemetry.update();
            // Wait to account for i2c bus lag for the gyro
            sleep(100);
        }
        driveStop();
    }

    private double integral(double error){
        sum = 0;
        pastError[4] = pastError[3];
        pastError[3] = pastError[2];
        pastError[2] = pastError[1];
        pastError[1] = pastError[0];
        pastError[0] = error;
        // Sum the past 5 error values
        // (Essentially take the integral of error vs time for the past 5 readings)
        for( double i : pastError) {
            sum += i;
        }
        return
                sum;
    }

    private double derivative(double error, double time){
        // Calculate the negative slope of the error vs time curve
        speed = (error-lastError)/(time-lastTime);
        lastError = error;
        lastTime = time;
        return
                speed;
    }

//    private void timedGyroTurn (int targetHeading, double time){
//        //boolean done = false;
//        double error;
//        double currentHeading;
//        double kp = .003;
//        double power;
//        ElapsedTime runtime = new ElapsedTime();
//        gyro.resetZAxisIntegrator();
//        runtime.reset();
//        sleep(250);
//
//        while (runtime.seconds() < time && opModeIsActive()){
//            currentHeading = -gyro.getIntegratedZValue();
//            error = (targetHeading-currentHeading);
//
//            if (error > 0)
//                power = .15+(error*kp);
//            else if (error < 0)
//                power = -.15+(error*kp);
//            else
//                power = 0;
//
//            drive(0+power, 0-power);
//
//            telemetry.addData("error", error);
//            telemetry.addData("power", power);
//            telemetry.addData("currentHeading", currentHeading);
//            telemetry.addData("targetHeading", targetHeading);
//            telemetry.update();
//        }
//        driveStop();
//    }

    private void setUpGyro() throws InterruptedException {

        // setup the Gyro
        // write some device information (connection info, name and type)
        // to the log file.
        hardwareMap.logDevices();
        // get a reference to our GyroSensor object.
        gyroSensor = hardwareMap.gyroSensor.get("gyro");
        gyro = (ModernRoboticsI2cGyro) gyroSensor;
        // calibrate the gyro.
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {
            sleep(50);
        }
        // End of setting up Gyro
    }

    public void runOpMode() throws InterruptedException {
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        belt1 = hardwareMap.servo.get("belt1");
        belt2 = hardwareMap.servo.get("belt2");
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        sweeper = hardwareMap.dcMotor.get("sweeper");
        catapult = hardwareMap.dcMotor.get("catapult");
        hopper = hardwareMap.servo.get("hopper");
        touch = hardwareMap.touchSensor.get("t");
        button = hardwareMap.servo.get("button");
        wheels = hardwareMap.servo.get("wheels");
        hopper.setPosition(0.8);
        button.setPosition(0.5);
        belt1.setPosition(.5);
        belt2.setPosition(.5);
        wheels.setPosition(.2);
        boolean center = false;
        boolean corner = false;
        boolean red = false;
        boolean blue = false;
        boolean nothing = false;
        int wait = 0;

        setUpGyro();
        resetEncoders();
        idle();
        useEncoders();
        idle();

        while (!red && !blue){
            telemetry.addLine("Press B for red alliance");
            telemetry.addLine("Press X for blue alliance");
            telemetry.addLine("Press dpad_right to turn vortex");
            telemetry.update();
            if (gamepad1.x)
                blue = true;
            else if (gamepad1.b)
                red = true;
        }
        sleep(1000);

        while (!center && !corner && !nothing) {
            telemetry.addLine("Press dpad_up to park center");
            telemetry.addLine("Press dpad_down to park ramp");
            telemetry.addLine("Press dpad_left to not park");
            telemetry.update();

            if (gamepad1.dpad_up)
                center = true;
            else if (gamepad1.dpad_down)
                corner = true;
            else if (gamepad1.dpad_left)
                nothing = true;
        }

        while (!isStarted()){
            if (blue)
                telemetry.addLine("Blue alliance");
            if (red)
                telemetry.addLine("Red alliance");
            if (center)
                telemetry.addLine("Center End");
            if (corner)
                telemetry.addLine("Corner End");
            if (nothing)
                telemetry.addLine("No End");
            if (gamepad1.y) {
                wait = wait + 1;
                telemetry.addData("Wait time", wait*1000);
                sleep(100);
            }
            telemetry.update();
        }

        waitForStart();
        wait = wait * 1000;
        timer.reset();

        if (red) {
            // Drive forward from wall
            encoderDrive(/*distance*/35, /*leftSpeed*/0.5, /*rightSpeed*/0.5, /*direction*/1);
            sleep(1000);
            // shoot both balls
            fire();
            sleep(wait);

            if (center) {
                encoderDrive(/*distance*/11, /*leftSpeed*/1, /*rightSpeed*/1, /*direction*/1);
                PIDGyroTurn(-135,4);
                encoderDrive(/*distance*/12, /*leftSpeed*/1, /*rightSpeed*/1, /*direction*/-1);
            }
            if (corner) {
                PIDGyroTurn(-45,2);

                encoderDrive(/*distance*/50, /*leftSpeed*/1, /*rightSpeed*/1, /*direction*/1);
                PIDGyroTurn(-45,2);
                encoderDrive(/*distance*/20, /*leftSpeed*/1, /*rightSpeed*/1, /*direction*/1);
            }

        }

        if (blue) {
            // Drive forward from wall
            encoderDrive(/*distance*/35, /*leftSpeed*/0.5, /*rightSpeed*/0.5, /*direction*/1);
            sleep(1000);
            fire();
            sleep(wait);

            if (center) {
                encoderDrive(/*distance*/11, /*leftSpeed*/1, /*rightSpeed*/1, /*direction*/1);
                PIDGyroTurn(135,4);
                encoderDrive(/*distance*/12, /*leftSpeed*/1, /*rightSpeed*/1, /*direction*/-1);
            }
            if (corner) {
                PIDGyroTurn(45,2);
                encoderDrive(/*distance*/50, /*leftSpeed*/1, /*rightSpeed*/1, /*direction*/1);
                PIDGyroTurn(50,2);
                encoderDrive(/*distance*/20, /*leftSpeed*/1, /*rightSpeed*/1, /*direction*/1);
            }
        }
    }
}