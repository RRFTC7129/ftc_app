package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
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
import org.opencv.core.Rect;

import java.io.FileWriter;
import java.io.PrintWriter;

@Autonomous (name="Passing Test",group="Opmode")

public class aTestingAutonomous extends LinearOpMode {

    private BNO055IMU imu;
    Orientation angles;


    public static double gyro1;
    public static double gyro2;

    double s1;
    double s2;

//    public  void main(String args[])
//    {
//        try{
//
//
//            FileWriter fw = new FileWriter("kms.txt");
//            PrintWriter pw =  new PrintWriter(fw);
//
//            pw.write("super, it works");
//            pw.write(1233212);
//
//            pw.close();
//            telemetry.addData("it works again lul","\n");
//            telemetry.update();
//        }catch (Exception e){//Catch exception if any
//            System.err.println("File not found");
//            telemetry.addData("file not found ","\n");
//            telemetry.update();
//        }
//    }


    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu3");
        imu.initialize(parameters_IMU);




        // selection();
        gyro1 = angles.firstAngle;

        waitForStart();

        gyro2 = angles.firstAngle;

        telemetry.addData("gyro1", gyro1);
        telemetry.addData("gyro2", gyro2);
        telemetry.update();


        sleep(1000000);
    }



}