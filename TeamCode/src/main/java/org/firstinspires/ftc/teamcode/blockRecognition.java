package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;
@TeleOp(name="visiontaketwo",group="Opmode")

public class blockRecognition extends LinearOpMode {
    private VuforiaLocalizer vuforia;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private TFObjectDetector tfod;

    float conf1;
    float conf2;
    float conf3;
    String item1;
    String item2;
    String item3;
    float dir1;
    float dir2;
    float dir3;



    float Tconf1;
    float Tconf2;
    float Tconf3;
    String Titem1;
    String Titem2;
    String Titem3;


    int position;

    private ElapsedTime visionTimer = new ElapsedTime();
    protected void initVuforia() {


        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Ac8xsqH/////AAAAGcG2OeE2NECwo7mM5f9KX1RKmDT79NqkIHc/ATgW2+loN9Fr8fkfb6jE42RZmiRYeei1FvM2M3kUPdl53j" +
                "+oeuhahXi7ApkbRv9cef0kbffj+4EkWKWCgQM39sRegfX+os6PjJh1fwGdxxijW0CYXnp2Rd1vkTjIs/cW2/7TFTtuJTkc17l" +
                "+FNJAeqLEfRnwrQ0FtxvBjO8yQGcLrpeKJKX/+sN+1kJ/cvO345RYfPSoG4Pi+wo/va1wmhuZ/WCLelUeww8w8u0douStuqcuz" +
                "ufrsWmQThsHqQDfDh0oGKZGIckh3jwCV2ABkP0lT6ICBDm4wOZ8REoyiY2kjsDnnFG6cT803cfzuVuPJl+uGTEf";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1);


        CameraDevice.getInstance().setFlashTorchMode(false);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void vision() {
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (visionTimer.milliseconds() < 5000) {

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3 || updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;

                            int run = 1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {

                                    goldMineralX = (int) recognition.getLeft();
                                    telemetry.addData("equals gold", recognition.getConfidence());
                                    telemetry.addData("Run: ", run);
                                    if(run == 1){
                                        conf1 = recognition.getConfidence();
                                        item1 = "gold";
                                        telemetry.addData("confidence 1", conf1);

                                    }
                                    else if (run == 2){
                                        conf2 = recognition.getConfidence();
                                        telemetry.addData("confidence 2", conf2);
                                        item2 = "gold";

                                    }
                                    else if (run == 3) {
                                        conf3 = recognition.getConfidence();
                                        telemetry.addData("confidence 3", conf3);
                                        item3 = "gold";

                                    }



                                } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)){
                                    telemetry.addData("equals silver", recognition.getConfidence());

                                    telemetry.addData("Run: ", run);
                                    if(run == 1){
                                        conf1 = recognition.getConfidence();
                                        telemetry.addData("confidence 1", conf1);
                                        item1 = "silver";

                                    }
                                    else if (run == 2){
                                        conf2 = recognition.getConfidence();
                                        telemetry.addData("confidence 2", conf2);
                                        item2 = "silver";

                                    }
                                    else if (run == 3) {
                                        conf3 = recognition.getConfidence();
                                        telemetry.addData("confidence 3", conf3);
                                        item3 = "silver";

                                    }

                                }
                                else {
                                    telemetry.addData("notRecognized", recognition.getConfidence());

                                    if(run == 1){
                                        conf1 = recognition.getConfidence();
                                        telemetry.addData("confidence 1", conf1);
                                        item1 = "unk";

                                    }
                                    else if (run == 2){
                                        conf2 = recognition.getConfidence();
                                        telemetry.addData("confidence 2", conf2);
                                        item2 = "unk";

                                    }
                                    else if (run == 3) {
                                        conf3 = recognition.getConfidence();
                                        telemetry.addData("confidence 2", conf3);
                                        item3 = "unk";

                                    }

                                }
                                //telemetry.update();
                                run = run + 1;
                                if (run == 4){
                                    run = 1;
                                }
//                                else if (silverMineral1X == -1) {
//                                    silverMineral1X = (int) recognition.getLeft();
//                                    telemetry.addData("left silver", recognition.getLeft());
//                                } else {
//                                    silverMineral2X = (int) recognition.getLeft();
//                                    telemetry.addData(" not left silver", recognition.getLeft());
//
//                                }
                            }
//                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                                    telemetry.addData("Gold Mineral Position", "Left");
//                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                                    telemetry.addData("Gold Mineral Position", "Right");
//                                } else {
//                                    telemetry.addData("Gold Mineral Position", "Center");
//                                }
//                            }
                        }
                        telemetry.update();
                    }
                }
            }

        }


    }

    public void logic(){
        if(item1 == "gold" && conf1 >0.99){
            position = 1;
        }
        else if(item2 == "gold" && conf2 > 0.99){
            position = 2;
        }
        else if(item3 == "gold" && conf3 > 0.99){
            position = 3;
        }
        if(conf3 == 0){
            telemetry.addData("only saw two (conf3)", conf3);
        }
        else if(conf1 > conf2 && conf2 > conf3) {
            telemetry.addData("Third item thrown out.", conf3);
            telemetry.addData("First item is :", conf1);
            telemetry.addData("Second item", conf2);
            if (item1 == "silver" && item2 == "gold") {
                position = 2;
            } else if (item1 == "gold" && item2 == "silver") {
                position = 1;
            } else if (item1 == "silver" && item2 == "silver") {
                position = 3;
            } else {
                telemetry.addData("pos setting messed up", position);
            }
        }
        else if (conf1 > conf3 && conf3 > conf2){
            telemetry.addData("Second item thrown out.", conf2);
            telemetry.addData("First item is :", conf1);
            telemetry.addData("Third item", conf3);
            if(item1 == "silver" && item3 == "gold"){
                position = 3;
            }
            else if(item1 == "gold" && item3 == "silver"){
                position = 1;
            }
            else if (item1 == "silver" && item3 == "silver"){
                position = 2;
            }
            else{
                telemetry.addData("pos setting messed up", position);
            }
        }

        else if (conf2 > conf1 && conf1 > conf3){
            telemetry.addData("Third item thrown out.", conf3);
            telemetry.addData("First item is :", conf1);
            telemetry.addData("Second item", conf2);

            if(item1 == "silver" && item2 == "gold"){
                position = 2;
            }
            else if(item1 == "gold" && item2 == "silver"){
                position = 1;
            }
            else if (item1 == "silver" && item2 == "silver"){
                position = 3;
            }
            else{
                telemetry.addData("pos setting messed up", position);
            }
        }
        else if (conf2 > conf3 && conf3 > conf1){
            telemetry.addData("First item thrown out.", conf1);
            telemetry.addData("Second item is :", conf2);
            telemetry.addData("Third item", conf3);
            if(item2 == "silver" && item3 == "gold"){
                position = 3;
            }
            else if(item2 == "gold" && item3 == "silver"){
                position = 2;
            }
            else if (item2 == "silver" && item3 == "silver"){
                position = 1;
            }
            else{
                telemetry.addData("pos setting messed up", position);
            }
        }
        else if (conf3 > conf1 && conf1 > conf2){
            telemetry.addData("Second item thrown out.", conf2);
            telemetry.addData("First item is :", conf1);
            telemetry.addData("Third item", conf3);
            if(item1 == "silver" && item3 == "gold"){
                position = 3;
            }
            else if(item1 == "gold" && item3 == "silver"){
                position = 1;
            }
            else if (item1 == "silver" && item3 == "silver"){
                position = 2;
            }
            else{
                telemetry.addData("pos setting messed up", position);
            }
        }
        else if(conf3 > conf2 && conf2 > conf1){
            telemetry.addData("First item thrown out.", conf1);
            telemetry.addData("Second item is :", conf2);
            telemetry.addData("Third item", conf3);
            if(item2 == "silver" && item3 == "gold"){
                position = 3;
            }
            else if(item2 == "gold" && item3 == "silver"){
                position = 2;
            }
            else if (item2 == "silver" && item3 == "silver"){
                position = 1;
            }
            else{
                telemetry.addData("pos setting messed up", position);
            }
        }


        else{
            telemetry.addData("something messed up1",conf1);
            telemetry.addData("something messed up2",conf2);
            telemetry.addData("something messed up3",conf3);


        }
        telemetry.addData("position is:", position);
        telemetry.addData("item1", item1);
        telemetry.addData("item2", item2);
        telemetry.addData("item3", item3);



        telemetry.update();
    }

    public void try2(){
        if (opModeIsActive()) {
            tfod.deactivate();
            sleep(50);
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
visionTimer.reset();
            while (visionTimer.milliseconds() < 5000) {

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3 || updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;

                            int run = 1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {

                                    goldMineralX = (int) recognition.getLeft();
                                    telemetry.addData("equals gold", recognition.getConfidence());
                                    telemetry.addData("Run: ", run);
                                    if(run == 1){
                                        conf1 = recognition.getConfidence();
                                        item1 = "gold";
                                        telemetry.addData("confidence 1", conf1);

                                    }
                                    else if (run == 2){
                                        conf2 = recognition.getConfidence();
                                        telemetry.addData("confidence 2", conf2);
                                        item2 = "gold";

                                    }
                                    else if (run == 3) {
                                        conf3 = recognition.getConfidence();
                                        telemetry.addData("confidence 3", conf3);
                                        item3 = "gold";

                                    }



                                } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)){
                                    telemetry.addData("equals silver", recognition.getConfidence());

                                    telemetry.addData("Run: ", run);
                                    if(run == 1){
                                        conf1 = recognition.getConfidence();
                                        telemetry.addData("confidence 1", conf1);
                                        item1 = "silver";

                                    }
                                    else if (run == 2){
                                        conf2 = recognition.getConfidence();
                                        telemetry.addData("confidence 2", conf2);
                                        item2 = "silver";

                                    }
                                    else if (run == 3) {
                                        conf3 = recognition.getConfidence();
                                        telemetry.addData("confidence 3", conf3);
                                        item3 = "silver";

                                    }

                                }
                                else {
                                    telemetry.addData("notRecognized", recognition.getConfidence());

                                    if(run == 1){
                                        conf1 = recognition.getConfidence();
                                        telemetry.addData("confidence 1", conf1);
                                        item1 = "unk";

                                    }
                                    else if (run == 2){
                                        conf2 = recognition.getConfidence();
                                        telemetry.addData("confidence 2", conf2);
                                        item2 = "unk";

                                    }
                                    else if (run == 3) {
                                        conf3 = recognition.getConfidence();
                                        telemetry.addData("confidence 2", conf3);
                                        item3 = "unk";

                                    }

                                }
                                //telemetry.update();
                                run = run + 1;
                                if (run == 4){
                                    run = 1;
                                }
//                                else if (silverMineral1X == -1) {
//                                    silverMineral1X = (int) recognition.getLeft();
//                                    telemetry.addData("left silver", recognition.getLeft());
//                                } else {
//                                    silverMineral2X = (int) recognition.getLeft();
//                                    telemetry.addData(" not left silver", recognition.getLeft());
//
//                                }
                            }
//                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                                    telemetry.addData("Gold Mineral Position", "Left");
//                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                                    telemetry.addData("Gold Mineral Position", "Right");
//                                } else {
//                                    telemetry.addData("Gold Mineral Position", "Center");
//                                }
//                            }
                        }
                        telemetry.update();
                    }
                }
            }

        }


    }

    public void visionOrdering() {
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (visionTimer.milliseconds() < 5000) {

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3 || updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;

                            int run = 1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {

                                    dir1 = recognition.getLeft();
                                    telemetry.addData("equals gold", recognition.getConfidence());
                                    telemetry.addData("Run: ", run);
                                    if(run == 1){
                                        conf1 = recognition.getConfidence();
                                        dir1 = recognition.getLeft();
                                        item1 = "gold";
                                        telemetry.addData("confidence 1", conf1);

                                    }
                                    else if (run == 2){
                                        conf2 = recognition.getConfidence();
                                        dir2 = recognition.getLeft();
                                        telemetry.addData("confidence 2", conf2);
                                        item2 = "gold";

                                    }
                                    else if (run == 3) {
                                        conf3 = recognition.getConfidence();
                                        dir3 = recognition.getLeft();
                                        telemetry.addData("confidence 3", conf3);
                                        item3 = "gold";

                                    }



                                } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)){
                                    telemetry.addData("equals silver", recognition.getConfidence());

                                    telemetry.addData("Run: ", run);
                                    if(run == 1){
                                        conf1 = recognition.getConfidence();
                                        dir1 = recognition.getLeft();
                                        telemetry.addData("confidence 1", conf1);
                                        item1 = "silver";

                                    }
                                    else if (run == 2){
                                        conf2 = recognition.getConfidence();
                                        dir2 = recognition.getLeft();
                                        telemetry.addData("confidence 2", conf2);
                                        item2 = "silver";

                                    }
                                    else if (run == 3) {
                                        conf3 = recognition.getConfidence();
                                        dir3 = recognition.getLeft();
                                        telemetry.addData("confidence 3", conf3);
                                        item3 = "silver";

                                    }

                                }
                                else {
                                    telemetry.addData("notRecognized", recognition.getConfidence());

                                    if(run == 1){
                                        conf1 = recognition.getConfidence();
                                        telemetry.addData("confidence 1", conf1);
                                        item1 = "unk";

                                    }
                                    else if (run == 2){
                                        conf2 = recognition.getConfidence();
                                        telemetry.addData("confidence 2", conf2);
                                        item2 = "unk";

                                    }
                                    else if (run == 3) {
                                        conf3 = recognition.getConfidence();
                                        telemetry.addData("confidence 2", conf3);
                                        item3 = "unk";

                                    }

                                }
                                //telemetry.update();
                                run = run + 1;
                                if (run == 4){
                                    run = 1;
                                }
//                                else if (silverMineral1X == -1) {
//                                    silverMineral1X = (int) recognition.getLeft();
//                                    telemetry.addData("left silver", recognition.getLeft());
//                                } else {
//                                    silverMineral2X = (int) recognition.getLeft();
//                                    telemetry.addData(" not left silver", recognition.getLeft());
//
//                                }
                            }
//                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                                    telemetry.addData("Gold Mineral Position", "Left");
//                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                                    telemetry.addData("Gold Mineral Position", "Right");
//                                } else {
//                                    telemetry.addData("Gold Mineral Position", "Center");
//                                }
//                            }
                        }
                        telemetry.update();
                    }
                    if(dir1 > dir2){
                        if(dir1 > dir3 && dir3 > dir2){
                            Tconf1 = conf1;
                            Titem1 = item1;
                            Tconf2 = conf3;
                            Titem2 = item3;
                            Tconf3 = conf2;
                            Titem3 = item2;

                        }
                        else if (dir1 > dir3 && dir2 > dir3){
                            Tconf1 = conf1;
                            Titem1 = item1;
                            Tconf3 = conf3;
                            Titem3 = item3;
                            Tconf2 = conf2;
                            Titem2 = item2;
                        }
                        else if (dir3 > dir1){
                            Tconf3 = conf1;
                            Titem3 = item1;
                            Tconf2 = conf3;
                            Titem2 = item3;
                            Tconf1 = conf2;
                            Titem1 = item2;

                        }
                    }
                    else if(dir2 > dir1){
                        if(dir2 > dir3 && dir3 > dir1){
                            Tconf2 = conf1;
                            Titem2 = item1;
                            Tconf1 = conf3;
                            Titem1 = item3;
                            Tconf3 = conf2;
                            Titem3 = item2;
                        }
                        else if(dir2 > dir3 && dir1 > dir3){
                            Tconf2 = conf1;
                            Titem2 = item1;
                            Tconf3 = conf3;
                            Titem3 = item3;
                            Tconf1 = conf2;
                            Titem1 = item2;
                        }
                        else if(dir3 > dir2){
                            Tconf3 = conf1;
                            Titem3 = item1;
                            Tconf1 = conf3;
                            Titem1 = item3;
                            Tconf2 = conf2;
                            Titem2 = item2;

                        }
                    }
                }
            }

        }


    }

    public void logicOrdering(){
        if(Titem1 == "gold" && Tconf1 >0.99){
            position = 1;
        }
        else if(Titem2 == "gold" && Tconf2 > 0.99){
            position = 2;
        }
        else if(Titem3 == "gold" && Tconf3 > 0.99){
            position = 3;
        }
        if(Tconf3 == 0){
            telemetry.addData("only saw two (Tconf3)", Tconf3);
        }
        else if(Tconf1 > Tconf2 && Tconf2 > Tconf3) {
            telemetry.addData("Third item thrown out.", Tconf3);
            telemetry.addData("First item is :", Tconf1);
            telemetry.addData("Second item", Tconf2);
            if (Titem1 == "silver" && Titem2 == "gold") {
                position = 2;
            } else if (Titem1 == "gold" && Titem2 == "silver") {
                position = 1;
            } else if (Titem1 == "silver" && Titem2 == "silver") {
                position = 3;
            } else {
                telemetry.addData("pos setting messed up", position);
            }
        }
        else if (Tconf1 > Tconf3 && Tconf3 > Tconf2){
            telemetry.addData("Second item thrown out.", Tconf2);
            telemetry.addData("First item is :", Tconf1);
            telemetry.addData("Third item", Tconf3);
            if(Titem1 == "silver" && Titem3 == "gold"){
                position = 3;
            }
            else if(Titem1 == "gold" && Titem3 == "silver"){
                position = 1;
            }
            else if (Titem1 == "silver" && Titem3 == "silver"){
                position = 2;
            }
            else{
                telemetry.addData("pos setting messed up", position);
            }
        }

        else if (Tconf2 > Tconf1 && Tconf1 > Tconf3){
            telemetry.addData("Third item thrown out.", Tconf3);
            telemetry.addData("First item is :", Tconf1);
            telemetry.addData("Second item", Tconf2);

            if(Titem1 == "silver" && Titem2 == "gold"){
                position = 2;
            }
            else if(Titem1 == "gold" && Titem2 == "silver"){
                position = 1;
            }
            else if (Titem1 == "silver" && Titem2 == "silver"){
                position = 3;
            }
            else{
                telemetry.addData("pos setting messed up", position);
            }
        }
        else if (Tconf2 > Tconf3 && Tconf3 > Tconf1){
            telemetry.addData("First item thrown out.", Tconf1);
            telemetry.addData("Second item is :", Tconf2);
            telemetry.addData("Third item", Tconf3);
            if(Titem2 == "silver" && Titem3 == "gold"){
                position = 3;
            }
            else if(Titem2 == "gold" && Titem3 == "silver"){
                position = 2;
            }
            else if (Titem2 == "silver" && Titem3 == "silver"){
                position = 1;
            }
            else{
                telemetry.addData("pos setting messed up", position);
            }
        }
        else if (Tconf3 > Tconf1 && Tconf1 > Tconf2){
            telemetry.addData("Second item thrown out.", Tconf2);
            telemetry.addData("First item is :", Tconf1);
            telemetry.addData("Third item", Tconf3);
            if(Titem1 == "silver" && Titem3 == "gold"){
                position = 3;
            }
            else if(Titem1 == "gold" && Titem3 == "silver"){
                position = 1;
            }
            else if (Titem1 == "silver" && Titem3 == "silver"){
                position = 2;
            }
            else{
                telemetry.addData("pos setting messed up", position);
            }
        }
        else if(Tconf3 > Tconf2 && Tconf2 > Tconf1){
            telemetry.addData("First item thrown out.", Tconf1);
            telemetry.addData("Second item is :", Tconf2);
            telemetry.addData("Third item", Tconf3);
            if(Titem2 == "silver" && Titem3 == "gold"){
                position = 3;
            }
            else if(Titem2 == "gold" && Titem3 == "silver"){
                position = 2;
            }
            else if (Titem2 == "silver" && Titem3 == "silver"){
                position = 1;
            }
            else{
                telemetry.addData("pos setting messed up", position);
            }
        }


        else{
            telemetry.addData("something messed up1",Tconf1);
            telemetry.addData("something messed up2",Tconf2);
            telemetry.addData("something messed up3",Tconf3);


        }

//        if(Titem1 == "gold"){
//            position = 1;
//        }
//        else if (Titem2 == "gold"){
//            position = 2;
//        }
//        else if (Titem3 == "gold"){
//            position = 3;
//        }
        telemetry.addData("position is:", position);
        telemetry.addData("item1", Titem1);
        telemetry.addData("item2", Titem2);
        telemetry.addData("item3", Titem3);



        telemetry.update();
    }

    public void logicTesting() {
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (visionTimer.milliseconds() < 5000) {

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3 || updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;

                            int run = 1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {

                                    //dir1 = recognition.getLeft();
                                    telemetry.addData("equals gold", recognition.getConfidence());
                                    telemetry.addData("Run: ", run);
                                    if (run == 1) {
                                        conf1 = recognition.getConfidence();
                                        dir1 = recognition.getLeft();
                                        item1 = "gold";
                                        telemetry.addData("confidence 1", conf1);

                                    } else if (run == 2) {
                                        conf2 = recognition.getConfidence();
                                        dir2 = recognition.getLeft();
                                        telemetry.addData("confidence 2", conf2);
                                        item2 = "gold";

                                    } else if (run == 3) {
                                        conf3 = recognition.getConfidence();
                                        dir3 = recognition.getLeft();
                                        telemetry.addData("confidence 3", conf3);
                                        item3 = "gold";

                                    }


                                } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                    telemetry.addData("equals silver", recognition.getConfidence());

                                    telemetry.addData("Run: ", run);
                                    if (run == 1) {
                                        conf1 = recognition.getConfidence();
                                        dir1 = recognition.getLeft();
                                        telemetry.addData("confidence 1", conf1);
                                        item1 = "silver";

                                    } else if (run == 2) {
                                        conf2 = recognition.getConfidence();
                                        dir2 = recognition.getLeft();
                                        telemetry.addData("confidence 2", conf2);
                                        item2 = "silver";

                                    } else if (run == 3) {
                                        conf3 = recognition.getConfidence();
                                        dir3 = recognition.getLeft();
                                        telemetry.addData("confidence 3", conf3);
                                        item3 = "silver";

                                    }

                                } else {
                                    telemetry.addData("notRecognized", recognition.getConfidence());

                                    if (run == 1) {
                                        conf1 = recognition.getConfidence();
                                        telemetry.addData("confidence 1", conf1);
                                        item1 = "unk";

                                    } else if (run == 2) {
                                        conf2 = recognition.getConfidence();
                                        telemetry.addData("confidence 2", conf2);
                                        item2 = "unk";

                                    } else if (run == 3) {
                                        conf3 = recognition.getConfidence();
                                        telemetry.addData("confidence 2", conf3);
                                        item3 = "unk";

                                    }

                                }
                                //telemetry.update();
                                run = run + 1;
                                if (run == 4) {
                                    run = 1;
                                }
//                                else if (silverMineral1X == -1) {
//                                    silverMineral1X = (int) recognition.getLeft();
//                                    telemetry.addData("left silver", recognition.getLeft());
//                                } else {
//                                    silverMineral2X = (int) recognition.getLeft();
//                                    telemetry.addData(" not left silver", recognition.getLeft());
//
//                                }
                            }
//                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                                    telemetry.addData("Gold Mineral Position", "Left");
//                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                                    telemetry.addData("Gold Mineral Position", "Right");
//                                } else {
//                                    telemetry.addData("Gold Mineral Position", "Center");
//                                }
//                            }
                        }
                        telemetry.update();
                    }
                    if (dir1 > dir2) {
                        if (dir1 > dir3 && dir3 > dir2) {
                            Tconf1 = conf1;
                            Titem1 = item1;
                            Tconf2 = conf3;
                            Titem2 = item3;
                            Tconf3 = conf2;
                            Titem3 = item2;

                        } else if (dir1 > dir3 && dir2 > dir3) {
                            Tconf1 = conf1;
                            Titem1 = item1;
                            Tconf3 = conf3;
                            Titem3 = item3;
                            Tconf2 = conf2;
                            Titem2 = item2;
                        } else if (dir3 > dir1) {
                            Tconf3 = conf1;
                            Titem3 = item1;
                            Tconf2 = conf3;
                            Titem2 = item3;
                            Tconf1 = conf2;
                            Titem1 = item2;

                        }
                    } else if (dir2 > dir1) {
                        if (dir2 > dir3 && dir3 > dir1) {
                            Tconf2 = conf1;
                            Titem2 = item1;
                            Tconf1 = conf3;
                            Titem1 = item3;
                            Tconf3 = conf2;
                            Titem3 = item2;
                        } else if (dir2 > dir3 && dir1 > dir3) {
                            Tconf2 = conf1;
                            Titem2 = item1;
                            Tconf3 = conf3;
                            Titem3 = item3;
                            Tconf1 = conf2;
                            Titem1 = item2;
                        } else if (dir3 > dir2) {
                            Tconf3 = conf1;
                            Titem3 = item1;
                            Tconf1 = conf3;
                            Titem1 = item3;
                            Tconf2 = conf2;
                            Titem2 = item2;

                        }
                    }
                }
            }


        }
    }
    @Override
    public void runOpMode() {

        initVuforia();

        initTfod();

        waitForStart();
        visionTimer.reset();
        visionOrdering();
        telemetry.addData("item1", Titem1);
        telemetry.addData("item2", Titem2);
        telemetry.addData("item3", Titem3);
        telemetry.update();
        sleep(10000);
//        if(conf3 == 0) {
//            telemetry.addData("activating try2", visionTimer);
//            try2();
//        }

        logicOrdering();
        sleep(20000);


    }
}

