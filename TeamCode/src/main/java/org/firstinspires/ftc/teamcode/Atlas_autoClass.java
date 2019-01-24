package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Autonomous(name="AutonomousREEE", group="OpmodesdaLUL")

public class Atlas_autoClass extends LinearOpMode {


    public void runOpMode() throws InterruptedException {

        AtlasClass base = new AtlasClass(hardwareMap);

        base.initAuto();
//
//        base.selection();
//
//        sleep(base.wait);
//
//        base.vision();
//
//        if(base.side) {
//            if(base.two) {
//                base.toBlockTwo();
//            }
//            else if (!base.two){
//                base.toBlockPark();
//            }
//        }
//
//        else if(!base.side) {
//            if(base.marker) {
//                base.toBlockCraterMarker();
//            }
//            else if(!base.marker){
//                base.toBlockCraterPark();
//
//            }
//        }





    }
}