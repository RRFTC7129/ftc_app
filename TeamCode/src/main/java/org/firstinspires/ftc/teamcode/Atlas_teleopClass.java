package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleop", group="Opmode")

public class Atlas_teleopClass extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Inset config file
        AtlasClass base = new AtlasClass(hardwareMap);



        waitForStart();

        while (opModeIsActive()) {

            base.drive();
            base.extend();
            base.dumpAndLift();
            base.collectionFlip();
            base.collectionSpin();

        }
    }
}