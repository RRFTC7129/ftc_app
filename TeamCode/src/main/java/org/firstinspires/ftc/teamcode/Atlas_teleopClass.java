package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="AtlasTeleopClass", group="Opmode")
public class Atlas_teleopClass extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Inset config file
//        float lStickX1;
//        float lStickY1;
//        float rStickX1;
//        float rightTrigger2;
//        float lefttrigger2;
//        boolean dpadUp2;
//        boolean dpadDown2;
//        boolean dpadLeft2;
//        float rightStickY2;
//        float leftStickY2;
//        boolean rightBumper2;
        AtlasClass base = new AtlasClass(hardwareMap);



        waitForStart();

        while (opModeIsActive()) {
//             lStickX1 = gamepad1.left_stick_x;
//             lStickY1 = gamepad1.left_stick_y;
//             rStickX1 = gamepad1.right_stick_x;
//             rightTrigger2 = gamepad2.right_trigger;
//             lefttrigger2 = gamepad2.left_trigger;
//             dpadUp2 = gamepad2.dpad_up;
//             dpadDown2 = gamepad2.dpad_down;
//             dpadLeft2 = gamepad2.dpad_left;
//             rightStickY2 = gamepad2.right_stick_y;
//             leftStickY2 = gamepad2.left_stick_y;
//             rightBumper2 = gamepad2.right_bumper;
//            base.drive(lStickY1, lStickX1, rStickX1);
//            base.extend(leftStickY2);
//            base.dumpAndLift(rightBumper2, rightStickY2);
//            base.collectionFlip(rightTrigger2, lefttrigger2);
//            base.collectionSpin(dpadUp2, dpadDown2, dpadLeft2);
        base.drive();
        base.collectionSpin();
        base.collectionFlip();
        base.dumpAndLift();
        base.extend();

        telemetry.addData("gucci", "gang");


        }
    }
}