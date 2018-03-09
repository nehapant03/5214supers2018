package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by hima on 2/25/18.
 */

@TeleOp(name="vivisercotest", group="Relic")

public class viviServoTest extends LinearOpMode{


    private Servo ld;

    @Override
    public void runOpMode() throws InterruptedException {


        ld = hardwareMap.servo.get("COLORSERVO");

        waitForStart();

        while (opModeIsActive()){//57,11,48
            ld.setPosition(.25);
            sleep(1000);
            ld.setPosition(.5);
            sleep(1000);
            ld.setPosition(.75);
            sleep(1000);

        }
    }
}
