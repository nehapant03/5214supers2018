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
    private Servo colorServo;


    @Override
    public void runOpMode() throws InterruptedException {

        colorServo = hardwareMap.get(Servo.class, "COLORSERVO");

        waitForStart();

        while (opModeIsActive()){
            colorServo.setPosition(0);
            sleep(1500);
            colorServo.setPosition(.75);
            sleep(15000);

        }
    }
}
