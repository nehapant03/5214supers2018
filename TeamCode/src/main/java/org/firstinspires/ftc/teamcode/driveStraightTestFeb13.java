/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="driveStraightTestFeb13", group="Team5214")
@Disabled
public class driveStraightTestFeb13 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor testMotor;
    private int target;
    //declare drive motors
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
       // testMotor  = hardwareMap.get(DcMotor.class, "test_motor");
        leftBack  = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        leftFront  = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");

        //drive motor directions
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            straightWithEncoder(.25, 22);
           // runMotorWithEncoder(rightFront, .25, 40);



            telemetry.update();
            break;
        }
    }

    private void runMotorWithEncoder(DcMotor motorName, double power, int inches){ //creates function with parameters for motor name, power, and distance to move wheels


        int ticks = (int)(inches * 1120/ (4*3.14159)); //converts inches to ticks
        telemetry.addData("ticks: ", ticks); //print ticks
        telemetry.update();


        if(power < 0){
            target = motorName.getCurrentPosition() - ticks;
            while(motorName.getCurrentPosition() > target) { //while the encoder value hasn't reached the target
                motorName.setPower(power); //run motor at given power
                telemetry.addData("current tick value: ", motorName.getCurrentPosition());
                telemetry.update();

            }
        }

        else {
            target = motorName.getCurrentPosition() + ticks; //sets target to whatever the encoder reading is before anything happens plus number of ticks we want to add
            while(motorName.getCurrentPosition() < target) { //while the encoder value hasn't reached the target
                motorName.setPower(power); //run motor at given power
                telemetry.addData("current tick value: ", motorName.getCurrentPosition());
                telemetry.update();

            }
        }

        telemetry.addData("target: ", target); //print target
        telemetry.update();



        motorName.setPower(0); //after encoder value reaches target turn the motor off.
    }

    private void straightWithEncoder(double strength, int straightInches){
//        runMotorWithEncoder(leftBack, strength, straightInches);
//        runMotorWithEncoder(leftFront, strength, straightInches);
        runMotorWithEncoder(rightBack, -strength, straightInches);
        runMotorWithEncoder(rightFront, -strength, straightInches);
        telemetry.update();
    }

    private void strafeWithEncoder(double power, int strafeInches){
        runMotorWithEncoder(leftBack, power, strafeInches);
        runMotorWithEncoder(leftFront, power, strafeInches);
        runMotorWithEncoder(rightBack, power, strafeInches);
        runMotorWithEncoder(rightFront, power, strafeInches);
        telemetry.update();
    }
}
