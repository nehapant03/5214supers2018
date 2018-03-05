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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

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

@TeleOp(name="drivingFeb16", group="Team5214")
@Disabled public class drivingFeb16 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor testMotor;
    private int target;
    //declare drive motors
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private int ticks;
    private int position2move2;
    private double angel;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Orientation angles2;
    Acceleration gravity;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftBack = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        leftFront = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");

        imu = hardwareMap.get(BNO055IMU.class, "GYRO");
        imu.initialize(parameters);

        //drive motor directions
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        composeTelemetry();

        //testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angles2   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Left Front: ", leftFront.getCurrentPosition());
            telemetry.addData("Left Back: ", leftBack.getCurrentPosition());
            telemetry.addData("Right Front: ", rightFront.getCurrentPosition());
            telemetry.addData("Right Back: ", rightBack.getCurrentPosition());
            telemetry.update();

           // straightWithEncoder(.5,15);

            sleep(2000);

            straightWithEncoder(.5,24);

            sleep(500);

            turnLeftDegress(90, parameters);

            sleep(500);

            straightWithEncoder(0.25, -15);

            turnRightDegrees(90, parameters);

            sleep(2000);

            strafeWithEncoder(.4, 15);

            sleep(2000);


            //straightWithEncoder(.5,15);

            telemetry.update();

            sleep(1500);



            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);

            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(15000);

            break;
        }
    }

    private void motorWithEncoder(DcMotor motorName, double power, int inches) {
        ticks = (int) (inches * 1120 / (4 * 3.14159)); //converts inches to ticks
//        telemetry.addData("ticks: ", ticks);
        telemetry.update();

        //modifies moveto position based on starting ticks position, keeps running tally
        position2move2 = motorName.getCurrentPosition() + ticks;
        motorName.setTargetPosition(position2move2);
        motorName.setPower(power);

        }

//always keep strength positive, use negative inches to go backwards
    private void straightWithEncoder(double strength, int straightInches){

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorWithEncoder(leftBack, strength, straightInches);
        motorWithEncoder(leftFront, strength, straightInches);
        motorWithEncoder(rightBack, strength, straightInches);
        motorWithEncoder(rightFront, strength, straightInches);

      while(leftBack.isBusy() && leftFront.isBusy() && rightBack.isBusy() && rightFront.isBusy()) {
      }

        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

        //reset encoder values
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //put the motors back into a mode where they can run
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

//defined so that power is positive always, right is positive inches, left is negative inches
    private void strafeWithEncoder(double unlimitedpower, int strafeInches){

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorWithEncoder(leftBack, unlimitedpower, -strafeInches);
        motorWithEncoder(leftFront, unlimitedpower, strafeInches);
        motorWithEncoder(rightBack, unlimitedpower, strafeInches);
        motorWithEncoder(rightFront, unlimitedpower, -strafeInches);

        while(leftBack.isBusy() && leftFront.isBusy() && rightBack.isBusy() && rightFront.isBusy()) {
        }

        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }




    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

//    private double convertTo0to360(double angle1){
//
//        if(angle1 >= 0){
//            angel = angle1;
//        }
//        else{
//            angel = angle1 + 360;
//        }
//
//        angel = angel % 360;
//
//        return angel;
//
//    }

//    private void correctTurnFunction(String direction, double power, int targetAngle){
//        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        Orientation agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double current = Double.parseDouble(formatAngle(agl.angleUnit,agl.firstAngle));
//        current = convertTo0to360(current);
//        double start = current;
//        targetAngle += start;
//        targetAngle = targetAngle % 360;
//        telemetry.addLine(Double.toString(targetAngle));
//        telemetry.update();
//
//        if(direction == "clockwise"){
//            while (((current) < (targetAngle-2)) || (current > (targetAngle+2) )){
//                double whereiam;
//                telemetry.addLine("raw encoder values: " + Double.parseDouble(formatAngle(agl.angleUnit,agl.firstAngle)));
//
//                //telemetry.addLine("modified encoder value: " + current);
//                //telemetry.update();
//
//                leftBack.setPower(power); //sets left wheels to move backward
//                leftFront.setPower(power);
//                rightBack.setPower(-power); // makes right hand wheels to move forward
//                rightFront.setPower(-power);
//
//                agl   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//                whereiam = Double.parseDouble(formatAngle(agl.angleUnit,agl.firstAngle));
//                whereiam = whereiam * -1;
//                current = convertTo0to360(whereiam);
//            }
//        }
//
//        if(direction == "counterclockwise"){
//            while (((current) < (targetAngle-2)) || (current > (targetAngle+2) )){
//                double whereiam;
//                telemetry.addLine("raw encoder values: " + Double.parseDouble(formatAngle(agl.angleUnit,agl.firstAngle)));
//
//                //telemetry.addLine("modified encoder value: " + current);
//                //telemetry.update();
//
//                leftBack.setPower(-power); //sets left wheels to move backward
//                leftFront.setPower(-power);
//                rightBack.setPower(power); // makes right hand wheels to move forward
//                rightFront.setPower(power);
//
//                agl   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//                whereiam = Double.parseDouble(formatAngle(agl.angleUnit,agl.firstAngle));
//                current = convertTo0to360(whereiam);
//        }
//
//        leftBack.setPower(0);
//        rightBack.setPower(0);
//        leftFront.setPower(0);
//        rightFront.setPower(0);
//
//        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//    }

    private void turn(double power){
        //left turn is positive power
        leftBack.setPower(-power); //sets left wheels to move backward
        leftFront.setPower(-power);
        rightBack.setPower(power); // makes right hand wheels to move forward
        rightFront.setPower(power);

        //makes the robot go forward for an indefinite amount of time

    }

    private void turnLeftDegress(double deg, BNO055IMU.Parameters parametersMeth){



        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Orientation agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double curent = Double.parseDouble(formatAngle(agl.angleUnit,agl.firstAngle));
        double start = curent;
        double stDeg = curent+deg;

        //this loop runs until the robot has turned the correct amount
        while (((curent) < (stDeg-1.5)) || (curent > (stDeg+1.5) )){
            telemetry.update();

            //prints all the variables
            telemetry.addLine("IM IN THE WHILE");
            telemetry.addLine("start: " + Double.toString(start));
            telemetry.addLine("stDeg: " + Double.toString(stDeg));
            telemetry.addLine("deg: " + Double.toString(deg));
            telemetry.addLine("current: " + Double.toString(curent));

            turn(.25);

            agl   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            curent = Double.parseDouble(formatAngle(agl.angleUnit,agl.firstAngle));
            telemetry.update();
        }

        telemetry.addLine("I LEFT THE WHILE");
        telemetry.update();

        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu.initialize(parametersMeth);

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void turnRightDegrees(double deg, BNO055IMU.Parameters parametersMeth){
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Orientation agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double curent = Double.parseDouble(formatAngle(agl.angleUnit,agl.firstAngle));
        double start = curent;
        double stDeg = curent+deg;

        //this loop runs until the robot has turned the correct amount
        while (((-curent) < (stDeg-1.5)) || (-curent > (stDeg+1.5) )){
            telemetry.update();

            //prints all the variables
            telemetry.addLine("IM IN THE WHILE");
            telemetry.addLine("start: " + Double.toString(start));
            telemetry.addLine("stDeg: " + Double.toString(stDeg));
            telemetry.addLine("deg: " + Double.toString(deg));
            telemetry.addLine("current: " + Double.toString(curent));

            turn(-.25);

            agl   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            curent = Double.parseDouble(formatAngle(agl.angleUnit,agl.firstAngle));
            telemetry.update();
        }

        telemetry.addLine("I LEFT THE WHILE");
        telemetry.update();

        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu.initialize(parametersMeth);

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------


    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public String valueHead() {
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }
    }
