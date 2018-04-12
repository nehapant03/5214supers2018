package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

@Autonomous(name="april9th_Blue_Far_v1", group="Team5214")
//@Disabled
public class april9th_Blue_Far_v1 extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;

    private DcMotor liftMotor;
    private DcMotor relicMotor;

    private DcMotor lBelt;
    private DcMotor rBelt;

    private Servo colorServo;
    private Servo FLICKSERVO;


    private String colorid;
    // declare color sensor
    private ColorSensor colorFront;

    private Servo rightDump;
    private Servo leftDump;
    private Servo centerDump;
    private Servo wrist;
    private Servo finger;

    private Servo leftPush;
    private Servo rightPush;

    private int ticks;
    private int position2move2;
    // The IMU sensor object
    BNO055IMU imu;
    VuforiaLocalizer vuforia;


    //use the two variables in two color sensors situation
//    ColorSensor colorFront;
//    ColorSensor colorBack;

    final double currentRatio = 1.3; //ratio set for red/blue, for color id function

    // State used for updating telemetry
    Orientation angles;
    Orientation angles2;
    Acceleration gravity;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //using phone camera for image recognition
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersVu = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //License key, do not change
        parametersVu.vuforiaLicenseKey = "AVOFXuz/////AAAAGf45mZPfQEQBt1NyBSqlPuYQkVhLXgkSQpOqQqWb/FoWqJ" +
                "WqG7KKeaIVeJzCSsLJ58FGWwE0Z/vvzSHrZBeZN9jN7c+gru1h0T3k0wLaoN1b6bFIHn93evRQ0DcFcgy4uMHZ1" +
                "T87fT4WrKldfG6XT7PyThP2Fk5C8SbASqna7IKl26eb+zdOFXRKG+U1pZyV9yGgMsmBVZCxDZiT/G6JUpg4DMGrZVT" +
                "8BXdVvs+mpDLQ4tH/XL5ikYp1++1fbYhJtA3naS5/laHiPiHONGAdLbHkE4s8EOxpB8+lqpJN6hlcqtMegarTOuwWYXXP" +
                "jSnNnkUWBKuW6nWqtF3k1CIUoSTBuFpbwAvf+T6i1CkL6IoB";
        //using back camera for recognizing
        parametersVu.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //importing the three image asset and hook up to vuforia engine
        this.vuforia = ClassFactory.createVuforiaLocalizer(parametersVu);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftBack = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        leftFront = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");

        lBelt = hardwareMap.dcMotor.get("LBELT");
        rBelt = hardwareMap.dcMotor.get("RBELT");

        liftMotor = hardwareMap.dcMotor.get("LIFT");
        relicMotor = hardwareMap.dcMotor.get("RELICMOTOR");

        centerDump = hardwareMap.servo.get("CD");
        rightDump = hardwareMap.servo.get("RD");
        leftDump = hardwareMap.servo.get("LD");
        colorServo = hardwareMap.servo.get("COLORSERVO");
        FLICKSERVO = hardwareMap.servo.get("FLICKSERVO");
        wrist = hardwareMap.servo.get("WRIST");
        finger = hardwareMap.servo.get("FINGER");

        leftPush = hardwareMap.servo.get("LPUSH");
        rightPush = hardwareMap.servo.get("RPUSH");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();



        imu = hardwareMap.get(BNO055IMU.class, "GYRO");
        colorFront = hardwareMap.get(ColorSensor.class,"CSF");
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

        FLICKSERVO.setPosition(.5);
        colorServo.setPosition(.67);
        centerDump.setPosition(.33);

        // leftDump.setPosition(.61);

        leftPush.setPosition(.5);
        rightPush.setPosition(.5);

        composeTelemetry();

        //testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        double start = System.currentTimeMillis();
        relicTrackables.activate();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angles2   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double end = System.currentTimeMillis();
        Double result = (end - start)/1000;
        telemetry.addData("here is the time guysssssss: ", result);

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

//            arm(.13); // put arm down
//
//            sleep(700);
//            colorid = checkColor(colorFront, currentRatio);
//
//            telemetry.addLine(colorid);
//            telemetry.update();
//
//            if (colorid.equals("RED")){
//                FLICKSERVO(0.2);
//            }
//            else if(checkColor(colorFront,.4).equals("BLUE")){
//                FLICKSERVO(.8);
//            }

            sleep(300);
            FLICKSERVO.setPosition(.5);
            arm(.67); // put arm up
            wrist.setPosition(1);


            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            sleep(200);
            telemetry.addLine(vuMark.toString());
            telemetry.update();

            String keyResult = vuMark.toString();
            //hard coded to left
            keyResult = "LEFT";

            switch (keyResult) {
                case "LEFT":
                    telemetry.addLine("robot headed to left position");
                    telemetry.update();

                    strafeWithEncoder(0.6, 34);
                    turnRightDegrees(179.5, parameters);
                    //put down intake here
                    leftPush.setPosition(.55);
                    rightPush.setPosition(.55);

                    straightWithEncoder(0.5, -15);
                    turnLeftDegress(34, parameters);
                    leftDump.setPosition(.61);
                    straightWithEncoder(0.6, -19);
                    sleep(300);

                    //dump
                    centerDump.setPosition(.8);
                    leftDump.setPosition(.18);


                    sleep(700);

                    leftDump.setPosition(0.71);

                    //push cube
                    straightWithEncoder(0.65, -7);
                    straightWithEncoder(0.65, 4);
                    straightWithEncoder(0.65, -5);
                    straightWithEncoder(0.65, 5);
                    break;
                case "CENTER":
                    telemetry.addLine("robot headed to centre position");
                    telemetry.update();

                    strafeWithEncoder(0.6, 34);
                    turnRightDegrees(179.5, parameters);
                    //put down intake here
                    leftPush.setPosition(.55);
                    rightPush.setPosition(.55);

                    straightWithEncoder(0.5, -15);
                    turnLeftDegress(21, parameters);
                    leftDump.setPosition(.61);
                    straightWithEncoder(0.6, -17);
                    sleep(300);


                    centerDump.setPosition(.8);
                    leftDump.setPosition(.18);


                    sleep(700);

                    leftDump.setPosition(0.71);

                    //push cube
                    straightWithEncoder(0.6, -7);
                    straightWithEncoder(0.6, 4);
                    straightWithEncoder(0.6, -5);
                    straightWithEncoder(0.6, 5);
                    break;
                case "RIGHT":
                    telemetry.addLine("robot headed to right position");
                    telemetry.update();
                    break;
                default:
                    telemetry.addLine("boi i dont get a reading so i guess i am going to the left position");
                    telemetry.update();
            }

            telemetry.update();

            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);

            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            idle();
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


    private void turn(double power){
        //left turn is positive power
        leftBack.setPower(-power); //sets left wheels to move backward
        leftFront.setPower(-power);
        rightBack.setPower(power); // makes right hand wheels to move forward
        rightFront.setPower(power);

        //makes the robot go forward for an indefinite amount of time

    }
    private void turnWithGyro(String direction, double power, double deg, BNO055IMU.Parameters parametersMeth) {
        //so that we can control the motors normally
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Orientation agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double current = Double.parseDouble(formatAngle(agl.angleUnit, agl.firstAngle));
        double start = current;
        double target = current + deg - 2;
        double n = 0;
        if(target>=50){
            n = 25;
        }else{
            n = 20;
        }
        double y = 0;

        telemetry.addLine("start: " + Double.toString(start));
        telemetry.addLine("target: " + Double.toString(target));
        telemetry.addLine("deg: " + Double.toString(deg));
        telemetry.update();

        if(direction == "left") {
            //keep the power constant for a certain amount of time (target - n degrees) before decreasing
            while (current < target - n) {
                telemetry.update();
                telemetry.addLine("IM IN THE 1ST WHILE");
                turn(power);
                agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                current = Double.parseDouble(formatAngle(agl.angleUnit, agl.firstAngle)); //update current position
                telemetry.addLine("current = " + Double.toString(current)); //print current
                telemetry.update();
            }
            telemetry.addLine("I left the target - n loop");
            telemetry.update();

            //have the power decrease until we reach target
            while (current < target) {
                telemetry.update();
                telemetry.addLine("IM IN THE 2ND WHILE NOW");
                y = (-(power - .2)/n)*(current - target) + .22;
                turn(y);
                agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                current = Double.parseDouble(formatAngle(agl.angleUnit, agl.firstAngle));
                telemetry.addLine("current = " + Double.toString(current));
                telemetry.update();
            }


        }
        else{
            //keep the power constant for a certain amount of time (target - n degrees) before decreasing
            while (Math.abs(current) < target - n) {
                telemetry.update();
                telemetry.addLine("IM IN THE 1ST WHILE");
                y = (-(power - .2)/n)*(current - target) + .22;
                turn(-power);
                agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                current = Math.abs(Double.parseDouble(formatAngle(agl.angleUnit, agl.firstAngle))); //update current position
                telemetry.addLine("current = " + Double.toString(current)); //print current
                telemetry.update();
            }
            telemetry.addLine("I left the target - n loop");
            telemetry.update();

            //have the power decrease until we reach target
            while (Math.abs(current) < target) {
                telemetry.update();
                telemetry.addLine("IM IN THE 2ND WHILE NOW");
                turn(-y);
                agl = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                current = Math.abs(Double.parseDouble(formatAngle(agl.angleUnit, agl.firstAngle)));
                telemetry.addLine("current = " + Double.toString(current));
                telemetry.update();
            }
        }
        telemetry.addLine(Double.toString(Double.parseDouble(formatAngle(agl.angleUnit, agl.firstAngle))));
        telemetry.addLine("I LEFT THE WHILE");
        telemetry.update();

        //kill the power
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

        sleep(1000);
        telemetry.addLine("final reading = " + formatAngle(agl.angleUnit, agl.firstAngle));
        telemetry.update();

        //reset encoders and reset the mode and other stuff that just needs to be there
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
        while (((curent) < (stDeg-1.5)) ){
            telemetry.update();

            //prints all the variables
            telemetry.addLine("IM IN THE WHILE");
            telemetry.addLine("start: " + Double.toString(start));
            telemetry.addLine("stDeg: " + Double.toString(stDeg));
            telemetry.addLine("deg: " + Double.toString(deg));
            telemetry.addLine("current: " + Double.toString(curent));

            turn(.3);

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
        while (((-curent) < (stDeg-1.5)) ){
            telemetry.update();

            //prints all the variables
            telemetry.addLine("IM IN THE WHILE");
            telemetry.addLine("start: " + Double.toString(start));
            telemetry.addLine("stDeg: " + Double.toString(stDeg));
            telemetry.addLine("deg: " + Double.toString(deg));
            telemetry.addLine("current: " + Double.toString(curent));

            turn(-.3);

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

    private void dump(double left, double right) {
        //setting the two dump servo to an input value
        leftDump.setPosition(left);
        // rightDump.setPosition(right);
    }
    private void FLICKSERVO(double position) {
        //setting the FLICKSERVO servo to an input value
        FLICKSERVO.setPosition(position);
        sleep(2000);
        FLICKSERVO.setPosition(0.5);

    }

    private void arm(double position) {
        //setting the color servo to an input value
        colorServo.setPosition(position);
    }
    private void rampDown(){
        leftDump.setPosition(.5);
        rightDump.setPosition(.5);

        leftDump.setPosition(.8);
        rightDump.setPosition(.4);

        sleep(3000);

        leftDump.setPosition(.5);
        rightDump.setPosition(.5);
    }
    private void sleep(int i) {
        //initial time takes the current hardware time in milliseconds
        long initial_time = System.currentTimeMillis();
        //inside the while loop cpu will stop working when the input time is more than the time passed in this loop
        //cpu will be back working when the loop reaches the target time
        while (System.currentTimeMillis() - initial_time < i) {

        }
    }
    private void intake(DcMotor leftIntake, DcMotor rightIntake, String status) {
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        switch (status) {
            case "IN":
                leftIntake.setPower(1);
                rightIntake.setPower(-1);
                break;
            case "OUT":
                leftIntake.setPower(-1);
                rightIntake.setPower(1);
                break;
            case "OFF":
                leftIntake.setPower(0);
                rightIntake.setPower(0);
                break;
            default:
                telemetry.addLine("ehhhhh i think you didnt write the correct status");
                break;
        }
    }


    private String checkColor(ColorSensor sensor, double ratio) {
        double redOverBlue = (sensor.red()+1) / (sensor.blue() + 1);
        if (redOverBlue >= ratio) {
            //if it is greater than ratio, it is red
            return "RED";
        }
        else if (redOverBlue <= ratio) {
            //if it is less than ratio, it is blue
            return "BLUE";
        }
        else {
            //if nothing is detected, return not defined
            return "UNDEF";
        }
    }

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