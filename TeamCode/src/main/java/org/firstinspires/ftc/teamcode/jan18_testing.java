package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="jan18_testing", group="Team5214")
@Disabled
public class jan18_testing extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //declare drive motors
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    // declare dump servo
    private Servo leftDump;
    private Servo rightDump;
    // declare color servo
    private Servo colorServo;
    private Servo flickServo;
    private String colorid;
    // declare color sensor
    ColorSensor colorFront;
 //   ColorSensor colorBack;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //mapping drive motors to configuration
        leftBack  = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        leftFront  = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");

        //mapping dump servos to configuration
        leftDump  = hardwareMap.get(Servo.class, "LD");
        rightDump = hardwareMap.get(Servo.class, "RD");

        //mapping color servo to configuration
        colorServo = hardwareMap.get(Servo.class, "COLORSERVO");
        flickServo = hardwareMap.get(Servo.class, "FLICKSERVO");

        //mapping color sensor to configuration
        colorFront  = hardwareMap.get(ColorSensor.class, "CSF");
 //       colorBack = hardwareMap.get(ColorSensor.class, "CSB");

        //drive motor directions
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //read key

            //check color
            arm(0.1);
            sleep(1000);
            //color gives the output of the front ball (the one which is closer to colorFront sensor)
 //           colorid = checkColor(colorFront, colorBack, 1.25);
            //print color state
            telemetry.addLine(colorid);
            telemetry.update();
            sleep(1000);

            //knock ball

            //score glyph

            driveStraight(.25, 2000);



            telemetry.update();
            //break;
        }
    }
    private void driveStraight (double power, int time) {
        leftBack.setPower(power);
        rightBack.setPower(power);
        leftFront.setPower(power);
        rightFront.setPower(power);

        sleep(time);

        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }
    private void turn(double power, int time){
        //left turn is positive power
        leftBack.setPower(-power); //sets left wheels to move backward
        leftFront.setPower(-power);
        rightBack.setPower(power); // makes right hand wheels to move forward
        rightFront.setPower(power);
        sleep(time);
        //those things happen for this amount of time and then all the wheels stop
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    private void dump(double left, double right) {
        //setting the two dump servo to an input value
        leftDump.setPosition(left);
        rightDump.setPosition(right);
    }
    private void arm(double position) {
        //setting the color servo to an input value
        colorServo.setPosition(position);
    }
    private void sleep(int i) {
        //initial time takes the current hardware time in milliseconds
        long initial_time = System.currentTimeMillis();
        //inside the while loop cpu will stop working when the input time is more than the time passed in this loop
        //cpu will be back working when the loop reaches the target time
        while (System.currentTimeMillis() - initial_time < i) {

        }
    }
    public String checkColor(ColorSensor front, ColorSensor back, double ratio) {
        double redOverBluFront = (front.red()+1)/(front.blue()+1); //red over blue ratio for front color sensor
        double redOverBluBack = (back.red()+1)/(back.blue()+1);//red over blue ratio for back color sensor
        if(1/redOverBluBack >= ratio && redOverBluFront >= ratio){ //if front is red and back is blue, return red
            return "RED";
        }
        else if (((redOverBluBack)>=ratio) && ((1/redOverBluFront)>=ratio)){
            //if front is blue and back is red, return blue
            return "BLUE";
        }
        else if (((1/redOverBluBack)>=ratio) && ((redOverBluFront)<=ratio) && ((redOverBluFront)>=1/ratio)){
            //if back is blue and front is unsure, return red
            return "RED";
        }
        else if (((redOverBluBack)>=ratio) && ((redOverBluFront)<=ratio) && ((redOverBluFront)>=1/ratio)){
            //if back is red and front is unsure, return blue
            return "BLUE";
        }
        else if (((1/redOverBluFront)>=ratio) && ((redOverBluBack)<=ratio) && ((redOverBluBack)>=1/ratio)){
            //if front is blue and back is unsure, return blue
            return "BLUE";
        }
        else if (((redOverBluFront)>=ratio) && ((redOverBluBack)<=ratio) && ((redOverBluBack)>=1/ratio)){
            //if front is red and back is unsure, return red
            return "RED";
        }
        else {
            //if none of the above, we don't know what's happening, so return undefined.
            return "UNDEF";
        }
    }
}
