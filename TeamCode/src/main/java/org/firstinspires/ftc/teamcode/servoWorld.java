package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Servo;

public class servoWorld {
    Servo leftDump, FLICKSERVO, colorServo, rightDump;
    sleep sleepyRobot = new sleep();
    public servoWorld(Servo leftDump, Servo FLICKSERVO, Servo colorServo, Servo rightDump) {
        this.leftDump = leftDump;
        this.FLICKSERVO = FLICKSERVO;
        this.colorServo = colorServo;
        this.rightDump = rightDump;
    }
    public void dump(double left, double right) {
        //setting the two dump servo to an input value
        leftDump.setPosition(left);
        // rightDump.setPosition(right);
    }
    public void FLICKSERVO(double position) {
        //setting the FLICKSERVO servo to an input value
        FLICKSERVO.setPosition(position);
        sleepyRobot.robotSleep(2000);
        FLICKSERVO.setPosition(0.5);

    }

    public void arm(double position) {
        //setting the color servo to an input value
        colorServo.setPosition(position);
    }
    public void rampDown(){
        leftDump.setPosition(.5);
        rightDump.setPosition(.5);

        leftDump.setPosition(.8);
        rightDump.setPosition(.4);

        sleepyRobot.robotSleep(2000);

        leftDump.setPosition(.5);
        rightDump.setPosition(.5);
    }
}
