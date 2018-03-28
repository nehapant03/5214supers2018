package org.firstinspires.ftc.teamcode;

public class sleep {
    private int i;
    public sleep(int i) {
        this.i = i;
    }
    public void robotSleep(int i) {
        //initial time takes the current hardware time in milliseconds
        long initial_time = System.currentTimeMillis();
        //inside the while loop cpu will stop working when the input time is more than the time passed in this loop
        //cpu will be back working when the loop reaches the target time

        while (System.currentTimeMillis() - initial_time < i) {

        }
    }
}
