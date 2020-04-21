package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double kp;
    private double ki;
    private double kd;
    private double setPoint;
    private double lastError;
    private double errorSum;
    private long lastTime;
    public PIDController(double setPoint, double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.setPoint = setPoint;

        lastError = 0;
        lastTime = System.currentTimeMillis();
        errorSum = 0;
    }
    public float update(double newInput){
        long time = System.currentTimeMillis();
        long period = time - lastTime;
        double error;

        error = setPoint - newInput;

        if((int)Math.signum(lastError) != (int)Math.signum(error)){
            errorSum = 0;
        }

        errorSum += error * period;
        double derError = (error - lastError) / period;

        double output = kp * error + ki *errorSum + kd *derError;

        lastError = error;
        lastTime = time;
        return (float) output;
        //make kp higher than ki because integral is the area under the curve and would

    }
    //everytime you reach your setpoint meaning your error(check paper doc) is 0, set your integral to 0
    //constantly adding current error + change in time,
    //error vs time curve using rectangles
    //apply constants
}
