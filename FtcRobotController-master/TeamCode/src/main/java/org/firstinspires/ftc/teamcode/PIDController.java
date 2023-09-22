package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;
    private double setpoint;
    private double integral;
    private double previousError;

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double calculate(double currentError) {
        double deltaTime = 0.01; // Time between iterations (adjust as needed)

        // Calculate proportional, integral, and derivative terms
        double proportional = Kp * currentError;
        integral += Ki * currentError * deltaTime;
        double derivative = Kd * (currentError - previousError) / deltaTime;

        // Calculate the PID output
        double output = proportional + integral + derivative;

        // Store the current error for the next iteration
        previousError = currentError;

        return output;
    }
}
