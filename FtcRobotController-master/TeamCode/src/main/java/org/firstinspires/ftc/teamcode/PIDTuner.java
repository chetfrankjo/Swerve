package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Step 2
 * This code is needed to tune the PID for the servo movement
 * The values can be adjusted in FTCDashboard to make tuning easier
 * the goal is to make the servo reach the desired position quickly with good accuracy
 */
@Config
@TeleOp
public class PIDTuner extends LinearOpMode {
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double targetAngle = 0;
    PIDController pid;
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo s = hardwareMap.get(CRServo.class, "fls");
        AnalogInput input = hardwareMap.get(AnalogInput.class, "fli");

        waitForStart();


        while (opModeIsActive()) {
            pid.setPID(p, i, d);

            // TODO: convert target to voltage target (from step 1)
            targetAngle = targetAngle * 0.00;

            double output = pid.calculate(input.getVoltage(), targetAngle);

            telemetry.addData("input", input.getVoltage());
            telemetry.addData("output", output);
            telemetry.update();
            s.setPower(output);

        }
    }
}
