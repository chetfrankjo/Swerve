package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    public static double p = 0.01;
    public static double i = 0.1;
    public static double di = 0;
    public static double targetAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        CRServo bls = hardwareMap.get(CRServo.class, "bls");
        CRServo brs = hardwareMap.get(CRServo.class, "brs");
        CRServo fls = hardwareMap.get(CRServo.class, "fls");
        CRServo frs = hardwareMap.get(CRServo.class, "frs");

        AnalogInput bli = hardwareMap.get(AnalogInput.class, "bli");
        AnalogInput bri = hardwareMap.get(AnalogInput.class, "bri");
        AnalogInput fli = hardwareMap.get(AnalogInput.class, "fli");
        AnalogInput fri = hardwareMap.get(AnalogInput.class, "fri");

        PIDController blc = new PIDController(p,i,di);
        PIDController brc = new PIDController(p,i,di);
        PIDController flc = new PIDController(p,i,di);
        PIDController frc = new PIDController(p,i,di);
        final FtcDashboard dashboard;
        dashboard = FtcDashboard.getInstance();
        waitForStart();


        while (opModeIsActive()) {

            blc.setPID(p,i,di);
            brc.setPID(p,i,di);
            flc.setPID(p,i,di);
            frc.setPID(p,i,di);

            // TODO: convert target to voltage target (from step 1)
            bls.setPower((targetAngle - (bli.getVoltage() / 3.3 * -360)+245) * p);
            //bls.setPower(-blc.calculate((bli.getVoltage() / 3.3 * -360)+245, targetAngle));
            //brs.setPower(-brc.calculate((bri.getVoltage() / 3.3 * -360)+172, targetAngle));
            //fls.setPower(flc.calculate((fli.getVoltage() / 3.3 * -360)+82.8,targetAngle));
            //frs.setPower(frc.calculate((fri.getVoltage() / 3.3 * -360)+110.0, targetAngle));

            telemetry.addData("input", (bli.getVoltage() / 3.3 * -360)+245);
            telemetry.update();


            TelemetryPacket packet = new TelemetryPacket();

            packet.put("input",(bli.getVoltage() / 3.3 * -360)+245);
            packet.put("tager", targetAngle);
            dashboard.sendTelemetryPacket(packet);

        }
    }
}
