package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Step 3
 * Implement the PID values and angle/voltage conversions into a (hopefully) successful driving class
 */
@Config
@TeleOp
public class SwerveMain extends LinearOpMode {
    static public double L = 12; //wheelbase
    static public double W = 12; //trackwidth
    static public DcMotorEx bl;
    static public DcMotorEx br;
    static public DcMotorEx fl;
    static public DcMotorEx fr;
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    static public PIDController blc;
    static public PIDController brc;
    static public PIDController flc;
    static public PIDController frc;
    public static CRServo bls;
    public static CRServo brs;
    public static CRServo fls;
    public static CRServo frs;
    public static AnalogInput bli;
    public static AnalogInput bri;
    public static AnalogInput fli;
    public static AnalogInput fri;
    @Override
    public void runOpMode() throws InterruptedException {

        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bls = hardwareMap.get(CRServo.class, "brs");
        brs = hardwareMap.get(CRServo.class, "bls");
        fls = hardwareMap.get(CRServo.class, "fls");
        frs = hardwareMap.get(CRServo.class, "frs");

        bli = hardwareMap.get(AnalogInput.class, "bli");
        bri = hardwareMap.get(AnalogInput.class, "bri");
        fli = hardwareMap.get(AnalogInput.class, "fli");
        fri = hardwareMap.get(AnalogInput.class, "fri");

        waitForStart();
        while (opModeIsActive()) {

            //do swerve stuff

            double val = drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            telemetry.addData("targetAngle", val);
            telemetry.update();


        }
    }

    public static double drive(double x, double y, double t) {

        double R = Math.hypot(L, W);

        double a = x - t * (L/R);
        double b = x + t * (L/R);
        double c = y - t * (W/R);
        double d = y + t * (W/R);

        // These values range from 0 to 1... we need it to range from -1 to 1
        double brs = (Math.hypot(a, d) - 0.5) * 2;
        double bls = (Math.hypot(a, c) - 0.5) * 2;
        double frs = (Math.hypot(b, d) - 0.5) * 2;
        double fls = (Math.hypot(b, c) - 0.5) * 2;

        double bra = (Math.atan2(a, d) / Math.PI) * 180; // added multiplication by 180 to return actual degrees (easier for conversion purposes)
        double bla  = (Math.atan2(a, c) / Math.PI) * 180;
        double fra  = (Math.atan2(b, d) / Math.PI) * 180;
        double fla  = (Math.atan2(b, c) / Math.PI) * 180;


        setMotorSpeeds(brs, bls, frs, fls);
        setServoAngles(bla, bra, fra, fla);

        return bla; // return a value for debugging
    }

    public static void setMotorSpeeds(double brs, double bls, double frs, double fls) {
        br.setPower(brs);
        bl.setPower(bls);
        fr.setPower(frs);
        fl.setPower(fls);
    }

    public static void setServoAngles(double bla, double bra, double fra, double fla) {
        blc.setPID(p, i, d);
        brc.setPID(p, i, d);
        frc.setPID(p, i, d);
        flc.setPID(p, i, d);

        //TODO: do conversion from target angle to analog val

        //TODO: "wrap" the angle back when the range exceeds possibility

        bls.setPower(blc.calculate(bli.getVoltage(), bla));

    }


}
