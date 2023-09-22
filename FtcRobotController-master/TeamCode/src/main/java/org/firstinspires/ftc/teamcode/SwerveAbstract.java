package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AbsoluteAnalogEncoder;
import org.opencv.core.Mat;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;


/**
 * Step 3
 * Implement the PID values and angle/voltage conversions into a (hopefully) successful driving class
 */
@Config
@TeleOp
public class SwerveAbstract extends LinearOpMode {
    static public double L = 12; //wheelbase
    static public double W = 12; //trackwidth

    public static double p = 0.01;
    public static double i = 0.1;
    public static double di = 0;

    public static final double WAIT_OFFSET = 5;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "bl");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "br");
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "fl");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "fr");

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        CRServo bls = hardwareMap.get(CRServo.class, "bls");
        CRServo brs = hardwareMap.get(CRServo.class, "brs");
        CRServo fls = hardwareMap.get(CRServo.class, "fls");
        CRServo frs = hardwareMap.get(CRServo.class, "frs");

        AbsoluteAnalogEncoder bli = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "bli"), 3.3).zero(5.068);
        AbsoluteAnalogEncoder bri = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "bri"), 3.3).zero(5.52);
        AbsoluteAnalogEncoder fli = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "fli"), 3.3).zero(1.69);
        AbsoluteAnalogEncoder fri = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "fri"), 3.3).zero(1.24);

        PIDController blc = new PIDController(p,i,di);
        PIDController brc = new PIDController(p,i,di);
        PIDController flc = new PIDController(p,i,di);
        PIDController frc = new PIDController(p,i,di);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        imu.resetYaw();

        waitForStart();

        while (opModeIsActive()) {

            //TODO: make gamepad input not negative?
            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double t = -gamepad1.right_stick_x;

            //SuperMegaDrive
            /*double head = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double relativeXToPoint = Math.cos(Math.toRadians(90)-Math.toRadians(head));
            double relativeYToPoint = Math.sin(Math.toRadians(90)-Math.toRadians(head));

            double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            x = (movementXPower*y)+(movementYPower*x);
            y = (movementYPower*-y)+(movementXPower*x);

             */

            double R = Math.hypot(L, W);

            double a = x - t * (L/R);
            double b = x + t * (L/R);
            double c = y - t * (W/R);
            double d = y + t * (W/R);

            double brsv = (Math.hypot(a, c));
            double blsv = (Math.hypot(a, d));
            double frsv = (Math.hypot(b, c));
            double flsv = (Math.hypot(b, d));

            double bra = (Math.atan2(a, c) * 180 / Math.PI); // added multiplication by 180 to return actual degrees (easier for conversion purposes)
            double bla = (Math.atan2(a, d) * 180 / Math.PI);
            double fra = (Math.atan2(b, c) * 180 / Math.PI);
            double fla = (Math.atan2(b, d) * 180 / Math.PI);

            telemetry.addData("t", t);
            telemetry.addData("br raw", bra);
            telemetry.addData("bl raw", bla);
            telemetry.addData("fl raw", fla);
            telemetry.addData("fr raw", fra);

            if (y == 0 && x == 0 && t==0) {
                bra=0;
                bla=0;
                fra=0;
                fla=0;
            }


            double blreading = convertTo180(Math.toDegrees(bli.getCurrentPosition()));
            double brreading = convertTo180(Math.toDegrees(bri.getCurrentPosition()));
            double flreading = convertTo180(Math.toDegrees(fli.getCurrentPosition()));
            double frreading = convertTo180(Math.toDegrees(fri.getCurrentPosition()));


            double[] brOptimized = optimize(bra, brreading, brsv);
            double[] blOptimized = optimize(bla, blreading, blsv);
            double[] frOptimized = optimize(fra, frreading, frsv);
            double[] flOptimized = optimize(fla, flreading, flsv);

            if (brOptimized[2] == 1) {
                //invert!
                brs.setPower(-brc.calculate(brreading, brOptimized[0]));
                telemetry.addData("br", "inverted");
            } else {
                brs.setPower(brc.calculate(brreading, brOptimized[0]));
                telemetry.addData("yuh", brOptimized[1]);
            }
            if (blOptimized[2] == 1) {
                //invert!
                bls.setPower(-blc.calculate(blreading, blOptimized[0]));
                telemetry.addData("bl", "inverted");
            } else {
                bls.setPower(blc.calculate(blreading, blOptimized[0]));
            }
            if (frOptimized[2] == 1) {
                //invert!
                frs.setPower(-frc.calculate(frreading, frOptimized[0]));
                telemetry.addData("fr", "inverted");
            } else {
                frs.setPower(frc.calculate(frreading, frOptimized[0]));
            }
            if (flOptimized[2] == 1) {
                //invert!
                fls.setPower(-flc.calculate(flreading, flOptimized[0]));
                telemetry.addData("fl", "inverted");
            } else {
                fls.setPower(flc.calculate(flreading, flOptimized[0]));
            }


            double blPower = (blOptimized[1]);
            double brPower = (-brOptimized[1]);
            double flPower = (flOptimized[1]);
            double frPower = (-frOptimized[1]);

            bl.setPower(blPower);
            br.setPower(brPower);
            fl.setPower(flPower);
            fr.setPower(frPower);





            //br.setPower(-brsv);
            //bl.setPower(blsv);
            //fr.setPower(-frsv);
            //fl.setPower(flsv);


            telemetry.addData("bl angle", blreading);
            telemetry.addData("br angle", brreading);
            telemetry.addData("fl angle", flreading);
            telemetry.addData("fr angle", frreading);
            telemetry.addData("bl target", blOptimized[0]);
            telemetry.addData("br target", brOptimized[0]);
            telemetry.update();


        }
    }


    public static double convertTo180(double degrees) {
        if (degrees > 180) {
            // Subtract 360 from degrees greater than 180 to bring them into the -180 to 180 range
            degrees -= 360;
        }

        // Make sure degrees are still in the -180 to 180 range
        degrees %= 360;

        return degrees;
    }

    public static double max(double... args){
        double max = args[0];
        for(double d : args){
            if(d > max) max = d;
        }
        return max;
    }


    public static double[] optimize(double v1, double curPose, double speedy) {
        double[] output = new double[3];
        double opt1 = v1;
        double opt2 = 0;
        double curPos = curPose;
        double speed = speedy;
        if (opt1 < 0) {
            opt2 = opt1 + 180;
        } else if (opt1 > 0) {
            opt2 = opt1 - 180;
        }
        double err1 = opt1 - curPos;
        double err2 = opt2 - curPos;

        if (err1 < -180) {
            err1 += 360;
        } else if (err1 > 180) {
            err1 -= 360;
        }
        if (err2 < -180) {
            err2 += 360;
        } else if (err2 > 180) {
            err2 -= 360;
        }

        double target;
        if (Math.abs(err1) <= Math.abs(err2)) {
            target = curPos + err1;
        } else {
            target = curPos + err2;
            speed = speed * -1;
        }

        if (target > 180) {
            target -= 360;
        } else if (target < -180) {
            target += 360;
        }
        if ((((target - curPos) > 180) || ((target - curPos) < -180)) && ((target*curPos) < 0)) {

            output = new double[] {target, speed, 1};
        } else {

            output = new double[] {target, speed, 0};
        }
        return output;
    }

    /**
     * Makes sure the pod is rotated before you drive
     * @param speed
     * @param target
     * @param curPos
     * @return a speed parameter that is either the original or 0
     */
    public static double checkDrive(double speed, double target, double curPos) {
        if ((Math.abs(target) > Math.abs(curPos) + 45) || (Math.abs(target) < Math.abs(curPos) - 45)) {
            speed = 0;
        }
        return speed;
    }

}
