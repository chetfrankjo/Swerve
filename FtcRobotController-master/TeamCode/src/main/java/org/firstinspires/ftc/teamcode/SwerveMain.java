package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.AbsoluteAnalogEncoder;
import org.opencv.core.Mat;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;


/**
 * Step 3
 * Implement the PID values and angle/voltage conversions into a (hopefully) successful driving class
 */
@Config
@TeleOp
public class SwerveMain extends LinearOpMode {
    static public double L = 12; //wheelbase
    static public double W = 12; //trackwidth

    public static double p = 0.01;
    public static double i = 0.1;
    public static double di = 0;

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

        //store forward offsets

        double prevBL = 0;
        double prevBR = 0;
        double prevFR = 0;
        double prevFL = 0;


        waitForStart();
        double BL_OFFSET = bli.getVoltage() * 3.3 / -360;
        double BR_OFFSET = bri.getVoltage() * 3.3 / -360;
        double FL_OFFSET = fli.getVoltage() * 3.3 / -360;
        double FR_OFFSET = fri.getVoltage() * 3.3 / -360;
        while (opModeIsActive()) {



            //TODO: make gamepad input not negative?
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double t = -gamepad1.right_stick_x;


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

            /*if (blreading > 180) {
                blreading = blreading - 360;
            }
            if (brreading > 180) {
                brreading = brreading - 360;
            }
            if (flreading > 180) {
                flreading = flreading - 360;
            }
            if (frreading > 180) {
                frreading = frreading - 360;
            }
             */





            /*if (fla < 0) {
                fla = fla + 360;
            }
            if (fra < 0) {
                fra = fra + 360;
            }
            if (bla < 0) {
                bla = bla + 360;
            }
            if (bra < 0) {
                bra = bra + 360;
            }

             */


            if (bra < 0) {
                if (Math.abs(bra - prevBR) > Math.abs(bra+180-prevBR)) {
                    bra = bra + 180;
                    brsv = -brsv;
                }
            } else {
                if (Math.abs(bra - prevBR) > Math.abs(bra-180-prevBR)) {
                    bra = bra - 180;
                    brsv = -brsv;
                }
            }
            if (bla < 0) {
                if (Math.abs(bla - prevBL) > Math.abs(bla+180-prevBL)) {
                    bla = bla + 180;
                    blsv = -blsv;
                }
            } else {
                if (Math.abs(bla - prevBL) > Math.abs(bla-180-prevBL)) {
                    bla = bla - 180;
                    blsv = -blsv;
                }
            }
            if (fra < 0) {
                if (Math.abs(fra - prevFR) > Math.abs(fra+180-prevFR)) {
                    fra = fra + 180;
                    frsv = -frsv;
                }
            } else {
                if (Math.abs(fra - prevFR) > Math.abs(fra-180-prevFR)) {
                    fra = fra - 180;
                    frsv = -frsv;
                }
            }
            if (fla < 0) {
                if (Math.abs(fla - prevFL) > Math.abs(fla+180-prevFL)) {
                    fla = fla + 180;
                    flsv = -flsv;
                }
            } else {
                if (Math.abs(fla - prevFL) > Math.abs(fla-180-prevFL)) {
                    fla = fla - 180;
                    flsv = -flsv;
                }
            }

            frc.calculate(1000);



            // invert speed to eliminate over-turning of pod
            /*if (bra > 90) {
                bra = bra - 180;
                brsv = -brsv;
            } else if (bra < -90) {
                bra = bra + 180;
                brsv = -brsv;
            }
            if (bla > 90) {
                bla = bla - 180;
                blsv = -blsv;
            } else if (bla < -90) {
                bla = bla + 180;
                blsv = -blsv;
            }
            if (fra > 90) {
                fra = fra - 180;
                frsv = -frsv;
            } else if (fra < -90) {
                fra = fra + 180;
                frsv = -frsv;
            }
            if (fla > 90) {
                fla = fla - 180;
                flsv = -flsv;
            } else if (fla < -90) {
                fla = fla + 180;
                flsv = -flsv;
            }

             */


            if (y == 0 && x == 0 && t==0) {
                bra=0;
                bla=0;
                fra=0;
                fla=0;
            }


            double[] speeds = new double[] {brsv, blsv, frsv, flsv};
            double speed = max(speeds);

            br.setPower(-brsv);
            bl.setPower(blsv);
            fr.setPower(-frsv);
            fl.setPower(flsv);


            blc.setPID(p, i, di);
            brc.setPID(p, i, di);
            frc.setPID(p, i, di);
            flc.setPID(p, i, di);


            // if you have very large error, go the "wrong" direction
            if (Math.abs(bla-prevBL) > 180) {
                bls.setPower(-blc.calculate(blreading, bla));
            } else {
                bls.setPower(blc.calculate(blreading, bla));
            }
            if (Math.abs(bra-prevBR) > 180) {
                brs.setPower(-brc.calculate(brreading, bra));
            } else {
                brs.setPower(brc.calculate(brreading, bra));
            }
            if (Math.abs(fla-prevFL) > 180) {
                fls.setPower(-flc.calculate(flreading, fla));
            } else {
                fls.setPower(flc.calculate(flreading, fla));
            }
            if (Math.abs(fra-prevFR) > 180) {
                frs.setPower(-frc.calculate(frreading, fra));
            } else {
                frs.setPower(frc.calculate(frreading, fra));
            }



            telemetry.addData("bl angle", blreading);
            telemetry.addData("br angle", brreading);
            telemetry.addData("fl angle", flreading);
            telemetry.addData("fr angle", frreading);
            telemetry.addData("bl target", bla);
            telemetry.addData("br target", bra);
            telemetry.update();

            prevBL = blreading;
            prevBR = brreading;
            prevFL = flreading;
            prevFR = frreading;
        }
    }
/*
    public static double drive(double x, double y, double t) {

        double R = Math.hypot(L, W);

        double a = x - t * (L/R);
        double b = x + t * (L/R);
        double c = y - t * (W/R);
        double d = y + t * (W/R);


        double brs = (Math.hypot(a, d));
        double bls = (Math.hypot(a, c));
        double frs = (Math.hypot(b, d));
        double fls = (Math.hypot(b, c));

        double bra = (Math.atan2(a, d) * 180 / Math.PI); // added multiplication by 180 to return actual degrees (easier for conversion purposes)
        double bla  = (Math.atan2(a, c) * 180 / Math.PI);
        double fra  = (Math.atan2(b, d) * 180 / Math.PI);
        double fla  = (Math.atan2(b, c) * 180 / Math.PI);

        // invert speed to eliminate over-turning of pod
        /*if (bra > 90) {
            bra = bra - 180;
            brs = -brs;
        } else if (bra < -90) {
            bra = bra + 180;
            brs = -brs;
        }
        if (bla > 90) {
            bla = bla - 180;
            bls = -bls;
        } else if (bra < -90) {
            bls = bla + 180;
            bls = -bls;
        }
        if (fra > 90) {
            fra = bra - 180;
            frs = -frs;
        } else if (fra < -90) {
            fra = fra + 180;
            frs = -frs;
        }
        if (fla > 90) {
            fla = fla - 180;
            fls = -fls;
        } else if (fla < -90) {
            fla = fla + 180;
            fls = -fls;
        }

         */

/*
        setMotorSpeeds(brs, bls, frs, fls);
        setServoAngles(bla, bra, fra, fla);

        return y; // return a value for debugging
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

        //TODO: "wrap" the angle back when the range exceeds possibility
        //      This seems to be done "automatically" for the most part... TBD

        bls.setPower(blc.calculate((bli.getVoltage() / 3.3 * -360)-BL_OFFSET, bla));
        brs.setPower(brc.calculate((bri.getVoltage() / 3.3 * -360)-BR_OFFSET, bra));
        frs.setPower(frc.calculate((fli.getVoltage() / 3.3 * -360)-FR_OFFSET, fra));
        fls.setPower(flc.calculate((fri.getVoltage() / 3.3 * -360)-FL_OFFSET, fla));
    }
    */

    public static double convertAngle(double degrees) {
        if (degrees < 0) {
            // Add 360 to negative degrees to bring them into the 0-360 range
            degrees += 360;
        }

        // Make sure degrees are still in the 0-360 range
        degrees %= 360;

        return degrees;
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
}
