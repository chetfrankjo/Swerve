package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.AbsoluteAnalogEncoder;
import org.opencv.core.Mat;

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
        AbsoluteAnalogEncoder bri = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "bri"), 3.3).zero(4.44);
        AbsoluteAnalogEncoder fli = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "fli"), 3.3).zero(1.69);
        AbsoluteAnalogEncoder fri = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "fri"), 3.3).zero(1.24);

        PIDController blc = new PIDController(p,i,di);
        PIDController brc = new PIDController(p,i,di);
        PIDController flc = new PIDController(p,i,di);
        PIDController frc = new PIDController(p,i,di);

        //store forward offsets


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

            /*double brsv = (Math.hypot(a, d));
            double blsv = (Math.hypot(a, c));
            double frsv = (Math.hypot(b, d));
            double flsv = (Math.hypot(b, c));

            double bra = (Math.atan2(a, d) * 180 / Math.PI); // added multiplication by 180 to return actual degrees (easier for conversion purposes)
            double bla = (Math.atan2(a, c) * 180 / Math.PI);
            double fra = (Math.atan2(b, d) * 180 / Math.PI);
            double fla = (Math.atan2(b, c) * 180 / Math.PI);

             */



            double brsv = (Math.hypot(a, c));
            double blsv = (Math.hypot(a, d));
            double frsv = (Math.hypot(b, c));
            double flsv = (Math.hypot(b, d));

            double bra = (Math.atan2(a, c) * 180 / Math.PI); // added multiplication by 180 to return actual degrees (easier for conversion purposes)
            double bla = (Math.atan2(a, d) * 180 / Math.PI);
            double fra = (Math.atan2(b, c) * 180 / Math.PI);
            double fla = (Math.atan2(b, d) * 180 / Math.PI);


            double blreading = Math.toDegrees(bli.getCurrentPosition())-180;
            double brreading = Math.toDegrees(bri.getCurrentPosition())-180;
            double flreading = Math.toDegrees(fli.getCurrentPosition())-180;
            double frreading = Math.toDegrees(fri.getCurrentPosition())-180;

            if (blreading > 180) {
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




            // invert speed to eliminate over-turning of pod
            if (bra > 145) {
                bra = bra - 180;
                brsv = -brsv;
            } else if (bra < -145) {
                bra = bra + 180;
                brsv = -brsv;
            }
            if (bla > 145) {
                bla = bla - 180;
                blsv = -blsv;
            } else if (bla < -145) {
                bla = bla + 180;
                blsv = -blsv;
            }
            if (fra > 145) {
                fra = fra - 180;
                frsv = -frsv;
            } else if (fra < -145) {
                fra = fra + 180;
                frsv = -frsv;
            }
            if (fla > 145) {
                fla = fla - 180;
                flsv = -flsv;
            } else if (fla < -145) {
                fla = fla + 180;
                flsv = -flsv;
            }



            br.setPower(brsv);
            bl.setPower(-blsv);
            fr.setPower(frsv);
            fl.setPower(-flsv);


            blc.setPID(p, i, di);
            brc.setPID(p, i, di);
            frc.setPID(p, i, di);
            flc.setPID(p, i, di);

            //TODO: "wrap" the angle back when the range exceeds possibility
            //      This seems to be done "automatically" for the most part... TBD

            bls.setPower(blc.calculate(blreading, bla));
            brs.setPower(brc.calculate(brreading, bra));
            frs.setPower(frc.calculate(frreading, fra));
            fls.setPower(flc.calculate(flreading, fla));





            telemetry.addData("bl angle", blreading);
            telemetry.addData("br angle", brreading);
            telemetry.addData("fl angle", flreading);
            telemetry.addData("fr angle", frreading);
            telemetry.addData("bl target", bla);
            telemetry.update();


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


}
