package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * step 1 for swerve is to figure out how each motor/servo/encoder behaves
 * this code helps with that by allowing controls for everything and logging all data into telemetry and FTCDashboard
 */
@Config
@TeleOp
public class ReadSensors extends LinearOpMode {

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

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        CRServoImplEx bls = hardwareMap.get(CRServoImplEx.class, "brs");
        CRServoImplEx brs = hardwareMap.get(CRServoImplEx.class, "bls");
        CRServoImplEx fls = hardwareMap.get(CRServoImplEx.class, "fls");
        CRServoImplEx frs = hardwareMap.get(CRServoImplEx.class, "frs");

        AnalogInput bli = hardwareMap.get(AnalogInput.class, "bli");
        AnalogInput bri = hardwareMap.get(AnalogInput.class, "bri");
        AnalogInput fli = hardwareMap.get(AnalogInput.class, "fli");
        AnalogInput fri = hardwareMap.get(AnalogInput.class, "fri");
        waitForStart();

        while (opModeIsActive()) {


            bl.setPower(-gamepad1.left_stick_y);
            br.setPower(-gamepad1.right_stick_y);
            fl.setPower(-gamepad2.left_stick_y);
            fr.setPower(-gamepad2.right_stick_y);

            bls.setPower(gamepad1.left_stick_x);
            brs.setPower(gamepad1.right_stick_x);
            fls.setPower(gamepad2.left_stick_x);
            frs.setPower(gamepad2.right_stick_x);


            if (gamepad1.a) {
                bls.setPwmEnable();
                brs.setPwmEnable();
                fls.setPwmEnable();
                frs.setPwmEnable();
            }

            if (gamepad1.b) {
                bls.setPwmDisable();
                brs.setPwmDisable();
                fls.setPwmDisable();
                frs.setPwmDisable();
            }

            telemetry.addData("back left", bli.getVoltage());
            telemetry.addData("back right", bri.getVoltage());
            telemetry.addData("front left", fli.getVoltage());
            telemetry.addData("front right", fri.getVoltage());
            telemetry.update();


            // TODO: multiplication value is encoder val / angle
        }
    }
}
