package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="teleop control swap",group="teleop")
public class drive_controls_swapped extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();

    DcMotor mtrBL , mtrBR , mtrFL , mtrFR , mtrIntake;
    Servo svoWobble;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        mtrBL = hardwareMap.get(DcMotor.class, "mtrBL");
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setDirection(DcMotor.Direction.REVERSE);

        mtrBR = hardwareMap.get(DcMotor.class, "mtrBR");
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setDirection(DcMotor.Direction.FORWARD);

        mtrFL = hardwareMap.get(DcMotor.class, "mtrFL");
        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFL.setDirection(DcMotor.Direction.REVERSE);

        mtrFR = hardwareMap.get(DcMotor.class, "mtrFR");
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setDirection(DcMotor.Direction.FORWARD);

        mtrIntake = hardwareMap.get(DcMotor.class, "mtrIntake");
        mtrIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrIntake.setDirection(DcMotor.Direction.FORWARD);

        svoWobble = hardwareMap.get(Servo.class,"svoWobble");
        svoWobble.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            mtrBL.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x));
            mtrBR.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
            mtrFL.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));
            mtrFR.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));

            if (gamepad1.a){
                mtrIntake.setPower(0.8);
            }
            if (gamepad1.b){
                mtrIntake.setPower(0);
            }
            if (gamepad1.x){
                mtrIntake.setPower(-0.8);
            }
            if (gamepad1.right_bumper){
                svoWobble.setPosition(0.3);
            }
            if (gamepad1.left_bumper){
                svoWobble.setPosition(0.95);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }


    }


}
