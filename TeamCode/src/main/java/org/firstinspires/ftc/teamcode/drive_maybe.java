package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="teleop dt+intake",group="Linear Opmode")

public class drive_maybe extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();
    DcMotorEx mtrBL , mtrBR , mtrFL , mtrFR , mtrIntake;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        mtrBL = hardwareMap.get(DcMotorEx.class, "mtrBL");
        mtrBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrBL.setDirection(DcMotorEx.Direction.REVERSE);

        mtrBR = hardwareMap.get(DcMotorEx.class, "mtrBR");
        mtrBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrBR.setDirection(DcMotorEx.Direction.FORWARD);

        mtrFL = hardwareMap.get(DcMotorEx.class, "mtrFL");
        mtrFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrFL.setDirection(DcMotorEx.Direction.REVERSE);

        mtrFR = hardwareMap.get(DcMotorEx.class, "mtrFR");
        mtrFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrFR.setDirection(DcMotorEx.Direction.FORWARD);

        mtrIntake = hardwareMap.get(DcMotorEx.class, "mtrIntake");
        mtrIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrIntake.setDirection(DcMotorEx.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));

            if (gamepad1.a){
                mtrIntake.setPower(0.8);
            }
            if (gamepad1.b){
                mtrIntake.setPower(0);
            }
            if (gamepad1.x){
                mtrIntake.setPower(-0.8);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }


    }


}
