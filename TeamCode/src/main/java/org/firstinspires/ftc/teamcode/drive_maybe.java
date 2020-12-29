package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="drive",group="Linear Opmode")

public class drive_maybe extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();

    DcMotor mtrBL , mtrBR , mtrFL , mtrFR , mtrIntake;

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

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));

            

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }


    }


}
