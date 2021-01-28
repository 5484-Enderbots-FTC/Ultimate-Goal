package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="single_flywheel",group="testing")
@Disabled
public class single_flywheel extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();

    DcMotor mtrWheel;

    final double noSpeed = 0;
    double speedValue = 0;
    final double speedIncrement = 0.1;

    public void runOpMode() {
        telemetry.addData("Status:", "Begin init");
        telemetry.update();

        //motor 0
        mtrWheel = hardwareMap.get(DcMotor.class, "mtrWheel");
        mtrWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrWheel.setDirection(DcMotor.Direction.REVERSE);
        mtrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        while (opModeIsActive()) {

            setSpeed(gamepad1.left_stick_y);

            if(gamepad1.dpad_up){
                speedValue = speedValue + speedIncrement;
                setSpeed(speedValue);
            }
            if(gamepad1.dpad_down){
                speedValue = speedValue - speedIncrement;
                setSpeed(speedValue);
            }

            if(gamepad1.a){
                speedValue = noSpeed;
                setSpeed(speedValue);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed = ", speedValue);
            telemetry.addData("Speed Increment = ", speedIncrement);
            telemetry.update();
        }



    }
    private void setSpeed(double speed){
        mtrWheel.setPower(speed);
    }


}
