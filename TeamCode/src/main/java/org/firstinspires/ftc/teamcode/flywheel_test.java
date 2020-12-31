package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="flywheel_test",group="Linear Opmode")
@Disabled
public class flywheel_test extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();

    DcMotor mtrLeft , mtrRight;
    //Servo svoLoad;

    final double noSpeed = 0;
    double speedValue = 0;
    final double speedIncrement = 0.0001;
    //double load = 0;

    public void runOpMode() {
        telemetry.addData("Status:", "Begin init");
        telemetry.update();

        //motor 0
        mtrLeft = hardwareMap.get(DcMotor.class, "mtrLeft");
        mtrLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrLeft.setDirection(DcMotor.Direction.REVERSE);
        mtrLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //motor 2
        mtrRight = hardwareMap.get(DcMotor.class, "mtrRight");
        mtrRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrRight.setDirection(DcMotor.Direction.FORWARD);
        mtrRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        svoLoad = hardwareMap.get(Servo.class, "extend");
        svoLoad.setDirection(Servo.Direction.FORWARD);

        svoLoad.setPosition(load);
         */

        waitForStart();
        runtime.reset();

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                speedValue += speedIncrement;
                setSpeed(speedValue);
            }
            if(gamepad1.dpad_down){
                speedValue -= speedIncrement;
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
        mtrLeft.setPower(speed);
        if(speedValue == 0){
            mtrRight.setPower(speed);
        }
        else{
            mtrRight.setPower(speed+(speed*.1));
        }
    }


}
