package org.firstinspires.ftc.teamcode.test_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="single_flywheel",group="testing")
@Disabled
public class single_flywheel extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();

    DcMotor mtrWheel;
    Servo svoTransfer;

    final double noSpeed = 0;
    double speedValue = 0;
    final double speedIncrement = 0.1;
    public boolean isItOn = false;
    public double powerTimer = 0;


    public void runOpMode() {
        telemetry.addData("Status:", "Begin init");
        telemetry.update();

        //motor 0
        mtrWheel = hardwareMap.get(DcMotor.class, "mtrWheel");
        mtrWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mtrWheel.setDirection(DcMotor.Direction.FORWARD);
        mtrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        svoTransfer = hardwareMap.get(Servo.class, "svoTransfer");
        svoTransfer.setDirection(Servo.Direction.FORWARD);

        svoTransfer.setPosition(0.32);

        waitForStart();
        runtime.reset();

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        while (opModeIsActive()) {

            while(gamepad1.b){
                setSpeed(1);
            }
            /*
            if(gamepad1.b&&powerTimer<0.1&&isItOn==false){
                isItOn=true;
                powerTimer=10;
            }
            if(isItOn=true){
                setSpeed(0.5);
            }
            if(gamepad1.b&&powerTimer<0.1&&isItOn==true){
                isItOn=false;
                powerTimer=10;
            }
            if(powerTimer>0){
                powerTimer=powerTimer-0.1;
            }

             */

            if(gamepad1.y){
                svoTransfer.setPosition(0.65);
            }
            if(gamepad1.x){
                svoTransfer.setPosition(0.32);
            }

            if(gamepad1.a){
                setSpeed(noSpeed);
                isItOn=false;
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Gamepad value = ", gamepad1.left_stick_y);
            telemetry.addData("Speed = ", speedValue);
            telemetry.addData("Speed Increment = ", speedIncrement);
            telemetry.update();
        }



    }
    private void setSpeed(double speed){
        mtrWheel.setPower(speed);
    }
    private void waitFor(double waittime) {
        runtime.reset();
        while (runtime.seconds() < waittime) {
            //don't do anything pls
        }
    }


}
