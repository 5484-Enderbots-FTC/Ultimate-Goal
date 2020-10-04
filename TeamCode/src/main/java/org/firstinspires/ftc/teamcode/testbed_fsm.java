package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="testbed_fsm",group="Linear Opmode")

public class testbed_fsm extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();

    DcMotor mtrBL , mtrBR , mtrFL , mtrFR;
    Servo svoLHook, svoRHook;

    final double Ldown = 0.75;
    final double Lup = 0.5;
    final double Rdown = 0.25;
    final double Rup = 0.5;
    final double hookTime = 1;

    private enum HooksState {
        HOOKS_UP,
        HOOKS_DOWN,
        LEFT_UP,
        RIGHT_UP
    }

    HooksState hooksState = HooksState.HOOKS_UP;

    public void runOpMode() {
        telemetry.addData("Status:", "Begin init");
        telemetry.update();

        //motor 3
        mtrBL = hardwareMap.get(DcMotor.class, "leftFront_drive");
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setDirection(DcMotor.Direction.REVERSE);
        //motor 1
        mtrBR = hardwareMap.get(DcMotor.class, "rightBack_drive");
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setDirection(DcMotor.Direction.FORWARD);
        //motor 0
        mtrFL = hardwareMap.get(DcMotor.class, "leftBack_drive");
        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFL.setDirection(DcMotor.Direction.REVERSE);
        //motor 2
        mtrFR = hardwareMap.get(DcMotor.class, "rightFront_drive");
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setDirection(DcMotor.Direction.FORWARD);

        svoLHook = hardwareMap.get(Servo.class, "svoLHook");
        svoLHook.setDirection(Servo.Direction.REVERSE);

        svoRHook = hardwareMap.get(Servo.class, "svoRHook");
        svoRHook.setDirection(Servo.Direction.REVERSE);

        svoLHook.setPosition(Lup);
        svoRHook.setPosition(Rup);

        waitForStart();
        runtime.reset();

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        while (opModeIsActive()) {

            //DRIVING
            mtrBL.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));
            mtrBR.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
            mtrFL.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x));
            mtrFR.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));


            switch (hooksState){
                case HOOKS_UP:
                    if(gamepad1.a){
                        //if a is pressed, put both hooks down
                        svoLHook.setPosition(Ldown);
                        svoRHook.setPosition(Rdown);
                        hooksState = HooksState.HOOKS_DOWN;
                    }
                    break;
                case HOOKS_DOWN:
                    if(svoLHook.getPosition() == Ldown && svoRHook.getPosition() == Rdown){
                        svoLHook.setPosition(Lup);
                        hooksState = HooksState.LEFT_UP;
                    }
                    break;
                case LEFT_UP:
                    if(svoLHook.getPosition() == Lup){
                        svoRHook.setPosition(Rup);
                        hooksState = HooksState.RIGHT_UP;
                    }
                    break;
                case RIGHT_UP:
                    if(svoRHook.getPosition() == Rup){
                        hooksState = HooksState.HOOKS_UP;
                    }
                    break;
                default:
                    hooksState = HooksState.HOOKS_UP;
            }

            if (gamepad1.b && hooksState != HooksState.HOOKS_UP){
                hooksState = HooksState.HOOKS_UP;
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }


    }


}
