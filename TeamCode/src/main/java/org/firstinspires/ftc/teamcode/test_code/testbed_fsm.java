package org.firstinspires.ftc.teamcode.test_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="testbed_fsm",group="testing")
@Disabled
public class testbed_fsm extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();

    DcMotor mtrBL , mtrBR , mtrFL , mtrFR;
    Servo svoLHook, svoRHook;

    final double Ldown = 0.75;
    final double Lup = 0.5;
    final double Rdown = 0.25;
    final double Rup = 0.5;

    boolean leftUpOnly = false;
    boolean bothUp = true;
    boolean bothDown = false;

    private enum HooksState {
        HOOKS_UP,
        HOOKS_DOWN,
        LEFT_UP
    }

    HooksState hooksState = HooksState.HOOKS_UP;

    public void runOpMode() {
        telemetry.addData("Status:", "Begin init");
        telemetry.update();

        //motor 3
        mtrBL = hardwareMap.get(DcMotor.class, "mtrBL");
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setDirection(DcMotor.Direction.FORWARD);
        //motor 1
        mtrBR = hardwareMap.get(DcMotor.class, "mtrBR");
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setDirection(DcMotor.Direction.REVERSE);
        //motor 0
        mtrFL = hardwareMap.get(DcMotor.class, "mtrFL");
        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFL.setDirection(DcMotor.Direction.FORWARD);
        //motor 2
        mtrFR = hardwareMap.get(DcMotor.class, "mtrFR");
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setDirection(DcMotor.Direction.REVERSE);

        svoLHook = hardwareMap.get(Servo.class, "svoLHook");
        svoLHook.setDirection(Servo.Direction.FORWARD);

        svoRHook = hardwareMap.get(Servo.class, "svoRHook");
        svoRHook.setDirection(Servo.Direction.FORWARD);

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

            if(gamepad1.right_bumper){
                svoRHook.setPosition(Rdown);
                svoLHook.setPosition(Ldown);
            }
            if(gamepad1.left_bumper){
                svoRHook.setPosition(Rup);
                svoLHook.setPosition(Lup);
            }

            switch (hooksState){
                case HOOKS_UP:
                    if(gamepad1.a && bothUp == true){
                        //if a is pressed, put both hooks down
                        svoLHook.setPosition(Ldown);
                        svoRHook.setPosition(Rdown);
                        hooksState = HooksState.HOOKS_DOWN;
                        bothUp = false;
                        bothDown = true;
                        leftUpOnly = false;
                        runtime.reset();
                    }
                    break;
                case HOOKS_DOWN:
                    if(getRuntime() > 1 && bothDown == true){
                        svoLHook.setPosition(Lup);
                        hooksState = HooksState.LEFT_UP;
                        bothUp = false;
                        bothDown = false;
                        leftUpOnly = true;
                        runtime.reset();
                    }
                    break;
                case LEFT_UP:
                    if(getRuntime() > 1 && leftUpOnly == true){
                        svoRHook.setPosition(Rup);
                        hooksState = HooksState.HOOKS_UP;
                        bothUp = true;
                        bothDown = false;
                        leftUpOnly = false;
                    }
                    break;

                    /*
                default:
                    hooksState = HooksState.HOOKS_UP;
                     */

            }

            if (gamepad1.b && hooksState != HooksState.HOOKS_UP){
                hooksState = HooksState.HOOKS_UP;
                bothUp = true;
                bothDown = false;
                leftUpOnly = false;
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }


    }


}
