package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="teleop normal",group="teleop")
public class teleop_control_normal extends LinearOpMode{
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    DcMotorEx mtrBL , mtrBR , mtrFL , mtrFR , mtrIntake, mtrWobble, mtrFlywheel;
    Servo svoMagLift, svoRingPush;

    rndmState currentState;

    /***

     ~ Constants ~

     */
    double magDown = 0.85;
    double magUp = 0.58;
    double ringPushOut = 0.6;
    double ringPushIn = 0.75;
    boolean magIsUp = false;


    public enum rndmState{
        INTAKE_START,
        WOBBLE_HOLD,
        WOBBLE_RELEASE
    }

    public void runOpMode() {

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
        mtrIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        mtrIntake.setDirection(DcMotorEx.Direction.REVERSE);

        mtrWobble = hardwareMap.get(DcMotorEx.class, "mtrWobble");
        mtrWobble.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrWobble.setDirection(DcMotorEx.Direction.FORWARD);

        mtrFlywheel = hardwareMap.get(DcMotorEx.class, "mtrFlywheel");
        mtrFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        mtrFlywheel.setDirection(DcMotorEx.Direction.FORWARD);

        svoMagLift = hardwareMap.get(Servo.class,"svoMagLift");
        svoMagLift.setDirection(Servo.Direction.FORWARD);

        svoRingPush = hardwareMap.get(Servo.class,"svoRingPush");
        svoRingPush.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //svoWobble.setPosition(wobbleGrab);
        svoRingPush.setPosition(ringPushIn);
        svoMagLift.setPosition(magDown);
        magIsUp = false;

        currentState = rndmState.INTAKE_START;

        waitForStart();
        runtime.reset();
        timer.reset();

        while (opModeIsActive()) {

            mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));

            mtrWobble.setPower(gamepad2.right_stick_y);

            if (gamepad1.a){
                mtrIntake.setPower(1);
            }
            if (gamepad1.b){
                mtrIntake.setPower(0);
            }
            if (gamepad1.x){
                mtrIntake.setPower(-1);
            }

            if (magIsUp){
                if(gamepad2.a){
                    svoRingPush.setPosition(ringPushOut);
                    waitFor(0.5);
                    svoRingPush.setPosition(ringPushIn);
                }
            }
            if(gamepad2.dpad_up){
                svoMagLift.setPosition(magUp);
                magIsUp = true;
            }
            if(gamepad2.dpad_down){
                svoMagLift.setPosition(magDown);
                magIsUp = false;
            }

            if(gamepad2.x){
                mtrFlywheel.setPower(1);
            }
            if(gamepad2.b){
                mtrFlywheel.setPower(0);
            }

            telemetry.addData("Status", " Run Time: " + runtime.toString());
            telemetry.addData("Timer Status", " Time: " + timer.toString());
            telemetry.update();
        }


    }
    private void waitFor(double waittime) {
        runtime.reset();
        while (runtime.seconds() < waittime) {
            //pls keep driving lol
            mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
        }
    }

}
