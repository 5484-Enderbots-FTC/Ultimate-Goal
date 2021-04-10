package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="teleop solo",group="1-teleop")
@Disabled
public class teleop_one_controller extends LinearOpMode{
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    DcMotorEx mtrBL , mtrBR , mtrFL , mtrFR , mtrIntake, mtrWobble;
    Servo svoMagLift, svoRingPush;

    rndmState currentState;
    wobbleDriveState wobbleState;

    /***

     ~ Constants ~

     */
    double magDown = 0.85;
    double magUp = 0.58;
    double ringPushOut = 0.6;
    double ringPushIn = 0.75;
    boolean magIsUp = false;
    boolean toggleSwitch = false;

    public enum wobbleDriveState{
        EVERTHING_ELSE,
        WOBBLE_START
    }

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
        mtrIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrIntake.setDirection(DcMotorEx.Direction.REVERSE);

        mtrWobble = hardwareMap.get(DcMotorEx.class, "mtrWobble");
        mtrWobble.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrWobble.setDirection(DcMotorEx.Direction.FORWARD);
/*
        svoWobble = hardwareMap.get(Servo.class,"svoWobble");
        svoWobble.setDirection(Servo.Direction.FORWARD);

 */

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

        wobbleState = wobbleDriveState.WOBBLE_START;

        waitForStart();
        runtime.reset();
        timer.reset();

        while (opModeIsActive()) {

            mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x));
            mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x));
            mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x));
            mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x));

            mtrWobble.setPower(gamepad1.right_stick_y);

            if (gamepad1.a){
                mtrIntake.setPower(1);
            }
            if (gamepad1.b){
                mtrIntake.setPower(0);
            }
            if (gamepad1.x){
                mtrIntake.setPower(-1);
            }
            if(gamepad1.y){

            }

            if (magIsUp){
                if(gamepad1.right_bumper){
                    svoRingPush.setPosition(ringPushOut);
                    waitFor(0.5);
                    svoRingPush.setPosition(ringPushIn);
                }
            }
            if(gamepad1.dpad_up){
                svoMagLift.setPosition(magUp);
                magIsUp = true;
            }
            if(gamepad1.dpad_down){
                svoMagLift.setPosition(magDown);
                magIsUp = false;
            }


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            telemetry.addData("Timer Status", "Time: " + timer.toString());
            telemetry.addData("Mag Status", "MagIsUp: " + magIsUp);
            telemetry.update();
        }


    }
    private void waitFor(double waittime) {
        runtime.reset();
        while (runtime.seconds() < waittime) {
            //don't do anything pls
            mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
        }
    }

}
