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
    Servo svoWobble, svoMagLift, svoRingPush, svoForkHold;

    /***

     ~ Constants ~

     */
    double magDown = 0.85;
    double magUp = 0.58;
    double ringPushOut = 0.6;
    double ringPushIn = 0.75;
    double ringJamnt = 0.2;
    double wobbleRelease = 0.37;
    double wobbleHold = 0.2;
    double forkHold = 0.8;
    double forkRelease = 0.7;

    boolean magIsUp = false;
    boolean backwardsMode = false;
    boolean slowMode = false;
    boolean forkHeld = true;

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
        mtrFlywheel.setDirection(DcMotorEx.Direction.REVERSE);

        svoMagLift = hardwareMap.get(Servo.class,"svoMagLift");
        svoMagLift.setDirection(Servo.Direction.FORWARD);

        svoRingPush = hardwareMap.get(Servo.class,"svoRingPush");
        svoRingPush.setDirection(Servo.Direction.REVERSE);

        svoWobble = hardwareMap.get(Servo.class,"svoWobble");
        svoWobble.setDirection(Servo.Direction.FORWARD);

        svoForkHold = hardwareMap.get(Servo.class,"svoForkHold");
        svoForkHold.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        svoRingPush.setPosition(ringPushIn);
        svoMagLift.setPosition(magDown);
        svoWobble.setPosition(wobbleHold);
        svoForkHold.setPosition(forkHold);

        waitForStart();
        runtime.reset();
        timer.reset();

        while (opModeIsActive()) {

            /**
             * Gamepad 1 Controls
             */

            if(gamepad1.right_bumper && (backwardsMode = false)){
                backwardsMode = true;
            }
            else if(gamepad1.right_bumper && (backwardsMode = true)){
                backwardsMode = false;
            }

            if(gamepad1.left_bumper && (slowMode = false)){
                slowMode = true;
            }
            else if(gamepad1.left_bumper && (slowMode = true)){
                slowMode = false;
            }

            if((backwardsMode = false) && (slowMode = false)) {
                mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
                mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
                mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
                mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            }
            if((backwardsMode = true) && (slowMode = false)){
                mtrBL.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
                mtrBR.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
                mtrFL.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
                mtrFR.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            }
            if((backwardsMode = false) && (slowMode = true)){
                mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
                mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
                mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
                mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
            }
            if((backwardsMode = true) && (slowMode = true)){
                mtrBL.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
                mtrBR.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
                mtrFL.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
                mtrFR.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
            }

            if (gamepad1.a){
                mtrIntake.setPower(1);
            }
            if (gamepad1.b){
                mtrIntake.setPower(0);
            }
            if (gamepad1.x){
                mtrIntake.setPower(-1);
            }

            /**
             * Gamepad 2 Controls
             */

            mtrWobble.setPower(gamepad2.right_stick_y);

            if (magIsUp){
                mtrIntake.setPower(0);
                if(gamepad2.right_bumper){
                    svoRingPush.setPosition(ringPushOut);
                    waitFor(0.5);
                    svoRingPush.setPosition(ringPushIn);
                }
            }
            if (gamepad2.y){
                svoRingPush.setPosition(ringJamnt);
            }

            if(gamepad2.dpad_up){
                svoMagLift.setPosition(magUp);
                mtrFlywheel.setPower(0.8);
                magIsUp = true;
            }
            if(gamepad2.dpad_down){
                mtrIntake.setPower(1);
                svoMagLift.setPosition(magDown);
                mtrFlywheel.setPower(0);
                magIsUp = false;
            }

            if(gamepad2.a){
                mtrFlywheel.setPower(0.8);
            }
            if(gamepad2.b){
                mtrFlywheel.setPower(0);
            }

            if(gamepad2.left_bumper && (forkHeld = false)){
                forkHeld = true;
            }
            else if(gamepad2.left_bumper && (forkHeld = true)){
                forkHeld = false;
            }

            if(forkHeld = true){
                svoForkHold.setPosition(forkHold);
            }
            if(forkHeld = false){
                svoForkHold.setPosition(forkRelease);
            }

            /**
             * Telemetry
             */

            telemetry.addData("Status", " Run Time: " + runtime.toString());
            telemetry.addData("Timer Status", " Time: " + timer.toString());
            telemetry.addData("backwardsMode status:", " " + backwardsMode);
            telemetry.addData("slowMode status:", " " + slowMode);
            telemetry.addData("forkHeld status:", " " + forkHeld);
            telemetry.update();
        }


    }
    private void waitFor(double waittime) {
        timer.reset();
        while (timer.seconds() < waittime) {
            //pls keep driving lol
            mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
        }
    }

}
