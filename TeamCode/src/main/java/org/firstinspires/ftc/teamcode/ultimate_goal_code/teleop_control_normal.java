package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name="teleop normal",group="1-teleop")
public class teleop_control_normal extends LinearOpMode{
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime toggleTimerS = new ElapsedTime();
    ElapsedTime toggleTimerB = new ElapsedTime();
    ElapsedTime toggleTimerF = new ElapsedTime();
    ElapsedTime toggleTimerR = new ElapsedTime();

    hardwareUltimateGoal robot   = new hardwareUltimateGoal();
    BNO055IMU imu;

    /***

     ~ Constants ~

     */

    double flywheelPower = 0.63;
    double lessFlywheelPower = 0.55;
    double magDown = 0.85;
    double magUp = 0.58;
    double ringPushOut = 0.6;
    double ringPushIn = 0.75;
    double ringJamnt = 0.2;
    double wobbleRelease = 0.37;
    double wobbleHold = 0.2;
    double forkHold = 0.8;
    double forkRelease = 0.7;
    double toggleWaitTime = 0.5;

    boolean magIsUp = false;
    boolean backwardsMode = false;
    boolean slowMode = false;
    boolean forkHeld = true;
    boolean ringSwiped = false;

    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();
        runtime.reset();
        timer.reset();
        toggleTimerB.reset();
        toggleTimerS.reset();
        toggleTimerF.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            /**
             * Gamepad 1 Controls
             */

            if(gamepad1.right_bumper && (backwardsMode == false) && (toggleTimerB.seconds() > toggleWaitTime)){
                //activate backwards mode
                backwardsMode = true;
                toggleTimerB.reset();
            }
            else if(gamepad1.right_bumper && (backwardsMode == true) && (toggleTimerB.seconds() > toggleWaitTime)){
                //deactivate backwards mode
                backwardsMode = false;
                toggleTimerB.reset();
            }

            if(gamepad1.left_bumper && (slowMode == false) && (toggleTimerS.seconds() > toggleWaitTime)){
                //activate slow mode
                slowMode = true;
                toggleTimerS.reset();
            }
            else if(gamepad1.left_bumper && (slowMode == true) && (toggleTimerS.seconds() > toggleWaitTime)){
                //deactivate slow mode
                slowMode = false;
                toggleTimerS.reset();
            }

            if((backwardsMode == false) && (slowMode == false)) {
                //default controls
                robot.mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
                robot.mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
                robot.mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
                robot.mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            }
            if((backwardsMode == true) && (slowMode == false)){
                //backwards mode only
                robot.mtrBL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
                robot.mtrBR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
                robot.mtrFL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
                robot.mtrFR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            }
            if((backwardsMode == false) && (slowMode == true)){
                //slow mode only
                robot.mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
                robot.mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
                robot.mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
                robot.mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
            }
            if((backwardsMode == true) && (slowMode == true)){
                //backwards and slow modes together
                robot.mtrBL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
                robot.mtrBR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
                robot.mtrFL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
                robot.mtrFR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
            }

            if (gamepad1.a){
                robot.mtrIntake.setPower(1);
            }
            if (gamepad1.b){
                robot.mtrIntake.setPower(0);
            }
            if (gamepad1.x){
                robot.mtrIntake.setPower(-1);
            }


            /**
             * Gamepad 2 Controls
             */

            robot.mtrWobble.setPower(gamepad2.right_stick_y);

            if (magIsUp){
                robot.mtrIntake.setPower(0);
                if(gamepad2.right_bumper){
                    robot.svoRingPush.setPosition(ringPushOut);
                    waitFor(0.5);
                    robot.svoRingPush.setPosition(ringPushIn);
                }
            }
            if(gamepad2.right_trigger > 0.1){
                robot.svoRingPush.setPosition(ringJamnt);
                waitFor(0.5);
                robot.svoRingPush.setPosition(ringPushIn);
            }

            if (gamepad2.y){
                robot.mtrFlywheel.setPower(lessFlywheelPower);
            }

            /*
            if (gamepad2.y && (ringSwiped == false)){
                svoRingPush.setPosition(ringJamnt);
                ringSwiped = true;
            }
            if (gamepad2.y && (ringSwiped == true)){
                svoRingPush.setPosition(ringPushIn);
                ringSwiped = false;
            }
             */

            if(gamepad2.dpad_up){
                robot.svoMagLift.setPosition(magUp);
                robot.mtrFlywheel.setPower(flywheelPower);
                magIsUp = true;
            }
            if(gamepad2.dpad_down){
                robot.mtrIntake.setPower(1);
                robot.svoMagLift.setPosition(magDown);
                robot.mtrFlywheel.setPower(0);
                magIsUp = false;
            }

            if(gamepad2.a){
                robot.mtrFlywheel.setPower(flywheelPower);
            }
            if(gamepad2.b){
                robot.mtrFlywheel.setPower(0);
            }

            if(gamepad2.left_bumper && (forkHeld == false) && (toggleTimerF.seconds() > toggleWaitTime)){
                robot.svoForkHold.setPosition(forkHold);
                forkHeld = true;
                toggleTimerF.reset();
            }
            else if(gamepad2.left_bumper && (forkHeld == true) && (toggleTimerF.seconds() > toggleWaitTime)){
                robot.svoForkHold.setPosition(forkRelease);
                forkHeld = false;
                toggleTimerF.reset();
            }

            /**
             * Telemetry
             */

            telemetry.addData("Status", " Run Time: " + runtime.toString());
            telemetry.addData("Timer Status", " Time: " + timer.toString());
            telemetry.addData("backwards timer sec:", " " + toggleTimerB.seconds());
            telemetry.addData("slow timer sec:", " " + toggleTimerS.seconds());
            telemetry.update();
        }


    }
    private void waitFor(double waittime) {
        timer.reset();
        while (timer.seconds() < waittime) {
            //pls keep driving lol
            robot.mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            robot.mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            robot.mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            robot.mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
        }
    }

}


