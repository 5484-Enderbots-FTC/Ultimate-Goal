package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="teleop solo",group="1-teleop")
@Disabled
public class teleop_one_controller extends LinearOpMode{
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime toggleTimerS = new ElapsedTime();
    ElapsedTime toggleTimerB = new ElapsedTime();
    ElapsedTime toggleTimerF = new ElapsedTime();

    ElapsedTime toggleTimerState = new ElapsedTime();

    gamepadState gamepadState;

    hardwareUltimateGoal robot = new hardwareUltimateGoal();

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

    public enum gamepadState{
        SHOOT_MODE,
        INTAKE_MODE
    }

    public void runOpMode() {
        robot.init(hardwareMap);

        gamepadState = gamepadState.INTAKE_MODE;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();
        timer.reset();
        runtime.reset();
        timer.reset();
        toggleTimerB.reset();
        toggleTimerS.reset();
        toggleTimerF.reset();
        toggleTimerState.reset();

        while (opModeIsActive()) {

            if ((backwardsMode == false) && (slowMode == false)) {
                //default controls
                robot.mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
                robot.mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
                robot.mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
                robot.mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            }
            if ((backwardsMode == true) && (slowMode == false)) {
                //backwards mode only
                robot.mtrBL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
                robot.mtrBR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
                robot.mtrFL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
                robot.mtrFR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            }
            if ((backwardsMode == false) && (slowMode == true)) {
                //slow mode only
                robot.mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 0.5);
                robot.mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * 0.5);
                robot.mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * 0.5);
                robot.mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 0.5);
            }
            if ((backwardsMode == true) && (slowMode == true)) {
                //backwards and slow modes together
                robot.mtrBL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * 0.5);
                robot.mtrBR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 0.5);
                robot.mtrFL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 0.5);
                robot.mtrFR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * 0.5);
            }

            if (gamepad1.dpad_up) {
                robot.svoMagLift.setPosition(magUp);
                robot.mtrFlywheel.setPower(flywheelPower);
                magIsUp = true;
            }
            if (gamepad1.dpad_down) {
                robot.mtrIntake.setPower(1);
                robot.svoMagLift.setPosition(magDown);
                robot.mtrFlywheel.setPower(0);
                magIsUp = false;
            }

            switch(gamepadState){

                case INTAKE_MODE:
                    if(gamepad1.y && toggleTimerState.seconds()<0.5){
                        gamepadState = gamepadState.SHOOT_MODE;
                    }
                    
                    /***
                     * Slow & Backward mode
                     */
                    if (gamepad1.right_bumper && (backwardsMode == false) && (toggleTimerB.seconds() > toggleWaitTime)) {
                        //activate backwards mode
                        backwardsMode = true;
                        toggleTimerB.reset();
                    } else if (gamepad1.right_bumper && (backwardsMode == true) && (toggleTimerB.seconds() > toggleWaitTime)) {
                        //deactivate backwards mode
                        backwardsMode = false;
                        toggleTimerB.reset();
                    }

                    if (gamepad1.left_bumper && (slowMode == false) && (toggleTimerS.seconds() > toggleWaitTime)) {
                        //activate slow mode
                        slowMode = true;
                        toggleTimerS.reset();
                    } else if (gamepad1.left_bumper && (slowMode == true) && (toggleTimerS.seconds() > toggleWaitTime)) {
                        //deactivate slow mode
                        slowMode = false;
                        toggleTimerS.reset();
                    }

                    /***
                     * Intake control
                     */
                    if (gamepad1.a) {
                        robot.mtrIntake.setPower(1);
                    }
                    if (gamepad1.b) {
                        robot.mtrIntake.setPower(0);
                    }
                    if (gamepad1.x) {
                        robot.mtrIntake.setPower(-1);
                    }


                case SHOOT_MODE:
                    if(gamepad1.y && toggleTimerState.seconds() > toggleWaitTime){
                        gamepadState = gamepadState.INTAKE_MODE;
                    }

                    /***
                     * Flywheel speed control
                     */
                    
                    if (gamepad1.y) {
                        robot.mtrFlywheel.setPower(lessFlywheelPower);
                    }
                    if (gamepad1.a) {
                        robot.mtrFlywheel.setPower(flywheelPower);
                    }
                    if (gamepad1.b) {
                        robot.mtrFlywheel.setPower(0);
                    }
                    
                    /***
                     * Ring push / unjam
                     */
                    
                    if (magIsUp) {
                        robot.mtrIntake.setPower(0);
                        if (gamepad1.right_bumper) {
                            robot.svoRingPush.setPosition(ringPushOut);
                            waitFor(0.5);
                            robot.svoRingPush.setPosition(ringPushIn);
                        }
                    }
                    if (gamepad1.right_trigger > 0.1) {
                        robot.svoRingPush.setPosition(ringJamnt);
                        waitFor(0.5);
                        robot.svoRingPush.setPosition(ringPushIn);
                    }

                default:
                    gamepadState = gamepadState.INTAKE_MODE;

            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }


    }
    private void waitFor(double waittime) {
        runtime.reset();
        while (runtime.seconds() < waittime) {
            //don't do anything pls
            robot.mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            robot.mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            robot.mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            robot.mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
        }
    }

}
