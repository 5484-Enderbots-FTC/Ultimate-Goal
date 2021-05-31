package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "teleop control swap", group = "teleop")
@Disabled
public class teleop_control_swap extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime toggleTimerS = new ElapsedTime();
    ElapsedTime toggleTimerB = new ElapsedTime();
    ElapsedTime toggleTimerF = new ElapsedTime();
    ElapsedTime toggleTimerIntake = new ElapsedTime();
    ElapsedTime toggleTimerRevIntake = new ElapsedTime();
    ElapsedTime toggleTimerShooter = new ElapsedTime();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    hardwareUltimateGoal robot = new hardwareUltimateGoal();

    ShootState currentState;

    /***

     ~ Constants ~

     */

    double targetVelo = 0;

    boolean magIsUp = false;
    boolean backwardsMode = false;
    boolean slowMode = false;
    boolean intakeRunning = false;
    boolean flywheelRunning = false;

    private enum ShootState {
        NOTHING,
        ZERO_RINGS_SHOT,
        ONE_RING_SHOT,
        TWO_RINGS_SHOT,
        FINISH
    }


    public void runOpMode() {
        robot.init(hardwareMap);
        robot.initShooterPID(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        currentState = ShootState.NOTHING;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //BEGIN
        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();
        timer.reset();
        toggleTimerB.reset();
        toggleTimerS.reset();
        toggleTimerF.reset();
        toggleTimerIntake.reset();
        toggleTimerShooter.reset();
        toggleTimerRevIntake.reset();

        while (!isStopRequested() && opModeIsActive()) {
            /**
             * Telemetry
             */

            telemetry.addData("topLimit: ", robot.topLimit.getState());
            telemetry.update();

            /**
             * Gamepad 1 Controls
             */

            if (gamepad1.right_bumper && (backwardsMode == false) && (toggleTimerB.seconds() > var.toggleWaitTime)) {
                //activate backwards mode
                backwardsMode = true;
                toggleTimerB.reset();
            } else if (gamepad1.right_bumper && (backwardsMode == true) && (toggleTimerB.seconds() > var.toggleWaitTime)) {
                //deactivate backwards mode
                backwardsMode = false;
                toggleTimerB.reset();
            }

            if (gamepad1.left_bumper && (slowMode == false) && (toggleTimerS.seconds() > var.toggleWaitTime)) {
                //activate slow mode
                slowMode = true;
                toggleTimerS.reset();
            } else if (gamepad1.left_bumper && (slowMode == true) && (toggleTimerS.seconds() > var.toggleWaitTime)) {
                //deactivate slow mode
                slowMode = false;
                toggleTimerS.reset();
            }


            if ((backwardsMode == false) && (slowMode == false)) {
                //default controls
                robot.updateDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
                /*
                robot.mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
                robot.mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
                robot.mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
                robot.mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));

                 */
            }
            if ((backwardsMode == true) && (slowMode == false)) {
                //backwards mode only
                robot.updateDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x);
                /*
                robot.mtrBL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
                robot.mtrBR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
                robot.mtrFL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
                robot.mtrFR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));

                 */
            }
            if ((backwardsMode == false) && (slowMode == true)) {
                //slow mode only
                robot.updateDrive(gamepad1.left_stick_y * 0.5, gamepad1.right_stick_x * 0.5, gamepad1.left_stick_x * 0.5);
                /*
                robot.mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 0.5);
                robot.mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * 0.5);
                robot.mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * 0.5);
                robot.mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 0.5);

                 */
            }
            if ((backwardsMode == true) && (slowMode == true)) {
                //backwards and slow modes together
                robot.updateDrive(-gamepad1.left_stick_y * 0.5, gamepad1.right_stick_x, -gamepad1.left_stick_x * 0.5);

                /*
                robot.mtrBL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * 0.5);
                robot.mtrBR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 0.5);
                robot.mtrFL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 0.5);
                robot.mtrFR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * 0.5);

                 */
            }

            if(!magIsUp) {
                if (gamepad1.a) {
                    robot.mtrIntake.setPower(1);
                }
            }

            if(gamepad1.b){
                robot.mtrIntake.setPower(0);
            }
            if(gamepad1.x){
                robot.mtrIntake.setPower(-1);
            }

            if(gamepad1.y){
                robot.svoWobble.setPosition(var.wobbleRelease);
            }

            /**
             * Gamepad 2 Controls
             */
            if (magIsUp) {
                if(gamepad1.x) {
                    robot.mtrIntake.setPower(-1);
                }
                else{
                    robot.mtrIntake.setPower(0);
                }
                intakeRunning = false;
            }

            switch(currentState){
                case NOTHING:
                    if (magIsUp) {
                        if (gamepad2.right_bumper) {
                            currentState = ShootState.ZERO_RINGS_SHOT;
                        }
                    }
                case ZERO_RINGS_SHOT:
                    robot.svoRingPush.setPosition(var.ringPushOut);
                    waitFor(var.servoMoveTime);
                    robot.svoRingPush.setPosition(var.ringPushIn);
                    waitFor(var.timeBetweenShots);
                    currentState = ShootState.ONE_RING_SHOT;
                case ONE_RING_SHOT:
                    robot.svoRingPush.setPosition(var.ringPushOut);
                    waitFor(var.servoMoveTime);
                    robot.svoRingPush.setPosition(var.ringPushIn);
                    waitFor(var.timeBetweenShots);
                    currentState = ShootState.TWO_RINGS_SHOT;
                case TWO_RINGS_SHOT:
                    robot.svoRingPush.setPosition(var.ringPushOut);
                    waitFor(var.servoMoveTime);
                    robot.svoRingPush.setPosition(var.ringPushIn);
                    waitFor(var.timeBetweenShots);
                    currentState = ShootState.FINISH;
                case FINISH:
                    robot.mtrIntake.setPower(1);
                    intakeRunning = true;
                    robot.svoMagLift.setPosition(var.magDown);
                    robot.mtrFlywheel.setPower(0);
                    flywheelRunning = false;
                    magIsUp = false;
                    currentState = ShootState.NOTHING;
            }

            if(gamepad2.y && currentState != ShootState.NOTHING){
                currentState = ShootState.NOTHING;
            }

            if (gamepad2.a && flywheelRunning == false && toggleTimerShooter.seconds() > var.toggleWaitTime) {
                robot.mtrFlywheel.setVelocity(var.normalFlywheelVelocity);
                targetVelo = var.normalFlywheelVelocity;
                flywheelRunning = true;
                toggleTimerShooter.reset();
            }
            if (gamepad2.b) {
                robot.mtrFlywheel.setVelocity(var.psFlywheelVelocity);
                targetVelo = var.psFlywheelVelocity;
                flywheelRunning = true;
            }
            if (gamepad2.a && flywheelRunning == true && toggleTimerShooter.seconds() > var.toggleWaitTime) {
                robot.mtrFlywheel.setPower(0);
                targetVelo = 0;
                flywheelRunning = false;
                toggleTimerShooter.reset();
            }

            if (gamepad2.right_trigger > 0.1) {
                robot.svoRingPush.setPosition(var.ringJamnt);
                waitFor(0.3);
                robot.svoRingPush.setPosition(var.ringPushIn);
            }

            if (gamepad2.dpad_up) {
                robot.svoMagLift.setPosition(var.magUp);
                robot.mtrFlywheel.setVelocity(var.normalFlywheelVelocity);
                targetVelo = var.normalFlywheelVelocity;
                flywheelRunning = true;
                magIsUp = true;
            }

            if (gamepad2.dpad_down) {
                robot.mtrIntake.setPower(1);
                intakeRunning = true;
                robot.svoMagLift.setPosition(var.magDown);
                robot.mtrFlywheel.setPower(0);
                flywheelRunning = false;
                magIsUp = false;
            }

            robot.mtrWobble.setPower(gamepad2.right_stick_y);

            if(gamepad2.left_bumper){
                robot.svoForkHold.setPosition(var.forkHold);
            }
            if(gamepad2.left_trigger > 0.1){
                robot.svoForkHold.setPosition(var.forkRelease);
                waitFor(1);
                robot.svoForkHold.setPosition(var.forkRaise);
            }
/*
            if(robot.topLimit.getState() == true){
                //limit switch isn't pressed

            }else{
                //it do be pressed
                robot.mtrWobble.setPower(0);
            }

 */
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

            robot.mtrWobble.setPower(gamepad2.right_stick_y);

        }
    }

}


