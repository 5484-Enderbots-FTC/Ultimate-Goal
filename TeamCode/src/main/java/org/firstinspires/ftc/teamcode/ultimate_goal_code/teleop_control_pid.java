package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.odometry.drive.SampleMecanumDrive;

@TeleOp(name = "teleop PID", group = "teleop")
public class teleop_control_pid extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime svoTimer = new ElapsedTime();
    ElapsedTime btwShotsTimer = new ElapsedTime();
    ElapsedTime toggleTimerS = new ElapsedTime();
    ElapsedTime toggleTimerB = new ElapsedTime();
    ElapsedTime toggleTimerF = new ElapsedTime();
    ElapsedTime toggleTimerIntake = new ElapsedTime();
    ElapsedTime toggleTimerRevIntake = new ElapsedTime();
    ElapsedTime toggleTimerShooter = new ElapsedTime();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    hardwareUltimateGoal robot = new hardwareUltimateGoal();

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
        SERVO_WAIT_1,
        ONE_RING_SHOT,
        SERVO_WAIT_2,
        TWO_RINGS_SHOT,
        SERVO_WAIT_3,
        FINISH
    }

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL,
        SHOOT_LEFT,
        SVO_MOVE_1,
        SHOOT_MID,
        SVO_MOVE_2,
        SHOOT_RIGHT,
        SVO_MOVE_3
    }

    ShootState currentState = ShootState.NOTHING;
    Mode currentMode = Mode.DRIVER_CONTROL;

    double targetShootHeading = Math.toRadians(0);

    Pose2d targetLeft = new Pose2d(0, -10, targetShootHeading);
    Pose2d targetMiddle = new Pose2d(0, -19, targetShootHeading);
    Pose2d targetRight = new Pose2d(0, -26.5, targetShootHeading);

    Pose2d againstWall = new Pose2d(0, 0, 0);

    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.initShooterPID(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /**
         * power shot trajectories uwu
         */

        Trajectory leftShot = drive.trajectoryBuilder(againstWall)
                .lineToLinearHeading(targetLeft)
                .build();
        Trajectory middleShot = drive.trajectoryBuilder(leftShot.end())
                .lineToLinearHeading(targetMiddle)
                .build();
        Trajectory rightShot = drive.trajectoryBuilder(middleShot.end())
                .lineToLinearHeading(targetRight)
                .build();

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

            if (gamepad1.right_bumper && (!backwardsMode) && (toggleTimerB.seconds() > var.toggleWaitTime)) {
                //activate backwards mode
                backwardsMode = true;
                toggleTimerB.reset();
            } else if (gamepad1.right_bumper && (backwardsMode) && (toggleTimerB.seconds() > var.toggleWaitTime)) {
                //deactivate backwards mode
                backwardsMode = false;
                toggleTimerB.reset();
            }

            if (gamepad1.left_bumper && (!slowMode) && (toggleTimerS.seconds() > var.toggleWaitTime)) {
                //activate slow mode
                slowMode = true;
                toggleTimerS.reset();
            } else if (gamepad1.left_bumper && (slowMode) && (toggleTimerS.seconds() > var.toggleWaitTime)) {
                //deactivate slow mode
                slowMode = false;
                toggleTimerS.reset();
            }


            if ((!backwardsMode) && (!slowMode)) {
                //default controls
                robot.updateDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);
            }
            if ((backwardsMode) && (!slowMode)) {
                //backwards mode only
                robot.updateDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            }
            if ((!backwardsMode) && (slowMode)) {
                //slow mode only
                robot.updateDrive(gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.5, true);
            }
            if ((backwardsMode) && (slowMode)) {
                //backwards and slow modes together
                robot.updateDrive(-gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.5, true);
            }

            if (!magIsUp) {
                if (gamepad1.a) {
                    robot.mtrIntake.setPower(1);
                }
            }

            if (gamepad1.b) {
                robot.mtrIntake.setPower(0);
            }
            if (gamepad1.x) {
                robot.mtrIntake.setPower(-1);
            }

            if (gamepad1.dpad_down) {
                robot.svoWobble.setPosition(var.wobbleHold);
            }
            if (gamepad1.dpad_up) {
                robot.svoWobble.setPosition(var.wobbleRelease);
            }
            switch (currentMode) {

                case DRIVER_CONTROL:
                    //literally just drive normally and hope u dont press y lmao
                    if (gamepad1.y) {
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;

                case AUTOMATIC_CONTROL:
                    drive.setPoseEstimate(againstWall);
                    currentMode = Mode.SHOOT_LEFT;
                    break;
                case SHOOT_LEFT:
                    robot.mtrFlywheel.setVelocity(var.psFlywheelVelocity);
                    robot.svoMagLift.setPosition(var.magUp);
                    drive.followTrajectory(leftShot);
                    robot.svoRingPush.setPosition(var.ringPushOut);
                    svoTimer.reset();
                    currentMode = Mode.SVO_MOVE_1;
                    break;
                case SVO_MOVE_1:
                    if (svoTimer.seconds() > var.servoMoveTime) {
                        robot.svoRingPush.setPosition(var.ringPushIn);
                        drive.followTrajectory(middleShot);
                        currentMode = Mode.SHOOT_MID;
                    }
                    break;
                case SHOOT_MID:
                    robot.svoRingPush.setPosition(var.ringPushOut);
                    svoTimer.reset();
                    currentMode = Mode.SVO_MOVE_2;
                    break;
                case SVO_MOVE_2:
                    if (svoTimer.seconds() > var.servoMoveTime) {
                        robot.svoRingPush.setPosition(var.ringPushIn);
                        drive.followTrajectory(rightShot);
                        svoTimer.reset();
                        currentMode = Mode.SHOOT_RIGHT;
                    }
                    break;
                case SHOOT_RIGHT:
                    if(svoTimer.seconds() > (var.servoMoveTime+0.2)) {
                        robot.svoRingPush.setPosition(var.ringPushOut);
                        svoTimer.reset();
                        currentMode = Mode.SVO_MOVE_3;
                    }
                    break;
                case SVO_MOVE_3:
                    if (svoTimer.seconds() > var.servoMoveTime) {
                        robot.svoRingPush.setPosition(var.ringPushIn);
                        robot.svoMagLift.setPosition(var.magDown);
                        robot.mtrFlywheel.setPower(0);
                        robot.svoForkHold.setPosition(var.forkRelease);

                        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
                default:
                    currentMode = Mode.DRIVER_CONTROL;
                    break;
            }
            if (gamepad1.right_trigger > 0.1 && currentMode != Mode.DRIVER_CONTROL) {
                drive.cancelFollowing();
                drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.mtrFlywheel.setPower(0);
                currentMode = Mode.DRIVER_CONTROL;
            }

            /**
             * Gamepad 2 Controls
             */
            if (magIsUp) {
                if (gamepad1.x) {
                    robot.mtrIntake.setPower(-1);
                } else {
                    robot.mtrIntake.setPower(0);
                }
                intakeRunning = false;
            }


            switch (currentState) {
                case NOTHING:
                    if (magIsUp) {
                        if (gamepad2.right_bumper) {
                            currentState = ShootState.ZERO_RINGS_SHOT;
                        }
                    }
                    break;
                case ZERO_RINGS_SHOT:
                    robot.svoRingPush.setPosition(var.ringPushOut);
                    svoTimer.reset();
                    currentState = ShootState.SERVO_WAIT_1;
                    break;

                case SERVO_WAIT_1:
                    if (svoTimer.seconds() > var.servoMoveTime) {
                        robot.svoRingPush.setPosition(var.ringPushIn);
                        btwShotsTimer.reset();
                        currentState = ShootState.ONE_RING_SHOT;
                    }
                    break;
                case ONE_RING_SHOT:
                    if (btwShotsTimer.seconds() > var.timeBetweenShots) {
                        robot.svoRingPush.setPosition(var.ringPushOut);
                        svoTimer.reset();
                        currentState = ShootState.SERVO_WAIT_2;
                    }
                    break;
                case SERVO_WAIT_2:
                    if (svoTimer.seconds() > var.servoMoveTime) {
                        robot.svoRingPush.setPosition(var.ringPushIn);
                        btwShotsTimer.reset();
                        currentState = ShootState.TWO_RINGS_SHOT;
                    }
                    break;
                case TWO_RINGS_SHOT:
                    if (btwShotsTimer.seconds() > var.timeBetweenShots) {
                        robot.svoRingPush.setPosition(var.ringPushOut);
                        svoTimer.reset();
                        currentState = ShootState.SERVO_WAIT_3;
                    }
                    break;
                case SERVO_WAIT_3:
                    if (svoTimer.seconds() > var.servoMoveTime) {
                        robot.svoRingPush.setPosition(var.ringPushIn);
                        btwShotsTimer.reset();
                        currentState = ShootState.FINISH;
                    }
                    break;
                case FINISH:
                    if (btwShotsTimer.seconds() > var.servoMoveTime) {
                        robot.mtrIntake.setPower(1);
                        intakeRunning = true;
                        robot.svoMagLift.setPosition(var.magDown);
                        robot.mtrFlywheel.setPower(0);
                        flywheelRunning = false;
                        magIsUp = false;
                        currentState = ShootState.NOTHING;
                    }
                    break;
            }

            if (gamepad2.y && currentState != ShootState.NOTHING) {
                robot.svoRingPush.setPosition(var.ringPushIn);
                currentState = ShootState.NOTHING;
            }

            if (gamepad2.a && !flywheelRunning && toggleTimerShooter.seconds() > var.toggleWaitTime) {
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
            if (gamepad2.a && flywheelRunning && toggleTimerShooter.seconds() > var.toggleWaitTime) {
                robot.mtrFlywheel.setPower(0);
                targetVelo = 0;
                flywheelRunning = false;
                toggleTimerShooter.reset();
            }

            if (gamepad2.right_trigger > 0.1) {
                robot.svoRingPush.setPosition(var.ringJamnt);
                waitFor(0.5);
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

            if (gamepad2.left_bumper) {
                robot.svoForkHold.setPosition(var.forkHold);
            }
            if (gamepad2.left_trigger > 0.1) {
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
            robot.updateDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);

            robot.mtrWobble.setPower(gamepad2.right_stick_y);


        }
    }

}


