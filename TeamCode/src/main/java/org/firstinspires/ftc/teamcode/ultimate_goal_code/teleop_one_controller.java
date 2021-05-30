package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "teleop solo", group = "teleop")
public class teleop_one_controller extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime toggleTimerS = new ElapsedTime();
    ElapsedTime toggleTimerB = new ElapsedTime();
    ElapsedTime toggleTimerF = new ElapsedTime();
    ElapsedTime toggleTimerIntake = new ElapsedTime();
    ElapsedTime toggleTimerShooter = new ElapsedTime();

    ElapsedTime toggleTimerState = new ElapsedTime();

    gamepadState gamepadState;
    ShootState currentState;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    hardwareUltimateGoal robot = new hardwareUltimateGoal();


    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(100, 0, 30, 18);
    /***

     ~ Constants ~

     */
    double timeBetweenShots = 0.7;
    double servoMoveTime = 0.33;

    double normalFlywheelVelocity = 1350;
    double psFlywheelVelocity = 1200;
    double targetVelo = 0;
    double magDown = 0.85;
    double magUp = 0.58;
    double ringPushOut = 0.6;
    double ringPushIn = 0.75;
    double ringJamnt = 0.2;
    double toggleWaitTime = 0.5;

    boolean magIsUp = false;
    boolean backwardsMode = false;
    boolean slowMode = false;
    boolean intakeRunning = false;
    boolean flywheelRunning = false;

    private enum gamepadState {
        SHOOT_MODE,
        INTAKE_MODE
    }
    private enum ShootState {
        NOTHING,
        ZERO_RINGS_SHOT,
        ONE_RING_SHOT,
        TWO_RINGS_SHOT
    }

    public void runOpMode() {
        robot.init(hardwareMap);

        //Velocity PID
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //PID motor config
        robot.mtrFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorConfigurationType motorConfigurationType = robot.mtrFlywheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        robot.mtrFlywheel.setMotorType(motorConfigurationType);

        setPIDFCoefficients(robot.mtrFlywheel, MOTOR_VELO_PID);


        //TuningController tuningController = new TuningController();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        currentState = ShootState.NOTHING;
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
                robot.mtrFlywheel.setVelocity(normalFlywheelVelocity);
                targetVelo = normalFlywheelVelocity;
                magIsUp = true;
            }
            if (gamepad1.dpad_down) {
                robot.mtrIntake.setPower(1);
                robot.svoMagLift.setPosition(magDown);
                robot.mtrFlywheel.setPower(0);
                magIsUp = false;
            }

            if (gamepad1.x && gamepadState != gamepadState.INTAKE_MODE && toggleTimerState.seconds() > toggleWaitTime) {
                gamepadState = gamepadState.INTAKE_MODE;
            } else if (gamepad1.x && gamepadState != gamepadState.SHOOT_MODE && toggleTimerState.seconds() > toggleWaitTime) {
                gamepadState = gamepadState.SHOOT_MODE;
            }

            switch (gamepadState) {
                case INTAKE_MODE:

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
                    if(!magIsUp) {
                        if (gamepad1.a && (intakeRunning == false) && toggleTimerIntake.seconds() > toggleWaitTime) {
                            robot.mtrIntake.setPower(1);
                            intakeRunning = true;
                            toggleTimerIntake.reset();
                        }
                    }
                    if (gamepad1.a && (intakeRunning == true) && toggleTimerIntake.seconds() > toggleWaitTime) {
                        robot.mtrIntake.setPower(0);
                        intakeRunning = false;
                        toggleTimerIntake.reset();
                    }
                    if (gamepad1.b) {
                        robot.mtrIntake.setPower(-1);
                        intakeRunning = true;
                        toggleTimerIntake.reset();
                    }


                case SHOOT_MODE:

                    /***
                     * Flywheel speed control
                     */

                    if (gamepad2.a && flywheelRunning == false && toggleTimerShooter.seconds() > toggleWaitTime) {
                        robot.mtrFlywheel.setVelocity(normalFlywheelVelocity);
                        targetVelo = normalFlywheelVelocity;
                        flywheelRunning = true;
                    }
                    if (gamepad2.b) {
                        robot.mtrFlywheel.setVelocity(psFlywheelVelocity);
                        targetVelo = psFlywheelVelocity;
                        flywheelRunning = true;
                    }
                    if (gamepad2.a && flywheelRunning == true && toggleTimerShooter.seconds() > toggleWaitTime) {
                        robot.mtrFlywheel.setPower(0);
                        targetVelo = 0;
                        flywheelRunning = false;
                    }

                    /***
                     * Shoot 3 rings
                     */

                    if (magIsUp) {
                        robot.mtrIntake.setPower(0);
                        intakeRunning = false;
                        if (gamepad2.right_bumper) {
                            currentState = ShootState.ZERO_RINGS_SHOT;
                        }
                    }

                    switch(currentState){
                        case NOTHING:
                            break;
                        case ZERO_RINGS_SHOT:
                            robot.svoRingPush.setPosition(ringPushOut);
                            waitFor(servoMoveTime);
                            robot.svoRingPush.setPosition(ringPushIn);
                            waitFor(timeBetweenShots);
                            currentState = ShootState.ONE_RING_SHOT;
                        case ONE_RING_SHOT:
                            robot.svoRingPush.setPosition(ringPushOut);
                            waitFor(servoMoveTime);
                            robot.svoRingPush.setPosition(ringPushIn);
                            waitFor(timeBetweenShots);
                            currentState = ShootState.TWO_RINGS_SHOT;
                        case TWO_RINGS_SHOT:
                            robot.svoRingPush.setPosition(ringPushOut);
                            waitFor(servoMoveTime);
                            robot.svoRingPush.setPosition(ringPushIn);
                            waitFor(timeBetweenShots);
                            currentState = ShootState.NOTHING;
                    }

                    if(gamepad2.y && currentState != ShootState.NOTHING){
                        currentState = ShootState.NOTHING;
                    }

                    if (gamepad2.right_trigger > 0.1) {
                        robot.svoRingPush.setPosition(ringJamnt);
                        waitFor(0.5);
                        robot.svoRingPush.setPosition(ringPushIn);
                    }

                default:
                    gamepadState = gamepadState.INTAKE_MODE;

            }



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("State Status: ", gamepadState);
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
    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / robot.batteryVoltageSensor.getVoltage()
        ));
    }
}
