package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "teleop w/ PID", group = "testing")
public class teleop_control_pid extends LinearOpMode {
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


    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(100, 0, 30, 18);

    /***

     ~ Constants ~

     */

    double timeBetweenShots = 0.37;
    double servoMoveTime = 0.25;

    double normalFlywheelVelocity = 1350;
    double psFlywheelVelocity = 1200;
    double targetVelo = 0;
    double magDown = 0.85;
    double magUp = 0.58;
    double ringPushOut = 0.6;
    double ringPushIn = 0.75;
    double ringJamnt = 0.2;
    double wobbleRelease = 0.37;
    double wobbleHold = 0.2;
    double forkHold = 0.75;
    double initFork = 0.85;
    double forkRaise = 0.79;
    double forkRelease = 0.5;
    double toggleWaitTime = 0.5;

    boolean magIsUp = false;
    boolean backwardsMode = false;
    boolean slowMode = false;
    boolean forkHeld = true;
    boolean intakeRunning = false;
    boolean flywheelRunning = false;

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
             * Telemetry & Velocity PID
             */
            double motorVelo = robot.mtrFlywheel.getVelocity();
            /*
            telemetry.addData("Status", " Run Time: " + runtime.toString());
            telemetry.addData("targetVelocity", targetVelo);
            telemetry.addData("velocity", motorVelo);
            telemetry.addData("error", targetVelo - motorVelo);

             */
            telemetry.addData("topLimit: ", robot.topLimit.getState());
            telemetry.update();

            /**
             * Gamepad 1 Controls
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
                robot.mtrIntake.setPower(0.75);
            }
            /*
            if (gamepad1.a && (intakeRunning == true) && toggleTimerIntake.seconds() > toggleWaitTime) {
                robot.mtrIntake.setPower(0);
                intakeRunning = false;
                toggleTimerIntake.reset();
            }
            if (gamepad1.b && (revIntakeRunning == false) && toggleTimerRevIntake.seconds() > toggleWaitTime) {
                robot.mtrIntake.setPower(-1);
                intakeRunning = false;
                revIntakeRunning = true;
                toggleTimerIntake.reset();
            }
            if (gamepad1.b && (revIntakeRunning == true) && toggleTimerRevIntake.seconds() > toggleWaitTime) {
                robot.mtrIntake.setPower(0);
                intakeRunning = false;
                revIntakeRunning = false;
                toggleTimerIntake.reset();
            }

             */

            /**
             * Gamepad 2 Controls
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

            /*
            if(gamepad2.y && currentState != ShootState.NOTHING){
                currentState = ShootState.NOTHING;
            }
            */

            if (gamepad2.a && flywheelRunning == false && toggleTimerShooter.seconds() > toggleWaitTime) {
                robot.mtrFlywheel.setVelocity(normalFlywheelVelocity);
                targetVelo = normalFlywheelVelocity;
                flywheelRunning = true;
                toggleTimerShooter.reset();
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
                toggleTimerShooter.reset();
            }

            if (gamepad2.right_trigger > 0.1) {
                robot.svoRingPush.setPosition(ringJamnt);
                waitFor(0.3);
                robot.svoRingPush.setPosition(ringPushIn);
            }

            if (gamepad2.dpad_up) {
                robot.svoMagLift.setPosition(magUp);
                robot.mtrFlywheel.setVelocity(normalFlywheelVelocity);
                targetVelo = normalFlywheelVelocity;
                flywheelRunning = true;
                magIsUp = true;
            }

            if (gamepad2.dpad_down) {
                robot.mtrIntake.setPower(1);
                intakeRunning = true;
                robot.svoMagLift.setPosition(magDown);
                robot.mtrFlywheel.setPower(0);
                flywheelRunning = false;
                magIsUp = false;
            }

            robot.mtrWobble.setPower(gamepad2.right_stick_y);

            if(gamepad2.left_bumper){
                robot.svoForkHold.setPosition(forkHold);
            }
            if(gamepad2.left_trigger > 0.1){
                robot.svoForkHold.setPosition(forkRelease);
                waitFor(1);
                robot.svoForkHold.setPosition(forkRaise);
            }
            if(gamepad2.y){
                robot.svoForkHold.setPosition(initFork);
            }
/*
            if(robot.topLimit.getState() == true){
                //limit switch isn't pressed

            }else{
                //it do be pressed
                robot.mtrWobble.setPower(0);
            }

 */

/*
            if (gamepad2.left_bumper && (forkHeld == false) && (toggleTimerF.seconds() > toggleWaitTime)) {
                robot.svoForkHold.setPosition(forkHold);
                forkHeld = true;
                toggleTimerF.reset();
            } else if (gamepad2.left_bumper && (forkHeld == true) && (toggleTimerF.seconds() > toggleWaitTime)) {
                robot.svoForkHold.setPosition(forkRelease);
                forkHeld = false;
                toggleTimerF.reset();
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

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / robot.batteryVoltageSensor.getVoltage()
        ));
    }


}


