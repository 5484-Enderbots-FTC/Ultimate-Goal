package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.test_code.TuningController;

@TeleOp(name="teleop w/ PID",group="testing")
public class teleop_control_pid extends LinearOpMode{
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime toggleTimerS = new ElapsedTime();
    ElapsedTime toggleTimerB = new ElapsedTime();
    ElapsedTime toggleTimerF = new ElapsedTime();
    ElapsedTime toggleTimerR = new ElapsedTime();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    hardwareUltimateGoal robot   = new hardwareUltimateGoal();

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(80, 0, 60, 17.5);

    /***

     ~ Constants ~

     */

    double normalFlywheelVelocity = 800;
    double psFlywheelVelocity = 600;
    double targetVelo = 0;
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


        TuningController tuningController = new TuningController();

        double lastKp = 0.0;
        double lastKi = 0.0;
        double lastKd = 0.0;
        double lastKf = getMotorVelocityF();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.clearAll();

        //BEGIN
        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();
        timer.reset();
        toggleTimerB.reset();
        toggleTimerS.reset();
        toggleTimerF.reset();

        tuningController.start();
        
        while (!isStopRequested() && opModeIsActive()) {
            /**
             * Telemetry & Velocity PID
             */
            double motorVelo = robot.mtrFlywheel.getVelocity();
            telemetry.addData("Status", " Run Time: " + runtime.toString());
            telemetry.addData("targetVelocity", targetVelo);
            telemetry.addData("velocity", motorVelo);
            telemetry.addData("error", targetVelo - motorVelo);

            if (lastKp != MOTOR_VELO_PID.p || lastKi != MOTOR_VELO_PID.i || lastKd != MOTOR_VELO_PID.d || lastKf != MOTOR_VELO_PID.f) {
                setPIDFCoefficients(robot.mtrFlywheel, MOTOR_VELO_PID);

                lastKp = MOTOR_VELO_PID.p;
                lastKi = MOTOR_VELO_PID.i;
                lastKd = MOTOR_VELO_PID.d;
                lastKf = MOTOR_VELO_PID.f;
            }

            tuningController.update();
            telemetry.update();

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



            if (gamepad2.y && (ringSwiped == false)){
                robot.svoRingPush.setPosition(ringJamnt);
                ringSwiped = true;
            }
            if (gamepad2.y && (ringSwiped == true)){
                robot.svoRingPush.setPosition(ringPushIn);
                ringSwiped = false;
            }

            if(gamepad2.dpad_up){
                robot.svoMagLift.setPosition(magUp);
                robot.mtrFlywheel.setVelocity(normalFlywheelVelocity);
                targetVelo = normalFlywheelVelocity;
                magIsUp = true;
            }


            if(gamepad2.dpad_down){
                robot.mtrIntake.setPower(1);
                robot.svoMagLift.setPosition(magDown);
                robot.mtrFlywheel.setPower(0);
                magIsUp = false;
            }
            if(gamepad2.a){
                robot.mtrFlywheel.setVelocity(normalFlywheelVelocity);
                targetVelo = normalFlywheelVelocity;

            }
            if (gamepad2.y){
                robot.mtrFlywheel.setVelocity(psFlywheelVelocity);
                targetVelo = psFlywheelVelocity;
            }
            if(gamepad2.b){
                robot.mtrFlywheel.setPower(0);
                targetVelo = 0;
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
    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / robot.batteryVoltageSensor.getVoltage()
        ));
    }

    public static double getMotorVelocityF() {
        return 32767 * 60.0 / (TuningController.MOTOR_MAX_RPM * TuningController.MOTOR_TICKS_PER_REV);
    }

}


