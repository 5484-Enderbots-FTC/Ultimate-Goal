package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    
    VoltageSensor batteryVoltageSensor;
    /*
    DcMotorEx mtrBL , mtrBR , mtrFL , mtrFR , mtrIntake, mtrWobble, mtrFlywheel;
    Servo svoWobble, svoMagLift, svoRingPush, svoForkHold;

     */

    DcMotorEx mtrFlywheel;

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

        //Velocity PID
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        mtrFlywheel = hardwareMap.get(DcMotorEx.class, "mtrFlywheel");
        mtrFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        mtrFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFlywheel.setDirection(DcMotorEx.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType = mtrFlywheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        mtrFlywheel.setMotorType(motorConfigurationType);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(mtrFlywheel, MOTOR_VELO_PID);

        TuningController tuningController = new TuningController();

        double lastKp = 0.0;
        double lastKi = 0.0;
        double lastKd = 0.0;
        double lastKf = getMotorVelocityF();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        /*
        //Drivetrain motors
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

        //Intake & Wobble lift
        mtrIntake = hardwareMap.get(DcMotorEx.class, "mtrIntake");
        mtrIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        mtrIntake.setDirection(DcMotorEx.Direction.REVERSE);

        mtrWobble = hardwareMap.get(DcMotorEx.class, "mtrWobble");
        mtrWobble.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrWobble.setDirection(DcMotorEx.Direction.FORWARD);

        //Flywheel motors


        //Servos
        svoMagLift = hardwareMap.get(Servo.class,"svoMagLift");
        svoMagLift.setDirection(Servo.Direction.FORWARD);

        svoRingPush = hardwareMap.get(Servo.class,"svoRingPush");
        svoRingPush.setDirection(Servo.Direction.REVERSE);

        svoWobble = hardwareMap.get(Servo.class,"svoWobble");
        svoWobble.setDirection(Servo.Direction.FORWARD);

        svoForkHold = hardwareMap.get(Servo.class,"svoForkHold");
        svoForkHold.setDirection(Servo.Direction.FORWARD);

        svoRingPush.setPosition(ringPushIn);
        svoMagLift.setPosition(magDown);
        svoWobble.setPosition(wobbleHold);
        svoForkHold.setPosition(forkHold);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.clearAll();


         */
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
            //double targetVelo = tuningController.update();
            //mtrFlywheel.setVelocity(targetVelo);

            telemetry.addData("targetVelocity", targetVelo);

            double motorVelo = mtrFlywheel.getVelocity();
            telemetry.addData("velocity", motorVelo);
            //telemetry.addData("error", targetVelo - motorVelo);

            telemetry.addData("upperBound", TuningController.rpmToTicksPerSecond(TuningController.TESTING_MAX_SPEED * 1.15));
            telemetry.addData("lowerBound", 0);

            if (lastKp != MOTOR_VELO_PID.p || lastKi != MOTOR_VELO_PID.i || lastKd != MOTOR_VELO_PID.d || lastKf != MOTOR_VELO_PID.f) {
                setPIDFCoefficients(mtrFlywheel, MOTOR_VELO_PID);

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
            /*

            if((backwardsMode == false) && (slowMode == false)) {
                //default controls
                mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
                mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
                mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
                mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            }
            if((backwardsMode == true) && (slowMode == false)){
                //backwards mode only
                mtrBL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
                mtrBR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
                mtrFL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
                mtrFR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            }
            if((backwardsMode == false) && (slowMode == true)){
                //slow mode only
                mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
                mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
                mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
                mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
            }
            if((backwardsMode == true) && (slowMode == true)){
                //backwards and slow modes together
                mtrBL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
                mtrBR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
                mtrFL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
                mtrFR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
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

             */


            /**
             * Gamepad 2 Controls
             */
/*
            mtrWobble.setPower(gamepad2.right_stick_y);

            if (magIsUp){
                mtrIntake.setPower(0);
                if(gamepad2.right_bumper){
                    svoRingPush.setPosition(ringPushOut);
                    waitFor(0.5);
                    svoRingPush.setPosition(ringPushIn);
                }
            }
            if(gamepad2.right_trigger > 0.1){
                svoRingPush.setPosition(ringJamnt);
                waitFor(0.5);
                svoRingPush.setPosition(ringPushIn);
            }

 */

            if (gamepad2.y){
                mtrFlywheel.setVelocity(psFlywheelVelocity);
                targetVelo = psFlywheelVelocity;
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
/*
            if(gamepad2.dpad_up){
                svoMagLift.setPosition(magUp);
                mtrFlywheel.setPower(flywheelPower);
                magIsUp = true;
            }
            if(gamepad2.dpad_down){
                mtrIntake.setPower(1);
                svoMagLift.setPosition(magDown);
                mtrFlywheel.setPower(0);
                magIsUp = false;
            }
*/
            if(gamepad2.a){
                mtrFlywheel.setVelocity(normalFlywheelVelocity);
                targetVelo = normalFlywheelVelocity;
                //mtrFlywheel.setPower(flywheelPower);
            }
            if(gamepad2.b){
                mtrFlywheel.setPower(0);
                targetVelo = 0;
            }
/*
            if(gamepad2.left_bumper && (forkHeld == false) && (toggleTimerF.seconds() > toggleWaitTime)){
                svoForkHold.setPosition(forkHold);
                forkHeld = true;
                toggleTimerF.reset();
            }
            else if(gamepad2.left_bumper && (forkHeld == true) && (toggleTimerF.seconds() > toggleWaitTime)){
                svoForkHold.setPosition(forkRelease);
                forkHeld = false;
                toggleTimerF.reset();
            }

 */

            /**
             * Telemetry
             */
/*
            telemetry.addData("Status", " Run Time: " + runtime.toString());
            telemetry.addData("Timer Status", " Time: " + timer.toString());
            telemetry.addData("backwards timer sec:", " " + toggleTimerB.seconds());
            telemetry.addData("slow timer sec:", " " + toggleTimerS.seconds());
            telemetry.update();

 */
        }


    }
    private void waitFor(double waittime) {
        timer.reset();
        while (timer.seconds() < waittime) {
            //pls keep driving lol
            /*
            mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));

             */
        }
    }
    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    public static double getMotorVelocityF() {
        return 32767 * 60.0 / (TuningController.MOTOR_MAX_RPM * TuningController.MOTOR_TICKS_PER_REV);
    }
}


