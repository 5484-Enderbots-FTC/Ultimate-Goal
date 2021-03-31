package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "encoder test cri", group = "auto")
@Disabled
public class encoder_testing_i_hate_it_here extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    //OpenCV stuff
    OpenCvCamera webcam;
    auto_wobble_shoot_park_FSM.RingStackDeterminationPipeline pipeline;

    //motors
    DcMotorEx mtrBL , mtrBR , mtrFL , mtrFR , mtrIntake, mtrWobble, mtrFlywheel;
    Servo svoWobble, svoMagLift, svoRingPush;

    //constants
    private final double ticksPerInchCalibrated = 43.3305;

    private final double ticksPerInchWobbleLift = 43.3305;

    double magDown = 0.85;
    double magUp = 0.58;
    double ringPushOut = 0.6;
    double ringPushIn = 0.75;
    double wobbleRelease = 0.48;
    double wobbleHold = 0.1;

    @Override
    public void runOpMode() {
        //motors
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
        mtrIntake.setDirection(DcMotorEx.Direction.FORWARD);

        mtrFlywheel = hardwareMap.get(DcMotorEx.class, "mtrFlywheel");
        mtrFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        mtrFlywheel.setDirection(DcMotorEx.Direction.REVERSE);

        mtrWobble = hardwareMap.get(DcMotorEx.class, "mtrWobble");
        mtrWobble.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrWobble.setDirection(DcMotorEx.Direction.FORWARD);

        svoMagLift = hardwareMap.get(Servo.class,"svoMagLift");
        svoMagLift.setDirection(Servo.Direction.FORWARD);

        svoRingPush = hardwareMap.get(Servo.class,"svoRingPush");
        svoRingPush.setDirection(Servo.Direction.REVERSE);

        svoWobble = hardwareMap.get(Servo.class,"svoWobble");
        svoWobble.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        waitForStart();

            runtime.reset();
           //encoderForward(0.5,12);
           //what it was supposed to go: 12inches
            //what it went:
            //what the tick value was: 91.44
            //what i changed it to: 43.0305

        encoderLiftUp(0.5,5);

        
    }
    private void waitFor(double waittime) {
        timer.reset();
        while (timer.seconds() < waittime) {
        }
    }

    private void pushARing(){
        svoRingPush.setPosition(ringPushOut);
        waitFor(0.5);
        svoRingPush.setPosition(ringPushIn);
    }
    private void shootThree(double inBetweenRingTime){
        mtrFlywheel.setPower(1);
        waitFor(1.5);
        pushARing();
        waitFor(inBetweenRingTime);
        pushARing();
        waitFor(inBetweenRingTime);
        pushARing();
        waitFor(inBetweenRingTime);
        mtrFlywheel.setPower(0);

    }
    private void resetEncoders() {
        mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrWobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void runToPosition() {
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrWobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void brakeMotors() {
        mtrFL.setPower(0);
        mtrFR.setPower(0);
        mtrBL.setPower(0);
        mtrBR.setPower(0);
        mtrWobble.setPower(0);
    }
    private void mtrFRisBusy() {
        while (mtrFR.isBusy()){
        }
    }
    private void mtrWobbleisBusy(){
        while (mtrWobble.isBusy()){
        }
    }
    private void mtrBLisBusy() {
        while (mtrBL.isBusy()){
        }
    }
    private void runWithoutEncoder() {
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrWobble.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void forward(double power) {
        mtrBL.setPower(power);
        mtrBR.setPower(power);
        mtrFL.setPower(power);
        mtrFR.setPower(power);


    }
    private void forwardPosition(int distance_inches) {
        mtrBL.setTargetPosition(distance_inches*(int)ticksPerInchCalibrated);
        mtrBR.setTargetPosition(distance_inches*(int)ticksPerInchCalibrated);
        mtrFL.setTargetPosition(distance_inches*(int)ticksPerInchCalibrated);
        mtrFR.setTargetPosition(distance_inches*(int)ticksPerInchCalibrated);
    }
    private void encoderForward(double power, int distance_inches){
        resetEncoders();
        forwardPosition(-distance_inches);
        runToPosition();
        forward(power);
        mtrFRisBusy();
        brakeMotors();
        runWithoutEncoder();
    }

    private void strafe(double power) {
        mtrBL.setPower(power);
        mtrBR.setPower(-power);
        mtrFL.setPower(-power);
        mtrFR.setPower(power);
    }
    private void strafePosition(int distance_inches){
        mtrBL.setTargetPosition(distance_inches*(int)ticksPerInchCalibrated);
        mtrBR.setTargetPosition(-distance_inches*(int)ticksPerInchCalibrated);
        mtrFL.setTargetPosition(-distance_inches*(int)ticksPerInchCalibrated);
        mtrFR.setTargetPosition(distance_inches*(int)ticksPerInchCalibrated);
    }
    private void encoderStrafe(double power, int distance_inches){
        resetEncoders();
        strafePosition(distance_inches);
        runToPosition();
        strafe(power);
        mtrFRisBusy();
        brakeMotors();
        runWithoutEncoder();
    }

    private void liftUp(double power){
        mtrWobble.setPower(power);
    }
    private void liftUpPosition(int distance_inches){
        mtrWobble.setTargetPosition(distance_inches*(int)ticksPerInchWobbleLift);
    }
    private void encoderLiftUp(double power, int distance_inches){
        resetEncoders();
        liftUpPosition(distance_inches);
        runToPosition();
        liftUp(power);
        mtrWobbleisBusy();
        brakeMotors();
        runWithoutEncoder();
    }


}
