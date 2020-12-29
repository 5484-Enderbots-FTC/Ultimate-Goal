package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Foundation Only", group = "Concept")
@Disabled
public class foundationOnly extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    //motors
    DcMotor mtrBL = null;
    DcMotor mtrBR = null;
    DcMotor mtrFL = null;
    DcMotor mtrFR = null;
    DcMotor mtrLift1;
    DcMotor mtrLift2;

    //servos
    Servo extend = null;
    Servo grab = null;
    Servo hooks = null;
    Servo LED_strip = null;

    //limit switches
    TouchSensor bottomLimit;

    //alliance switch
    DigitalChannel alliance_switch;
    DigitalChannel position_switch;

    //potentiometers
    AnalogInput delaySlide;

    //sensors
    DistanceSensor distance_left;
    DistanceSensor distance_right;
    RevColorSensorV3 color_sensor;

    //constants
    double colorRed = 0.6695;
    double colorBlue = 0.7445;
    double colorBlack = 0.7745;
    double colorWhite = 0.7595;

    int strafeSwapper = 1;
    double alliance = 0;
    final double redAlliance = 1;
    final double blueAlliance = -1;
    int ParkingSpot = 0;
    int in = 0;
    int out = 1;
    private static final int red = 1;
    private static final int blue = -1;

    private final double ticksPerMm = 1.68240559922;
    private final double ticksPerMmCalibrated = 1.518268;
    private final double ticksPerMmLift = 4.5;

    double fullGrab = 0.15;
    double releaseGrab = 0.7;
    double initGrab = 0.76;

    double extendOut = 0;
    final double extendIn = 0.195;
    final double extendInit = 0.45;

    double hooksDown = 0.67;
    double hooksUp = 0.59;

    double LEDwhite = 0.7595;
    double LEDred = 0.6695;
    double LEDblue = 0.7445;

    int initialStrafe = (int) (250*ticksPerMmCalibrated);
    int wallToFoundation = (int) (660*ticksPerMmCalibrated);
    int foundationSlow = (int) (200*ticksPerMmCalibrated);
    int backUpFromFoundation = (int) (200*ticksPerMmCalibrated);
    int strafeToWall = (int) (220*ticksPerMmCalibrated);
    int straightPark = (int) (750*ticksPerMmCalibrated);
    int straightParkWithColor = (int) (600*ticksPerMmCalibrated);
    int halfPark = (int) (230*ticksPerMmCalibrated);
    int finishPark = (int) (530*ticksPerMmCalibrated);
    int strafePark = (int) (680*ticksPerMmCalibrated);

    @Override
    public void runOpMode() {
        //configuration of robot stuff
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        //motors
        mtrFR = hardwareMap.get(DcMotor.class, "rightFront_drive");
        mtrFR.setDirection(DcMotor.Direction.REVERSE);

        mtrFL = hardwareMap.get(DcMotor.class, "leftFront_drive");
        mtrFL.setDirection(DcMotor.Direction.FORWARD);

        mtrBL = hardwareMap.get(DcMotor.class, "leftBack_drive");
        mtrBL.setDirection(DcMotor.Direction.FORWARD);

        mtrBR = hardwareMap.get(DcMotor.class, "rightBack_drive");
        mtrBR.setDirection(DcMotor.Direction.REVERSE);

        //lift motors
        mtrLift1 = hardwareMap.get(DcMotor.class, "mtrLift1");
        mtrLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrLift1.setDirection(DcMotor.Direction.REVERSE);

        mtrLift2 = hardwareMap.get(DcMotor.class, "mtrLift2");
        mtrLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mtrLift2.setDirection(DcMotor.Direction.REVERSE);

        //servos
        grab = hardwareMap.get(Servo.class, "grab");
        grab.setDirection(Servo.Direction.REVERSE);
        extend = hardwareMap.get(Servo.class, "extend");
        extend.setDirection(Servo.Direction.FORWARD);
        hooks = hardwareMap.get(Servo.class, "hooks");
        hooks.setDirection(Servo.Direction.FORWARD);

        //sensors
        distance_left = hardwareMap.get(DistanceSensor.class, "distance_left");
        distance_right = hardwareMap.get(DistanceSensor.class, "distance_right");

        color_sensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");

        //LED lights
        LED_strip = hardwareMap.get(Servo.class, "LED_strip");

        //limit switch
        bottomLimit = hardwareMap.get(TouchSensor.class, "bottomLimit");

        //potentiometers
        delaySlide = hardwareMap.get(AnalogInput.class, "delaySlide");

        //switches
        alliance_switch = hardwareMap.get(DigitalChannel.class, "alliance_switch");
        position_switch = hardwareMap.get(DigitalChannel.class, "position_switch");

        //determine side
        if (alliance_switch.getState() == true) {
            telemetry.addData("Alliance:", "Red");
            strafeSwapper = 1;
            alliance = redAlliance;
            LED_strip.setPosition(colorRed);
        } else {
            telemetry.addData("Alliance", "Blue");
            strafeSwapper = -1;
            alliance = blueAlliance;
            LED_strip.setPosition(colorBlue);
        }
        if (position_switch.getState() == true) {
            telemetry.addData("Innermost spot:", "### ___");
            ParkingSpot = in;
        } else {
            telemetry.addData("Outer Spot", "___ ###");
            ParkingSpot = out;
        }


        //all init movements
        hooks.setPosition(hooksUp);
        grab.setPosition(releaseGrab);
        liftEncoder(-50, -1);
        extend.setPosition(extendInit);
        liftPower(-0.2);
        waitFor(1);
        brakeLiftMotors();
        /**

         Code begins here

         */


        waitForStart();
        runtime.reset();
        liftPower(0.2);
        waitFor(0.75);

        //strafe over a little to get in position
        encoderStrafe(0.2,initialStrafe);

        //drive up to foundation
        encoderForward(0.5,wallToFoundation);
        encoderForward(0.23,foundationSlow);

        //lower hooks to grab foundation
        lowerHooks();
        waitFor(0.5);

        if (alliance == blueAlliance) {
            randomMove(-1600, -330, -1600, -330, 0.7);
        } else {
            randomMove(-330, -1600, -330, -1600, 0.7);
            //530,1800,530,1800
        }
        telemetry.addData("Foundation Turn:", "Done");

        //push into wall
        if (alliance == blueAlliance) {
            randomMove(450, 550, 450, 550, 0.5);
        } else {
            randomMove(550, 450, 550, 450, 0.5);
        }

        //lift up hooks
        raiseHooks();

        //back off from foundation a bit
        encoderForward(-0.3, -backUpFromFoundation);

        //strafe right into wall
        encoderStrafe(0.2, strafeToWall);

        //park
        encoderForward(-0.5,-straightParkWithColor);
        findBridgeLine(-0.2);

        /**

         End of Program

         */

    }

    private void waitFor(double waittime) {
        runtime.reset();
        while (runtime.time() < waittime) {
        }
    }
    private void resetEncoders() {
        mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void runToPosition() {
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void brakeMotors() {
        mtrFL.setPower(0);
        mtrFR.setPower(0);
        mtrBL.setPower(0);
        mtrBR.setPower(0);
    }
    private void mtrFRisBusy() {
        while (mtrFR.isBusy()){
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
    }
    private void runUsingEncoders() {
        mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void forward(double power) {
        mtrFR.setPower(power);
        mtrFL.setPower(power);
        mtrBL.setPower(power);
        mtrBR.setPower(power);
    }
    private void forwardPosition(int position) {
        mtrFR.setTargetPosition(position);
        mtrFL.setTargetPosition(position);
        mtrBR.setTargetPosition(position);
        mtrBL.setTargetPosition(position);
    }
    private void encoderForward(double power, int position){
        resetEncoders();
        forwardPosition(position);
        runToPosition();
        forward(power);
        mtrFRisBusy();
        brakeMotors();
        runWithoutEncoder();
    }
    private void encoderForwardTime(double power, int position, double time){
        resetEncoders();
        forwardPosition(position);
        runToPosition();
        forward(power);
        while (mtrFR.isBusy()) {
            if(runtime.time()>time){
                break;
            }
        }
        brakeMotors();
        runWithoutEncoder();
    }

    private void strafe(double power) {
        mtrFR.setPower(-power * strafeSwapper);
        mtrFL.setPower(power * strafeSwapper);
        mtrBL.setPower(-power * strafeSwapper);
        mtrBR.setPower(power * strafeSwapper);
    }
    private void strafePosition(int position){
        mtrFR.setTargetPosition(-position * strafeSwapper);
        mtrFL.setTargetPosition(position * strafeSwapper);
        mtrBR.setTargetPosition(position * strafeSwapper);
        mtrBL.setTargetPosition(-position * strafeSwapper);
    }
    private void encoderStrafe(double power, int position){
        resetEncoders();
        strafePosition(position);
        runToPosition();
        strafe(power);
        mtrFRisBusy();
        brakeMotors();
        runWithoutEncoder();
    }

    private void lowerHooks() {
        hooks.setPosition(hooksDown);
    }

    private void raiseHooks() {
        hooks.setPosition(hooksUp);
    }
    private void brakeLiftMotors() {
        mtrLift1.setPower(0);
        mtrLift2.setPower(0);
    }
    private void runLiftToPosition() {
        mtrLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void liftIsBusy() {
        while (mtrLift1.isBusy()){
            runtime.reset();
            if(runtime.time()>2){
                break;
            }
        }
    }
    private void liftPower(double power){
        mtrLift1.setPower(power);
        mtrLift2.setPower(power);
    }
    private void liftPosition(int position){
        mtrLift1.setTargetPosition(position);
        mtrLift2.setTargetPosition(position);
    }
    private void liftEncoder (int position, double power){
        resetEncoders();
        int positionTicks;
        positionTicks = (int) (position * ticksPerMmLift);
        liftPosition(positionTicks);
        runLiftToPosition();
        liftPower(power);
        liftIsBusy();
        brakeLiftMotors();
        runWithoutEncoder();
    }

    private void park(double power){
        if(ParkingSpot == in){
            encoderForward(-power,-halfPark);
            encoderStrafe(-power,-strafePark);
            encoderForward(-power,-finishPark);
        }
        if(ParkingSpot == out){
            encoderForward(-power, -straightPark);
        }
    }

    private void speedForward(double power) {
        resetEncoders();
        runUsingEncoders();
        forward(power);
        runWithoutEncoder();
    }

    private void findBridgeLine(double power){
        while(!isStopRequested()) {
            telemetry.addData("Color sensor reading", ": " + color_sensor.getLightDetected());
            telemetry.update();
            if (color_sensor.getLightDetected() > 0.3) {
                brakeMotors();
                break;
            }
            else{
                speedForward(power);
            }
        }
    }


    private void randomMove(int FL_Dist, int FR_Dist, int BL_Dist, int BR_Dist, double power) {
        resetEncoders();

        int maxDistFront;
        int maxDistBack;
        int maxDist;

        int FL_DistRound;
        int FR_DistRound;
        int BL_DistRound;
        int BR_DistRound;

        double velocityRatio;

        maxDistFront = Math.max(Math.abs(FL_Dist), Math.abs(FL_Dist));
        maxDistBack = Math.max(Math.abs(BL_Dist), Math.abs(BR_Dist));
        maxDist = Math.max(maxDistFront, maxDistBack);
        velocityRatio = power / maxDist;

        FR_DistRound = (int) (FR_Dist * ticksPerMm);
        FL_DistRound = (int) (FL_Dist * ticksPerMm);
        BR_DistRound = (int) (BR_Dist * ticksPerMm);
        BL_DistRound = (int) (BL_Dist * ticksPerMm);

        mtrFR.setTargetPosition(FR_DistRound);
        mtrFL.setTargetPosition(FL_DistRound);
        mtrBL.setTargetPosition(BL_DistRound);
        mtrBR.setTargetPosition(BR_DistRound);
        runToPosition();

        telemetry.addData("Velocity:", velocityRatio);
        telemetry.update();

        mtrFR.setPower(velocityRatio * FR_Dist);
        mtrFL.setPower(velocityRatio * FL_Dist);
        mtrBL.setPower(velocityRatio * BL_Dist);
        mtrBR.setPower(velocityRatio * BR_Dist);
        runtime.reset();
        while (mtrFR.isBusy() && mtrBL.isBusy() && mtrBR.isBusy() && mtrFL.isBusy()) {
            if (runtime.time() > 4) {
                break;
            }
        }
        brakeMotors();
    }
}
