package org.firstinspires.ftc.teamcode.skystone_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="two_remote_chomper", group="teleop")
@Disabled
public class teleop_two_remotes_redesign extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    //motors
    DcMotor mtrBL;
    DcMotor mtrBR;
    DcMotor mtrFL;
    DcMotor mtrFR;
    DcMotor mtrLift1;
    DcMotor mtrLift2;

    //servos
    Servo extend; //0
    Servo grab; //1
    Servo hooks; //2
    Servo LED_strip; //3
    Servo capstone; //4

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
    ModernRoboticsI2cRangeSensor rangeSensor;

    /**

     CONSTANTS

     */

    //servos
    double extendOut = 0;
    final double extendIn = 0.195;
    double extendInChange = 0.195;
    double extendInit = 0.45;

    double fullGrab = 0.25;
    double releaseGrab = 0.7;
    double initGrab = 0.76;

    double hooksDown = 0.7;
    double hooksUp = 0.54;

    double capstoneRelease = 0.75;
    double capstoneHold = 0;

    //motors
    double liftReversal = 0.05;
    double liftStop = -0;

    //alliance switch
    double alliance = 0;
    final double red = 1;
    final double blue = -1;
    int ParkingSpot = 0;
    int insideLane = 0;
    int outsideLane = 1;
    double LEDblue = 0.3845;
    double LEDred = 0.3945;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // motors
        mtrBL = hardwareMap.get(DcMotor.class, "leftFront_drive");
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setDirection(DcMotor.Direction.REVERSE);

        mtrBR = hardwareMap.get(DcMotor.class, "rightBack_drive");
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setDirection(DcMotor.Direction.FORWARD);

        mtrFL = hardwareMap.get(DcMotor.class, "leftBack_drive");
        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFL.setDirection(DcMotor.Direction.REVERSE);

        mtrFR = hardwareMap.get(DcMotor.class, "rightFront_drive");
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setDirection(DcMotor.Direction.FORWARD);

        mtrLift1 = hardwareMap.get(DcMotor.class, "mtrLift1");
        mtrLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrLift1.setDirection(DcMotor.Direction.REVERSE);

        mtrLift2 = hardwareMap.get(DcMotor.class, "mtrLift2");
        mtrLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mtrLift2.setDirection(DcMotor.Direction.REVERSE);

        //servos
        extend = hardwareMap.get(Servo.class, "extend");
        extend.setDirection(Servo.Direction.FORWARD);

        grab = hardwareMap.get(Servo.class, "grab");
        grab.setDirection(Servo.Direction.REVERSE);

        hooks = hardwareMap.get(Servo.class, "hooks");
        hooks.setDirection(Servo.Direction.FORWARD);

        capstone = hardwareMap.get(Servo.class, "capstone");
        capstone.setDirection(Servo.Direction.FORWARD);

        //sensors
        distance_left = hardwareMap.get(DistanceSensor.class, "distance_left");
        distance_right = hardwareMap.get(DistanceSensor.class, "distance_right");

        color_sensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

        //LED lights
        LED_strip = hardwareMap.get(Servo.class, "LED_strip");

        //limit switch
        bottomLimit = hardwareMap.get(TouchSensor.class, "bottomLimit");

        //potentiometers
        delaySlide = hardwareMap.get(AnalogInput.class, "delaySlide");

        //switches
        alliance_switch = hardwareMap.get(DigitalChannel.class, "alliance_switch");
        position_switch = hardwareMap.get(DigitalChannel.class, "position_switch");

        //set on init
        if (alliance_switch.getState() == true) {
            telemetry.addData("Alliance:", "Red");
            alliance = red;
            LED_strip.setPosition(LEDred);
        } else {
            telemetry.addData("Alliance", "Blue");
            alliance = blue;
            LED_strip.setPosition(LEDblue);
        }
        if (position_switch.getState() == true) {
            telemetry.addData("Innermost spot:", "### ___");
            telemetry.update();
            ParkingSpot = insideLane;
        } else {
            telemetry.addData("Outer Spot", "___ ###");
            telemetry.update();
            ParkingSpot = outsideLane;
        }

        grab.setPosition(initGrab);
        hooks.setPosition(hooksUp);
        extend.setPosition(extendIn);
        capstone.setPosition(capstoneHold);


        //start
        waitForStart();
        runtime.reset();
        telemetry.update();

        while (opModeIsActive()) {

            /**

             GAMEPAD 1 CONTROLS

             **/

            //DRIVING
            mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (1-(gamepad1.right_trigger*0.75)));
            mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * (1-(gamepad1.right_trigger*0.75)));
            mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * (1-(gamepad1.right_trigger*0.75)));
            mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (1-(gamepad1.right_trigger*0.75)));


            //GRABBING
            //full grab
            while(gamepad1.b) {
                if(distance_left.getDistance(DistanceUnit.INCH) > 8){
                    strafe(0.3);
                    telemetry.addData("Left distance", ": " + distance_left.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Right distance", ": " + distance_right.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }
                if(distance_right.getDistance(DistanceUnit.INCH) > 8){
                    strafe(-0.3);
                    telemetry.addData("Left distance", ": " + distance_left.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Right distance", ": " + distance_right.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }
                if((distance_right.getDistance(DistanceUnit.INCH)<8) && (distance_left.getDistance(DistanceUnit.INCH)<8)){
                    brakeDriveMotors();
                    if(distance_left.getDistance(DistanceUnit.INCH) > 2.4){
                        forward(0.3);
                    }
                    telemetry.addData("Left distance", ": " + distance_left.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Right distance", ": " + distance_right.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }
            }
            if (color_sensor.alpha() >= 3900) {
                brakeMotors();
            }
            if (color_sensor.getLightDetected() >= 0.29) {
                brakeMotors();
            }

            if(gamepad1.a){
                capstone.setPosition(capstoneRelease);
            }

            //FOUNDATION MOVEMENT
            //claws both go down
            if(gamepad1.right_bumper) {
                hooks.setPosition(hooksDown);
            }
            //claws both retract
            if(gamepad1.left_bumper){
                hooks.setPosition(hooksUp);
                grab.setPosition(releaseGrab);
                extend.setPosition(extendIn + 0.05);
            }



            /**

             GAMEPAD 2 CONTROLS

             **/

            //EXTENTION
            if (gamepad2.dpad_down){
                extend.setPosition(extendIn);
            }
            if (gamepad2.dpad_up){
                extend.setPosition(extendOut);
            }


            if(gamepad2.a){
                if(extend.getPosition() == 0.195){
                    extendInChange = 0.195;
                }
                extendInChange = extendInChange - 0.05;
                extend.setPosition(extendInChange);
            }
            if(gamepad2.b){
                if(extend.getPosition() == 0.195){
                    extendInChange = 0.195;
                }
                extendInChange = extendInChange + 0.05;
                extend.setPosition(extendInChange);
            }

            //GRABBING
            //full grab
            if(gamepad2.right_trigger == 1) {
                grab.setPosition(fullGrab);
            }
            //release grab
            if(gamepad2.left_trigger == 1) {
                grab.setPosition(releaseGrab);
            }
/*
            if(gamepad2.b){
                initBox();
            }

            if(gamepad2.y){
                initBoxReverse();
            }

            */

                //bottom limit switch
                if(gamepad2.left_stick_y>0 && bottomLimit.isPressed()){
                    mtrLift1.setPower(liftStop);
                    mtrLift2.setPower(liftStop);
                }
                else {
                    //left stick pushed down makes it a value greater than 0 so positive power makes the lift go down
                    //and negative power makes the lift go up
                    mtrLift1.setPower(gamepad2.left_stick_y);
                    mtrLift2.setPower(gamepad2.left_stick_y);
                }

            //shows elapsed time and limit switch values
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Bottom Limit state", ": " + bottomLimit.isPressed());
            telemetry.addData("Left distance", ": " + distance_left.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right distance", ": " + distance_right.getDistance(DistanceUnit.INCH));
            telemetry.addData("Color sensor reading", ": " + color_sensor.getLightDetected());
            telemetry.addData("Color sensor reading: ", color_sensor.alpha());
            telemetry.addData("Delay slide reading", ": " + delaySlide.getVoltage());
            telemetry.addData("Delay slide actual delay (sec)", ": " + delaySlide.getVoltage()*8.95);
            telemetry.addData("Range Sensor reading", ": " + rangeSensor.rawUltrasonic());
            telemetry.addData("Range Sensor reading Cm", ": " + rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
    private void resetEncoders() {
        mtrLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void runLiftToPosition() {
        mtrLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void runWithoutEncoder() {
        mtrLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void brakeLiftMotors() {
        mtrLift1.setPower(0);
        mtrLift2.setPower(0);
    }
    private void brakeDriveMotors() {
        mtrFR.setPower(0);
        mtrFL.setPower(0);
        mtrBL.setPower(0);
        mtrBR.setPower(0);
    }
    private void liftIsBusy() {
        while (mtrLift1.isBusy()){
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
        liftPosition(position);
        runLiftToPosition();
        liftPower(power);
        liftIsBusy();
        brakeLiftMotors();
        runWithoutEncoder();
    }
    private void initBox() {
        liftEncoder(-300, -0.4);
        extend.setPosition(extendInit);
        liftEncoder(300,0.2);
    }
    private void initBoxReverse() {
        liftEncoder(400, 0.4);
        extend.setPosition(extendIn);
        liftEncoder(-300,-0.2);
    }
    private void forward(double power) {
        mtrFR.setPower(-power);
        mtrFL.setPower(-power);
        mtrBL.setPower(-power);
        mtrBR.setPower(-power);
    }
    private void strafe(double power) {
        mtrBL.setPower(-power);
        mtrBR.setPower(-power);
        mtrFL.setPower(power);
        mtrFR.setPower(power);


    }
    private void brakeMotors() {
        mtrFL.setPower(0);
        mtrFR.setPower(0);
        mtrBL.setPower(0);
        mtrBR.setPower(0);
    }
}


