/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.State;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "wobble shoot park FSM", group = "auto")
public class auto_wobble_shoot_park_FSM extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    //OpenCV stuff
    OpenCvCamera webcam;
    RingStackDeterminationPipeline pipeline;

    //motors
    hardwareUltimateGoal robot = new hardwareUltimateGoal();

    State currentState;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(100, 0, 30, 18);

    //constants
    private final double ticksPerInchCalibrated = 43.3305;

    double normalFlywheelVelocity = 1350;
    double magDown = 0.85;
    double magUp = 0.58;
    double ringPushOut = 0.6;
    double ringPushIn = 0.75;
    double wobbleRelease = 0.37;
    double wobbleHold = 0.2;
    double forkHold = 0.8;
    double forkRelease = 0.7;
    double flywheelPower = 0.614;

    public static class var {
        private static int RingStackIndentified = 0;
    }

    public enum Zone {
        A,
        B,
        C
    }

    Zone targetZone = Zone.A;

    public enum State {
        DETECT_RING_STACK,
        NO_RINGS,
        ONE_RING,
        FOUR_RINGS,
        STOP
    }

    @Override
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

        //telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //openCV config
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RingStackDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        );

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        waitForStart();
        currentState = State.DETECT_RING_STACK;

        while (!isStopRequested() && opModeIsActive()) {


            if ((pipeline.position == RingStackDeterminationPipeline.RingPosition.NONE)) {
                telemetry.addLine("Zone A, no rings");
                telemetry.update();
            } else if ((pipeline.position == RingStackDeterminationPipeline.RingPosition.ONE)) {
                telemetry.addLine("Zone B, one ring");
                telemetry.update();
            } else if ((pipeline.position == RingStackDeterminationPipeline.RingPosition.FOUR)) {
                telemetry.addLine("Zone C, four rings");
                telemetry.update();
            }

            switch (currentState) {

                case DETECT_RING_STACK:
                    if ((pipeline.position == RingStackDeterminationPipeline.RingPosition.NONE) && (var.RingStackIndentified == 1)) {
                        targetZone = Zone.A;
                        currentState = State.NO_RINGS;
                    } else if ((pipeline.position == RingStackDeterminationPipeline.RingPosition.ONE) && (var.RingStackIndentified == 1)) {
                        targetZone = Zone.B;
                        currentState = State.ONE_RING;
                    } else if ((pipeline.position == RingStackDeterminationPipeline.RingPosition.FOUR) && (var.RingStackIndentified == 1)) {
                        targetZone = Zone.C;
                        currentState = State.FOUR_RINGS;
                    }
                    break;
                case NO_RINGS:
                    telemetry.addLine("Zone A, no rings");
                    telemetry.addData("Analysis", pipeline.getAnalysis());
                    telemetry.addData("Position", pipeline.position);
                    telemetry.update();
                    //forward to shooting pos whereever the donde that is lol and shoot shoot
                    encoderForwardNoBrake(0.6, 46);
                    encoderForward(0.2, 15);
                    encoderStrafe(0.3, 12);
                    robot.svoMagLift.setPosition(magUp);
                    shootThree(0.6);
                    robot.svoMagLift.setPosition(magDown);

                    // nav to zone a
                    encoderStrafe(0.4, 40);
                    robot.svoWobble.setPosition(wobbleRelease);
                    waitFor(1);
                    //back up to second wobble (no rings so navigate however)

                    //180 spin spin

                    //drop it like its hot

                    //park
                    encoderForward(-0.4, -6);
                    encoderStrafe(-0.4, -28);
                    encoderForward(0.6, 17);
                    currentState = State.STOP;

                    break;
                case ONE_RING:
                    telemetry.addLine("Zone B, one ring");
                    telemetry.addData("Analysis", pipeline.getAnalysis());
                    telemetry.addData("Position", pipeline.position);
                    telemetry.update();
                    //forward to shooting pos donde estas and brrrr
                    encoderForwardNoBrake(0.6, 46);
                    encoderForward(0.2, 15);
                    encoderStrafe(0.2, 12);
                    robot.svoMagLift.setPosition(magUp);
                    shootThree(0.6);
                    robot.svoMagLift.setPosition(magDown);

                    //nav to zone b and rain drop drop top
                    encoderStrafe(0.2, 4);
                    encoderForward(0.6, 21);
                    robot.svoWobble.setPosition(wobbleRelease);
                    waitFor(1);

                    //back up to second wobble

                    //scooch over and forward to the single ring and turn on the intake and yOink

                    //tHEn 180 woosh

                    //drip drop

                    //park
                    encoderForward(-0.6, -5);
                    currentState = State.STOP;
                    break;

                case FOUR_RINGS:
                    telemetry.addLine("Zone C, four rings");
                    telemetry.addData("Analysis", pipeline.getAnalysis());
                    telemetry.addData("Position", pipeline.position);
                    telemetry.update();
                    //forward to shooter babey skrrrAH
                    encoderForwardNoBrake(0.6, 46);
                    encoderForward(0.2, 15);
                    encoderStrafe(0.2, 12);
                    robot.svoMagLift.setPosition(magUp);
                    shootThree(0.6);
                    robot.svoMagLift.setPosition(magDown);

                    //nav to zone c lol
                    encoderStrafe(0.3, 40);
                    encoderForwardNoBrake(0.6, 46);
                    encoderForward(0.6, 6);
                    robot.svoWobble.setPosition(wobbleRelease);
                    waitFor(1);

                    //then back that ass straight tf up along the wall then strafe at the right point to pick up wobble dos

                    //180 babey

                    //zone c again

                    //park
                    encoderForward(-0.6, -35);
                    currentState = State.STOP;
                    break;

                case STOP:
                    brakeMotors();
                    break;

            }

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
        }
    }


    public static class RingStackDeterminationPipeline extends OpenCvPipeline {

        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        static final Scalar BLUE = new Scalar(0, 0, 255, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0, 255);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(210, 126);

        static final int REGION_WIDTH = 25;
        static final int REGION_HEIGHT = 30;

        final int FOUR_RING_THRESHOLD = 170;
        //before: 150
        //182

        final int ONE_RING_THRESHOLD = 149;
        //before: 135
        //156
        //154

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2);

            position = RingPosition.FOUR;
            if (avg1 > FOUR_RING_THRESHOLD) {
                var.RingStackIndentified = 1;
                position = RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                var.RingStackIndentified = 1;
                position = RingPosition.ONE;
            } else {
                var.RingStackIndentified = 1;
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    GREEN,
                    -1);

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
    }

    private void waitFor(double waittime) {
        timer.reset();
        while (timer.seconds() < waittime) {
        }
    }

    private void pushARing() {
        robot.svoRingPush.setPosition(ringPushOut);
        waitFor(0.5);
        robot.svoRingPush.setPosition(ringPushIn);
    }

    private void shootThree(double inBetweenRingTime) {
        robot.mtrFlywheel.setVelocity(normalFlywheelVelocity);
        waitFor(1.4);
        pushARing();
        waitFor(inBetweenRingTime);
        pushARing();
        waitFor(inBetweenRingTime+0.3);
        pushARing();
        waitFor(inBetweenRingTime);
        robot.mtrFlywheel.setPower(0);

    }

    private void resetEncoders() {
        robot.mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void runToPosition() {
        robot.mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void brakeMotors() {
        robot.mtrFL.setPower(0);
        robot.mtrFR.setPower(0);
        robot.mtrBL.setPower(0);
        robot.mtrBR.setPower(0);
    }

    private void mtrFRisBusy() {
        while (robot.mtrFR.isBusy()) {
        }
    }

    private void runWithoutEncoder() {
        robot.mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void forward(double power) {
        robot.mtrBL.setPower(power);
        robot.mtrBR.setPower(power);
        robot.mtrFL.setPower(power);
        robot.mtrFR.setPower(power);


    }

    private void forwardPosition(int distance_inches) {
        robot.mtrBL.setTargetPosition(distance_inches * (int) ticksPerInchCalibrated);
        robot.mtrBR.setTargetPosition(distance_inches * (int) ticksPerInchCalibrated);
        robot.mtrFL.setTargetPosition(distance_inches * (int) ticksPerInchCalibrated);
        robot.mtrFR.setTargetPosition(distance_inches * (int) ticksPerInchCalibrated);
    }

    private void encoderForward(double power, int distance_inches) {
        resetEncoders();
        forwardPosition(-distance_inches);
        runToPosition();
        forward(power);
        mtrFRisBusy();
        brakeMotors();
        runWithoutEncoder();
    }

    private void encoderForwardNoBrake(double power, int distance_inches) {
        resetEncoders();
        forwardPosition(-distance_inches);
        runToPosition();
        forward(power);
        mtrFRisBusy();
        runWithoutEncoder();
    }

    private void strafe(double power) {
        robot.mtrBL.setPower(power);
        robot.mtrBR.setPower(-power);
        robot.mtrFL.setPower(-power);
        robot.mtrFR.setPower(power);
    }

    private void strafePosition(int distance_inches) {
        robot.mtrBL.setTargetPosition(distance_inches * (int) ticksPerInchCalibrated);
        robot.mtrBR.setTargetPosition(-distance_inches * (int) ticksPerInchCalibrated);
        robot.mtrFL.setTargetPosition(-distance_inches * (int) ticksPerInchCalibrated);
        robot.mtrFR.setTargetPosition(distance_inches * (int) ticksPerInchCalibrated);
    }

    private void encoderStrafe(double power, int distance_inches) {
        resetEncoders();
        strafePosition(distance_inches);
        runToPosition();
        strafe(power);
        mtrFRisBusy();
        brakeMotors();
        runWithoutEncoder();
    }
    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / robot.batteryVoltageSensor.getVoltage()
        ));
    }


}
