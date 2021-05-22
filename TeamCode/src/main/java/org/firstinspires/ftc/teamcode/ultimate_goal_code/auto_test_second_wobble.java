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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Autonomous(name = "wobble testing", group = "testing")
public class auto_test_second_wobble extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime wobbleTimer = new ElapsedTime();

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
    double forkHold = 0.75;
    double initFork = 0.95;
    double forkRelease = 0.5;
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
        LIFT_UP,
        STOP
    }

    @Override
    public void runOpMode() throws InterruptedException {

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

        while (!isStopRequested() && opModeIsActive()) {

            robot.svoForkHold.setPosition(forkRelease);
            //encoderForward(0.4,5);
            //waitFor(0.5);
            //encoderForward(-0.4,-10);

            robot.mtrWobble.setPower(0.2);
            Thread.sleep(1000);

            //wobbleUp(1,-0.2);
            //robot.mtrWobble.setPower(0.05);

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

    private void wobbleUp(double seconds, double power){
        currentState = State.LIFT_UP;

        switch (currentState) {

            case LIFT_UP:

                wobbleTimer.reset();
                if(wobbleTimer.seconds() < seconds){
                    robot.mtrWobble.setPower(power);
                }
                else{
                    currentState = State.STOP;
                }
                telemetry.addData("Status", "WobbleTimer: " + wobbleTimer.toString());
                telemetry.update();
                break;
            case STOP:
                robot.mtrWobble.setPower(0);
                break;
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
