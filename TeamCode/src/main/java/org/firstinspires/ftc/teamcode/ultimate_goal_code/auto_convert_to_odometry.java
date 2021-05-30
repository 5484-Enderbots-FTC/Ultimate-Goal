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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.odometry.drive.SampleMecanumDrive;
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

@Autonomous(name = "odometry pog", group = "auto")
public class auto_convert_to_odometry extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    //OpenCV stuff
    RingStackDeterminationPipeline pipeline;

    //motors
    hardwareUltimateGoal robot = new hardwareUltimateGoal();

    State currentState;

    //constants
    private final double ticksPerInchCalibrated = 43.3305;

    double shootCorrection = 5;
    double shootAngleCorrection = 8;

    double flywheelRevUpTime = 0.85;

    double wobbleGoalPower = -0.4;
    double counterWobblePower = -0.15;
    double wobbleDownPower = 0.1;
    double wobbleTime = 0.25;



    public static class vars {
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

    Vector2d shootingPosition = new Vector2d(-3,-30);

    Vector2d noRingWobblePickUp = new Vector2d(-39,-49.5);
    Vector2d oneRingWobblePickUp = new Vector2d(-39,-49.5);
    Vector2d fourRingWobblePickUp = new Vector2d(-39,-49.75);

    Vector2d park01 = new Vector2d(9,-35);
    Vector2d park4 = new Vector2d(9,-55);

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);
        robot.initShooterPID(hardwareMap);
        robot.initWebcam(hardwareMap);
        
        pipeline = new RingStackDeterminationPipeline();
        robot.webcam.setPipeline(pipeline);

        /***
         *               ODOMETRY TRAJECTORIES
         */
        Pose2d startPose = new Pose2d(-63, -14, 0);
        drive.setPoseEstimate(startPose);

        //shooting trajectory for A lol:
        Trajectory toShootA = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(
                        shootingPosition, Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        //shoot to go around ring stack lmao:
        Trajectory toShootBC = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(
                        new Vector2d(1, -4), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        shootingPosition, Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();


        /***
         * NO RING
         */
        Trajectory noRingToA = drive.trajectoryBuilder(toShootA.end())
                .strafeTo(
                        new Vector2d(4, -50)
                )
                .build();


        Trajectory noRingToWobble = drive.trajectoryBuilder(noRingToA.end(), true)
                .lineToConstantHeading(
                        noRingWobblePickUp,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();


        Trajectory noRingToA2 = drive.trajectoryBuilder(noRingToWobble.end())
                .splineToLinearHeading(
                        new Pose2d(-3, -55, Math.toRadians(180) + 1e-6), Math.toRadians(0))
                .addTemporalMarker(1, () ->
                        robot.mtrWobble.setPower(counterWobblePower))
                .build();


        Trajectory noRingToPark = drive.trajectoryBuilder(noRingToA2.end())
                .splineToConstantHeading(
                        new Vector2d(-18, -55), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-18, -35), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        park01, Math.toRadians(0)
                )
                .build();


        /***
         * ONE RING
         */

        //spline to align with B
        Trajectory oneRingToB = drive.trajectoryBuilder(toShootBC.end())
                .lineToConstantHeading(new Vector2d(26, -27))
                .build();


        Trajectory oneRingToWobble1 = drive.trajectoryBuilder(oneRingToB.end())
                .lineToConstantHeading(
                        new Vector2d(20, -49.5),
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory oneRingToWobble2 = drive.trajectoryBuilder(oneRingToWobble1.end(), true)
                .lineToConstantHeading(
                        new Vector2d(-8, -49.5),
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory oneRingToWobble3 = drive.trajectoryBuilder(oneRingToWobble2.end(), true)
                .lineToConstantHeading(
                        oneRingWobblePickUp,
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();


        Trajectory oneRingToShoot = drive.trajectoryBuilder(oneRingToWobble3.end())
                //go pick up da ring!!!
                .addDisplacementMarker(() -> {
                    robot.mtrIntake.setPower(1);
                })
                .splineToConstantHeading(
                        new Vector2d(-39, -33), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-18, -33), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        shootingPosition, Math.toRadians(0)
                )
                .addDisplacementMarker(() -> {
                    drive.turn(Math.toRadians(shootCorrection));
                    waitFor(1);
                    robot.svoMagLift.setPosition(var.magUp);
                    shootOne();
                    robot.svoMagLift.setPosition(var.magDown);
                })
                .addTemporalMarker(1, () ->
                        robot.mtrWobble.setPower(counterWobblePower))
                .build();


        Trajectory oneRingToB2 = drive.trajectoryBuilder(oneRingToShoot.end())
                //then turn around to slap da wobble down umu
                .splineToLinearHeading(
                        new Pose2d(18, -30, Math.toRadians(180) + 1e-6), Math.toRadians(0))
                .build();


        Trajectory oneRingToPark = drive.trajectoryBuilder(oneRingToB2.end())
                .lineToConstantHeading(
                        park01
                )

                .build();

        /***
         * FOUR RINGS
         */

        //spline to align with C
        Trajectory fourRingsToC = drive.trajectoryBuilder(toShootBC.end().plus(new Pose2d(0, 0, Math.toRadians(shootAngleCorrection))))
                .splineToConstantHeading(
                        new Vector2d(50, -47), Math.toRadians(0)
                )
                .build();

        Trajectory fourRingsToWobble1 = drive.trajectoryBuilder(fourRingsToC.end())
                .lineToConstantHeading(
                        new Vector2d(-20, -49.75)
                )
                .build();
        Trajectory fourRingsToWobble2 = drive.trajectoryBuilder(fourRingsToWobble1.end())
                .addDisplacementMarker(() ->
                        robot.mtrIntake.setPower(1))
                .lineToConstantHeading(
                        fourRingWobblePickUp,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory fourRingsToShoot10 = drive.trajectoryBuilder(fourRingsToWobble2.end().plus(new Pose2d (0,0, Math.toRadians(60))))

                .lineToConstantHeading(
                        new Vector2d(-27, -44)
                )
                .build();
        Trajectory fourRingsToShoot11 = drive.trajectoryBuilder(fourRingsToShoot10.end())
                .lineToConstantHeading(
                        new Vector2d(-25, -42)
                )
                .build();
        Trajectory fourRingsToShoot12 = drive.trajectoryBuilder(fourRingsToShoot11.end().plus(new Pose2d (0,0, Math.toRadians(-60))))
                .lineToConstantHeading(
                        new Vector2d(-3, -41)
                )
                .build();



        /***
         * here we go code for ring stack and park
         */

        Trajectory fourRingsToC2 = drive.trajectoryBuilder(fourRingsToShoot12.end().plus(new Pose2d (0,0, Math.toRadians(-179.9))),true)
                //then turn around to slap da wobble down umu
                .lineToConstantHeading(
                        new Vector2d(44, -54)
                )
                .build();


        Trajectory fourRingsToPark = drive.trajectoryBuilder(fourRingsToC2.end())
                .lineToConstantHeading(
                        park4
                )

                .build();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        waitForStart();
        currentState = State.DETECT_RING_STACK;

        while (!isStopRequested() && opModeIsActive()) {

            if ((pipeline.position == RingStackDeterminationPipeline.RingPosition.NONE)) {
                telemetry.addData("Analysis", pipeline.getAnalysis());
                telemetry.addLine("Zone A, no rings");
                telemetry.update();
            } else if ((pipeline.position == RingStackDeterminationPipeline.RingPosition.ONE)) {
                telemetry.addData("Analysis", pipeline.getAnalysis());
                telemetry.addLine("Zone B, one ring");
                telemetry.update();
            } else if ((pipeline.position == RingStackDeterminationPipeline.RingPosition.FOUR)) {
                telemetry.addData("Analysis", pipeline.getAnalysis());
                telemetry.addLine("Zone C, four rings");
                telemetry.update();
            }


            if(isStopRequested()){
                if(drive.isBusy()){
                    drive.cancelFollowing();
                }
                currentState = State.STOP;
            }

            switch (currentState) {

                case DETECT_RING_STACK:
                    if ((pipeline.position == RingStackDeterminationPipeline.RingPosition.NONE) && (vars.RingStackIndentified == 1)) {
                        targetZone = Zone.A;
                        telemetry.addLine("Zone A, no rings");
                        telemetry.addData("Analysis", pipeline.getAnalysis());
                        telemetry.addData("Position", pipeline.position);
                        telemetry.update();
                        currentState = State.NO_RINGS;
                    } else if ((pipeline.position == RingStackDeterminationPipeline.RingPosition.ONE) && (vars.RingStackIndentified == 1)) {
                        targetZone = Zone.B;
                        telemetry.addLine("Zone B, one ring");
                        telemetry.addData("Analysis", pipeline.getAnalysis());
                        telemetry.addData("Position", pipeline.position);
                        telemetry.update();
                        currentState = State.ONE_RING;
                    } else if ((pipeline.position == RingStackDeterminationPipeline.RingPosition.FOUR) && (vars.RingStackIndentified == 1)) {
                        targetZone = Zone.C;
                        telemetry.addLine("Zone C, four rings");
                        telemetry.addData("Analysis", pipeline.getAnalysis());
                        telemetry.addData("Position", pipeline.position);
                        telemetry.update();
                        currentState = State.FOUR_RINGS;
                    }
                    break;
                case NO_RINGS:
                    if (!isStopRequested()) {
                        telemetry.addLine("Zone A, no rings");
                        telemetry.addData("Analysis", pipeline.getAnalysis());
                        telemetry.addData("Position", pipeline.position);
                        telemetry.update();

                        //go to shoot
                        drive.followTrajectory(toShootA);

                        //correct the shoot angle
                        drive.turn(Math.toRadians(shootCorrection));

                        //shoot
                        robot.svoMagLift.setPosition(var.magUp);
                        shootThree();
                        robot.svoMagLift.setPosition(var.magDown);

                        //undo correction
                        drive.turn(Math.toRadians(-shootCorrection));

                        //go to zone A
                        robot.svoForkHold.setPosition(var.forkRelease);
                        drive.followTrajectory(noRingToA);

                        //bye bye wobble
                        robot.svoWobble.setPosition(var.wobbleRelease);
                        waitFor(1);

                        //BACKWARDS trajectory tiem :D
                        drive.followTrajectory(noRingToWobble);

                        pickUpWobble();

                        //hold da wobble while driving to spot with fancy spin move
                        drive.followTrajectory(noRingToA2);

                        //dROP
                        dropWobble();
                        robot.mtrWobble.setPower(0);

                        //el parque :DDD
                        drive.followTrajectory(noRingToPark);

                        //stahp
                        currentState = State.STOP;
                    }
                    break;
                case ONE_RING:
                    if (!isStopRequested()) {
                        telemetry.addLine("Zone B, one ring");
                        telemetry.addData("Analysis", pipeline.getAnalysis());
                        telemetry.addData("Position", pipeline.position);
                        telemetry.update();

                        //go to shoot
                        drive.followTrajectory(toShootBC);

                        drive.turn(Math.toRadians(shootCorrection));

                        //shoot
                        robot.svoMagLift.setPosition(var.magUp);
                        shootThree();
                        robot.svoMagLift.setPosition(var.magDown);

                        //undo correction
                        drive.turn(Math.toRadians(-shootCorrection));

                        //nav to zone b
                        robot.svoForkHold.setPosition(var.forkRelease);
                        drive.followTrajectory(oneRingToB);

                        //raindrop drop top
                        robot.svoWobble.setPosition(var.wobbleRelease);
                        waitFor(1);

                        //BACKWARDS to second wobble tiem TWO :DDD
                        drive.followTrajectory(oneRingToWobble1);
                        drive.followTrajectory(oneRingToWobble2);
                        drive.followTrajectory(oneRingToWobble3);

                        pickUpWobble();

                        //scooch over and forward to the single ring and turn on the intake and yOink
                        drive.followTrajectory(oneRingToShoot);
                        //undo correction
                        drive.turn(Math.toRadians(-shootCorrection));

                        //tHEn 180 woosh
                        drive.followTrajectory(oneRingToB2);

                        //drip drop
                        dropWobble();
                        robot.mtrWobble.setPower(0);

                        //park
                        drive.followTrajectory(oneRingToPark);
                        currentState = State.STOP;
                    }
                    break;

                case FOUR_RINGS:
                    if (!isStopRequested()) {
                        telemetry.addLine("Zone C, four rings");
                        telemetry.addData("Analysis", pipeline.getAnalysis());
                        telemetry.addData("Position", pipeline.position);
                        telemetry.update();


                        //go to shoot
                        drive.followTrajectory(toShootBC);

                        drive.turn(Math.toRadians(shootCorrection));

                        //shoot
                        robot.svoMagLift.setPosition(var.magUp);
                        shootThree();
                        robot.svoMagLift.setPosition(var.magDown);

                        //undo correction
                        drive.turn(Math.toRadians(-shootCorrection));
                        waitFor(0.3);

                        //nav to zone C
                        robot.svoForkHold.setPosition(var.forkRelease);
                        drive.followTrajectory(fourRingsToC);
                        drive.turn(Math.toRadians(-6));

                        //bye bye wobble
                        robot.svoWobble.setPosition(var.wobbleRelease);
                        waitFor(1);

                        //go go power rangers (to wobble dos)
                        drive.followTrajectory(fourRingsToWobble1);
                        drive.followTrajectory(fourRingsToWobble2);

                        pickUpWobble();

                        /***
                         * ok so this is the code for if i actually do the ring stack and through to parking
                         */
                        //smack da stacc and yoink den shoot
                        robot.mtrWobble.setPower(counterWobblePower);
                        drive.turn(Math.toRadians(60));
                        drive.followTrajectory(fourRingsToShoot10);
                        waitFor(0.7);
                        drive.followTrajectory(fourRingsToShoot11);
                        drive.turn(Math.toRadians(-60));
                        drive.followTrajectory(fourRingsToShoot12);

                        //turn for shoot
                        drive.turn(Math.toRadians(8));
                        robot.svoMagLift.setPosition(var.magUp);
                        shootTwo();
                        robot.svoMagLift.setPosition(var.magDown);
                        drive.turn(Math.toRadians(-8));
                        drive.turn(Math.toRadians(-179.9));
                        robot.mtrIntake.setPower(0);

                        //deposit da wobbly boi
                        drive.followTrajectory(fourRingsToC2);

                        dropWobble();
                        robot.mtrWobble.setPower(0);

                        //parque
                        drive.followTrajectory(fourRingsToPark);

                        currentState = State.STOP;
                    }
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

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(210, 124);
        /**
         * (0,0).--------------x--------------> x = 320
         * |                                .
         * |                                .
         * y                                .
         * |                                .
         * |                                .
         * \/                                .
         * y = 240 . . . . . . . . . . . . . . .
         */

        static final int REGION_WIDTH = 25;
        static final int REGION_HEIGHT = 30;

        final int FOUR_RING_THRESHOLD = 168;
        //170 actual field
        //140 ben's

        final int ONE_RING_THRESHOLD = 140;
        //150 actual
        //125 ben's

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
                vars.RingStackIndentified = 1;
                position = RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                vars.RingStackIndentified = 1;
                position = RingPosition.ONE;
            } else {
                vars.RingStackIndentified = 1;
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
        robot.svoRingPush.setPosition(var.ringPushOut);
        waitFor(var.servoMoveTime);
        robot.svoRingPush.setPosition(var.ringPushIn);
    }

    private void shootThree() {
        robot.mtrFlywheel.setVelocity(var.normalFlywheelVelocity);
        waitFor(flywheelRevUpTime+0.1);
        pushARing();
        waitFor(var.timeBetweenShots);
        pushARing();
        waitFor(var.timeBetweenShots);
        pushARing();
        waitFor(var.timeBetweenShots);
        robot.mtrFlywheel.setPower(0);

    }

    private void shootTwo() {
        robot.mtrFlywheel.setVelocity(var.normalFlywheelVelocity);
        waitFor(flywheelRevUpTime+0.2);
        pushARing();
        waitFor(var.timeBetweenShots);
        pushARing();
        waitFor(var.timeBetweenShots);
        robot.mtrFlywheel.setPower(0);
    }

    private void shootOne() {
        robot.mtrFlywheel.setVelocity(var.normalFlywheelVelocity);
        waitFor(flywheelRevUpTime+0.1);
        pushARing();
        robot.mtrFlywheel.setPower(0);
    }

    private void pickUpWobble(){
        robot.mtrWobble.setPower(wobbleGoalPower);
        waitFor(wobbleTime);
    }

    private void dropWobble(){
        robot.mtrWobble.setPower(wobbleDownPower);
        waitFor(1);
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

}
