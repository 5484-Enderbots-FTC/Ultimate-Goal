package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.odometry.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ultimate_goal_code.hardwareUltimateGoal;
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

@TeleOp(name = "odometry testing", group = "testing")
public class auto_odometry_tests extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    hardwareUltimateGoal robot = new hardwareUltimateGoal();

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(100, 0, 30, 18);

    double normalFlywheelVelocity = 1350;

    double shootAngleCorrection = 0;

    double timeBetweenShots = 0.4;
    double servoMoveTime = 0.3;
    double flywheelRevUpTime = 0.85;

    double wobbleGoalPower = -0.2;
    double counterWobblePower = -0.15;
    double wobbleDownPower = 0.1;

    double magDown = 0.85;
    double magUp = 0.58;
    double ringPushOut = 0.6;
    double ringPushIn = 0.75;
    double wobbleRelease = 0.3;
    double wobbleHold = 0.19;
    double forkHold = 0.8;
    double forkRelease = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Shooter PID config
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //PID motor config
        robot.mtrFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorConfigurationType motorConfigurationType = robot.mtrFlywheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        robot.mtrFlywheel.setMotorType(motorConfigurationType);

        setPIDFCoefficients(robot.mtrFlywheel, MOTOR_VELO_PID);

        /***
         *               ODOMETRY TRAJECTORIES
         */
        /*

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        Trajectory endTangent0 = drive.trajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(36, 36, Math.toRadians(90)), Math.toRadians(0))
                .build();

        Trajectory endTangent90 = drive.trajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(36, 36, Math.toRadians(90)), Math.toRadians(90))
                .build();

        Trajectory endTangent180 = drive.trajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(36, 36, Math.toRadians(90)), Math.toRadians(180))
                .build();

        Trajectory endTangent270 = drive.trajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(36, 36, Math.toRadians(90)), Math.toRadians(270))
                .build();


         */

        Pose2d wobblePose = new Pose2d(-39, -51, 0);
        drive.setPoseEstimate(wobblePose);
        Trajectory fourRingsToShoot1 = drive.trajectoryBuilder(wobblePose)
                .lineToConstantHeading(
                        new Vector2d(-22, -57)
                )
                .addDisplacementMarker(() ->
                        robot.mtrIntake.setPower(1))
                .build();

        Trajectory fourRingsToShoot2 = drive.trajectoryBuilder(fourRingsToShoot1.end().plus(new Pose2d (0,0, Math.toRadians(90))))
                .lineToConstantHeading(
                        new Vector2d(-22, -40)
                )
                .build();
        Trajectory fourRingsToShoot3 = drive.trajectoryBuilder(fourRingsToShoot2.end().plus(new Pose2d (0,0, Math.toRadians(-90))))
                .lineTo(
                        new Vector2d(-3, -40)
                )
                .build();

        Trajectory fourRingsToShoot4 = drive.trajectoryBuilder(fourRingsToShoot3.end())
                .lineTo(
                        new Vector2d(-3, -30)
                )
                //.addTemporalMarker(1, () ->
                     //   robot.mtrWobble.setPower(counterWobblePower))
                .build();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        waitForStart();


        while(!isStopRequested() && opModeIsActive()){

            robot.mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            robot.mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            robot.mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            robot.mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
/*
            if(gamepad1.a){
                drive.setPoseEstimate(startPose);
                drive.followTrajectory(endTangent0);
            }
            if(gamepad1.b){
                drive.setPoseEstimate(startPose);
                drive.followTrajectory(endTangent90);
            }
            if(gamepad1.y){
                drive.setPoseEstimate(startPose);
                drive.followTrajectory(endTangent180);
            }
            if(gamepad1.x){
                drive.setPoseEstimate(startPose);
                drive.followTrajectory(endTangent270);
            }

 */
            if(gamepad1.a){
                robot.mtrWobble.setPower(-0.4);
                waitFor(0.5);
                robot.mtrWobble.setPower(0);
            }

            if(gamepad1.right_bumper){
                drive.followTrajectory(fourRingsToShoot1);
                drive.turn(Math.toRadians(90));
                drive.followTrajectory(fourRingsToShoot2);
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(fourRingsToShoot3);
                drive.followTrajectory(fourRingsToShoot4);
                waitFor(1);
                robot.svoMagLift.setPosition(magUp);
                shootOne();
                waitFor(0.4);
                shootOne();
                robot.svoMagLift.setPosition(magDown);
                robot.mtrIntake.setPower(0);
                robot.mtrWobble.setPower(0);
                robot.mtrFlywheel.setPower(0);
            }



        }

    }
    private void pushARing() {
        robot.svoRingPush.setPosition(ringPushOut);
        waitFor(servoMoveTime);
        robot.svoRingPush.setPosition(ringPushIn);
    }

    private void shootOne() {
        robot.mtrFlywheel.setVelocity(normalFlywheelVelocity);
        waitFor(flywheelRevUpTime);
        pushARing();
    }

    private void waitFor(double waittime) {
        timer.reset();
        while (timer.seconds() < waittime) {
        }
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / robot.batteryVoltageSensor.getVoltage()
        ));
    }
}


















