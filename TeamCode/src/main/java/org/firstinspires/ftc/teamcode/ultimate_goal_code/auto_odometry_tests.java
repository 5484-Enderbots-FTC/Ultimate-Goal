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
    double psFlywheelVelocity = 1200;

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

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    double targetShootHeading = Math.toRadians(5);

    Pose2d targetLeft = new Pose2d(0, -10,targetShootHeading);
    Pose2d targetMiddle = new Pose2d(0, -20,targetShootHeading);
    Pose2d targetRight = new Pose2d(0, -30,targetShootHeading);



    Pose2d againstWall = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            robot.mtrBL.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            robot.mtrBR.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            robot.mtrFL.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            robot.mtrFR.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));

            switch (currentMode) {

                case DRIVER_CONTROL:
                    /***
                     * gotta decide if driver control can live outside of this case or not - i think it can?? since it's constantly updated :P
                     */

                    if (gamepad1.a) {
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }

                case AUTOMATIC_CONTROL:
                    drive.setPoseEstimate(againstWall);

                    Trajectory leftShot = drive.trajectoryBuilder(againstWall)
                            .lineToLinearHeading(targetLeft)
                            .build();
                    Trajectory middleShot = drive.trajectoryBuilder(leftShot.end())
                            .lineToLinearHeading(targetMiddle)
                            .build();
                    Trajectory rightShot = drive.trajectoryBuilder(middleShot.end())
                            .lineToLinearHeading(targetRight)
                            .build();

                    robot.mtrFlywheel.setVelocity(psFlywheelVelocity);
                    robot.svoMagLift.setPosition(magUp);
                    drive.followTrajectory(leftShot);
                    pushARing();
                    drive.followTrajectory(middleShot);
                    pushARing();
                    drive.followTrajectory(rightShot);
                    pushARing();
                    robot.svoMagLift.setPosition(magDown);

                    robot.svoForkHold.setPosition(forkRelease);

                    drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    currentMode = Mode.DRIVER_CONTROL;
                    break;

            }

            if (gamepad1.b && currentMode != Mode.DRIVER_CONTROL) {
                drive.cancelFollowing();
                drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                currentMode = Mode.DRIVER_CONTROL;
            }


        }

    }
    private void pushARing() {
        robot.svoRingPush.setPosition(ringPushOut);
        waitFor(servoMoveTime);
        robot.svoRingPush.setPosition(ringPushIn);
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


















