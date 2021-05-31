package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.odometry.drive.SampleMecanumDrive;


@TeleOp(name = "odometry testing", group = "testing")
public class auto_odometry_tests extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    hardwareUltimateGoal robot = new hardwareUltimateGoal();

    enum Mode {
        MOVE_SERVO_START,
        MOVE_SERVO_WAIT,
        MOVE_AGAIN,
        MOVE_WAIT,
        MOVE_BACK
    }

    Mode currentMode = Mode.MOVE_SERVO_START;

    double targetShootHeading = Math.toRadians(5);

    Pose2d targetLeft = new Pose2d(0, -15,targetShootHeading);
    Pose2d targetMiddle = new Pose2d(0, -30,targetShootHeading);
    Pose2d targetRight = new Pose2d(0, -45,targetShootHeading);

    Pose2d againstWall = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.initShooterPID(hardwareMap);

        /***
         *               ODOMETRY TRAJECTORIES
         */

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            robot.updateDrive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,true);

            switch (currentMode){
                case MOVE_SERVO_START:
                    if(gamepad1.x){
                        timer.reset();
                        robot.svoMagLift.setPosition(var.magUp);
                        currentMode = Mode.MOVE_SERVO_WAIT;
                    }
                    break;
                case MOVE_SERVO_WAIT:
                    if(timer.seconds() > 0.5){
                        currentMode = Mode.MOVE_AGAIN;
                    }
                    break;
                case MOVE_AGAIN:
                    robot.svoRingPush.setPosition(var.ringPushOut);
                    timer.reset();
                    currentMode = Mode.MOVE_WAIT;
                    break;
                case MOVE_WAIT:
                    if(timer.seconds() > 1){
                        robot.svoRingPush.setPosition(var.ringPushIn);
                        currentMode = Mode.MOVE_BACK;
                    }
                    break;
                case MOVE_BACK:
                    robot.svoMagLift.setPosition(var.magDown);
                    currentMode = Mode.MOVE_SERVO_START;
                    break;
                default:
                    currentMode = Mode.MOVE_SERVO_START;
                    break;

            }
            if(gamepad1.b && currentMode != Mode.MOVE_SERVO_START){
                currentMode = Mode.MOVE_SERVO_START;
            }

        }

    }
    private void pushARing() {
        robot.svoRingPush.setPosition(var.ringPushOut);
        waitFor(var.servoMoveTime);
        robot.svoRingPush.setPosition(var.ringPushIn);
    }

    private void waitFor(double waittime) {
        timer.reset();
        while (timer.seconds() < waittime) {
        }
    }
}


















