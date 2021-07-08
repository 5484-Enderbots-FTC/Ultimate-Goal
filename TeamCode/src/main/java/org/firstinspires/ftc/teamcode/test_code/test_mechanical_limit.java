package org.firstinspires.ftc.teamcode.test_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ultimate_goal_code.hardwareUltimateGoal;


@TeleOp(name = "mechanial limit switch", group = "testing")
public class test_mechanical_limit extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    hardwareUltimateGoal robot = new hardwareUltimateGoal();

    TouchSensor limit;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initShooterPID(hardwareMap);

        /**
         * - Digital port
         * - 
         */

        limit = hardwareMap.get(TouchSensor.class, "mech_limit");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            robot.updateDrive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);

            telemetry.addData("Is limit pressed: ", limit.isPressed());
            telemetry.update();
        }

    }

}


















