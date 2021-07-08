package org.firstinspires.ftc.teamcode.test_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ultimate_goal_code.hardwareUltimateGoal;

import com.qualcomm.hardware.rev.RevColorSensorV3;

@TeleOp(name = "rev color sensor v3", group = "testing")
public class test_rev_color_sensor_v3 extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    hardwareUltimateGoal robot = new hardwareUltimateGoal();
    RevColorSensorV3 color_sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initShooterPID(hardwareMap);

        /**
         * - I2C port
         * - most accurate when positioned within 2cm of the target color
         * - there's a white LED on the sensor that can be turned off with the cream
         *     colored lever on the side of the sensor (but it is quite useful for accurate
         *     color readings, so it's advised to keep it on)
         * - provides proximity sensing, but not accurate (mostly used in the context of is something
         *     right in front of the sensor y/n
         * -
         */

        color_sensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            robot.updateDrive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);


            telemetry.addData("Red: ", color_sensor.red());
            telemetry.addData("Green: ", color_sensor.green());
            telemetry.addData("Blue: ", color_sensor.blue());
            telemetry.addData("light detected: ", color_sensor.getLightDetected());
            telemetry.addData("raw light detected: ", color_sensor.getRawLightDetected());
            telemetry.addData("raw optical: ", color_sensor.rawOptical());
            telemetry.addData("is light on: ", color_sensor.isLightOn());
            telemetry.update();
        }

    }

}