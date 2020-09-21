package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="drive",group="Linear Opmode")

public class drive_maybe extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();

    DcMotor mtrBL , mtrBR , mtrFL , mtrFR , mtrLift;
    Servo extend , grab , arm , svoFR , svoFL;
    TouchSensor topLimit , bottomLimit;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }


    }


}
