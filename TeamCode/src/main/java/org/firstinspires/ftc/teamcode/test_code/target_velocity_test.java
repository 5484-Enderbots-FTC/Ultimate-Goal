package org.firstinspires.ftc.teamcode.test_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.test_code.TuningController;

@Config
@Disabled
@TeleOp(name="target velocity test",group="testing")
public class target_velocity_test extends LinearOpMode{
    //Velocity PID
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;

    DcMotorEx mtrFlywheel;

    double normalFlywheelVelocity = 800;
    double psFlywheelVelocity = 600;
    double targetVelo = 0;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(80, 0, 60, 17.5);

    @Override
    public void runOpMode(){

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        mtrFlywheel = hardwareMap.get(DcMotorEx.class, "mtrFlywheel");
        mtrFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        mtrFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFlywheel.setDirection(DcMotorEx.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType = mtrFlywheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        mtrFlywheel.setMotorType(motorConfigurationType);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(mtrFlywheel, MOTOR_VELO_PID);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        waitForStart();
        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            if(gamepad2.a){
                mtrFlywheel.setVelocity(normalFlywheelVelocity);
                targetVelo = normalFlywheelVelocity;

            }
            if (gamepad2.y){
                mtrFlywheel.setVelocity(psFlywheelVelocity);
                targetVelo = psFlywheelVelocity;
            }
            if(gamepad2.b){
                mtrFlywheel.setPower(0);
                targetVelo = 0;
            }
        }
    }
    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }
    private void setTargetVelo(DcMotorEx mtrFlywheel){
//ugh confusing ;-;
    }

    public static double getMotorVelocityF() {
        return 32767 * 60.0 / (TuningController.MOTOR_MAX_RPM * TuningController.MOTOR_TICKS_PER_REV);
    }

}
