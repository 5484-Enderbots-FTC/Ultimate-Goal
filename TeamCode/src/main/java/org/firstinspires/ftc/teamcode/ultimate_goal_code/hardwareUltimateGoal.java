package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class hardwareUltimateGoal {

    DcMotorEx mtrBL, mtrBR, mtrFL, mtrFR, mtrIntake, mtrWobble, mtrFlywheel = null;
    Servo svoWobble, svoMagLift, svoRingPush, svoForkHold = null;
    DigitalChannel topLimit;
    VoltageSensor batteryVoltageSensor;
    OpenCvCamera webcam;
    HardwareMap hwMap = null;

    private static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(100, 0, 30, 18);

    public hardwareUltimateGoal() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

        mtrBL = hwMap.get(DcMotorEx.class, "mtrBL");
        mtrBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrBL.setDirection(DcMotorEx.Direction.REVERSE);

        mtrBR = hwMap.get(DcMotorEx.class, "mtrBR");
        mtrBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrBR.setDirection(DcMotorEx.Direction.FORWARD);

        mtrFL = hwMap.get(DcMotorEx.class, "mtrFL");
        mtrFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrFL.setDirection(DcMotorEx.Direction.REVERSE);

        mtrFR = hwMap.get(DcMotorEx.class, "mtrFR");
        mtrFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrFR.setDirection(DcMotorEx.Direction.FORWARD);

        mtrIntake = hwMap.get(DcMotorEx.class, "mtrIntake");
        mtrIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        mtrIntake.setDirection(DcMotorEx.Direction.FORWARD);

        mtrWobble = hwMap.get(DcMotorEx.class, "mtrWobble");
        mtrWobble.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrWobble.setDirection(DcMotorEx.Direction.FORWARD);

        mtrFlywheel = hwMap.get(DcMotorEx.class, "mtrFlywheel");
        mtrFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        mtrFlywheel.setDirection(DcMotorEx.Direction.REVERSE);

        svoMagLift = hwMap.get(Servo.class, "svoMagLift");
        svoMagLift.setDirection(Servo.Direction.FORWARD);

        svoRingPush = hwMap.get(Servo.class, "svoRingPush");
        svoRingPush.setDirection(Servo.Direction.REVERSE);

        svoWobble = hwMap.get(Servo.class, "svoWobble");
        svoWobble.setDirection(Servo.Direction.FORWARD);

        svoForkHold = hwMap.get(Servo.class, "svoForkHold");
        svoForkHold.setDirection(Servo.Direction.FORWARD);

        topLimit = hwMap.get(DigitalChannel.class, "topLimit");
        topLimit.setMode(DigitalChannel.Mode.INPUT);

        svoRingPush.setPosition(var.ringPushIn);
        svoMagLift.setPosition(var.magDown);
        svoWobble.setPosition(var.wobbleHold);
        svoForkHold.setPosition(var.initFork);

    }
    public void initShooterPID(HardwareMap ahwMap) {
        hwMap = ahwMap;
        //Velocity PID
        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //PID motor config
        mtrFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorConfigurationType motorConfigurationType = mtrFlywheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        mtrFlywheel.setMotorType(motorConfigurationType);

        setPIDFCoefficients(mtrFlywheel, MOTOR_VELO_PID);
    }
    public void initWebcam(HardwareMap ahwMap) {
        hwMap = ahwMap;

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        );
    }
    public void updateDrive(double fwdStick, double bkwdStick, double strStick){
        mtrBL.setPower((fwdStick - bkwdStick + strStick));
        mtrBR.setPower((fwdStick + bkwdStick - strStick));
        mtrFL.setPower((fwdStick - bkwdStick - strStick));
        mtrFR.setPower((fwdStick + bkwdStick + strStick));
    }
    public void updateDrive(double fwdStick, double bkwdStick, double strStick, boolean reversed){
        mtrBL.setPower((-fwdStick + bkwdStick - strStick));
        mtrBR.setPower((-fwdStick - bkwdStick + strStick));
        mtrFL.setPower((-fwdStick + bkwdStick + strStick));
        mtrFR.setPower((-fwdStick - bkwdStick - strStick));
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }
}
