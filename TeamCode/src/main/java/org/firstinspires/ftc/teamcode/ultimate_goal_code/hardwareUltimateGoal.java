package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class hardwareUltimateGoal {

    DcMotorEx mtrBL, mtrBR, mtrFL, mtrFR, mtrIntake, mtrWobble, mtrFlywheel = null;
    Servo svoWobble, svoMagLift, svoRingPush, svoForkHold = null;
    TouchSensor topLimit;
    VoltageSensor batteryVoltageSensor;
    HardwareMap hwMap = null;

    double ringPushIn = 0.75;
    double magDown = 0.85;
    double wobbleHold = 0.2;
    double forkHold = 0.95;

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

        topLimit = hwMap.get(TouchSensor.class, "topLimit");

        svoRingPush.setPosition(ringPushIn);
        svoMagLift.setPosition(magDown);
        svoWobble.setPosition(wobbleHold);
        svoForkHold.setPosition(forkHold);

    }
}
