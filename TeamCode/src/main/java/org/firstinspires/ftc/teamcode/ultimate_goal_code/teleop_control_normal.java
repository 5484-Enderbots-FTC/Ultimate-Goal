package org.firstinspires.ftc.teamcode.ultimate_goal_code;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name="teleop normal",group="1-teleop")
public class teleop_control_normal extends LinearOpMode{
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    DcMotorEx mtrBL , mtrBR , mtrFL , mtrFR , mtrIntake, mtrWobble, mtrFlywheel;
    Servo svoWobble, svoMagLift, svoRingPush, svoForkHold;

    BNO055IMU imu;

    double globalAngle, power = .30, correction;
    Orientation lastAngles = new Orientation();
    Orientation angles;
    Acceleration gravity;

    /***

     ~ Constants ~

     */

    double flywheelPower = 0.65;
    double magDown = 0.85;
    double magUp = 0.58;
    double ringPushOut = 0.6;
    double ringPushIn = 0.75;
    double ringJamnt = 0.2;
    double wobbleRelease = 0.37;
    double wobbleHold = 0.2;
    double forkHold = 0.8;
    double forkRelease = 0.7;

    boolean magIsUp = false;
    boolean backwardsMode = false;
    boolean slowMode = false;
    boolean forkHeld = true;
    boolean ringSwiped = false;

    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        mtrBL = hardwareMap.get(DcMotorEx.class, "mtrBL");
        mtrBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrBL.setDirection(DcMotorEx.Direction.REVERSE);

        mtrBR = hardwareMap.get(DcMotorEx.class, "mtrBR");
        mtrBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrBR.setDirection(DcMotorEx.Direction.FORWARD);

        mtrFL = hardwareMap.get(DcMotorEx.class, "mtrFL");
        mtrFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrFL.setDirection(DcMotorEx.Direction.REVERSE);

        mtrFR = hardwareMap.get(DcMotorEx.class, "mtrFR");
        mtrFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrFR.setDirection(DcMotorEx.Direction.FORWARD);

        mtrIntake = hardwareMap.get(DcMotorEx.class, "mtrIntake");
        mtrIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        mtrIntake.setDirection(DcMotorEx.Direction.REVERSE);

        mtrWobble = hardwareMap.get(DcMotorEx.class, "mtrWobble");
        mtrWobble.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrWobble.setDirection(DcMotorEx.Direction.FORWARD);

        mtrFlywheel = hardwareMap.get(DcMotorEx.class, "mtrFlywheel");
        mtrFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        mtrFlywheel.setDirection(DcMotorEx.Direction.REVERSE);

        svoMagLift = hardwareMap.get(Servo.class,"svoMagLift");
        svoMagLift.setDirection(Servo.Direction.FORWARD);

        svoRingPush = hardwareMap.get(Servo.class,"svoRingPush");
        svoRingPush.setDirection(Servo.Direction.REVERSE);

        svoWobble = hardwareMap.get(Servo.class,"svoWobble");
        svoWobble.setDirection(Servo.Direction.FORWARD);

        svoForkHold = hardwareMap.get(Servo.class,"svoForkHold");
        svoForkHold.setDirection(Servo.Direction.FORWARD);

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        svoRingPush.setPosition(ringPushIn);
        svoMagLift.setPosition(magDown);
        svoWobble.setPosition(wobbleHold);
        svoForkHold.setPosition(forkHold);

        waitForStart();
        runtime.reset();
        timer.reset();

        while (!isStopRequested() && opModeIsActive()) {

            /**
             * Gamepad 1 Controls
             */

            if(gamepad1.right_bumper && (backwardsMode == false)){
                //activate backwards mode
                backwardsMode = true;
            }
            else if(gamepad1.right_bumper && (backwardsMode == true)){
                //deactivate backwards mode
                backwardsMode = false;
            }

            if(gamepad1.left_bumper && (slowMode == false)){
                //activate slow mode
                slowMode = true;
            }
            else if(gamepad1.left_bumper && (slowMode == true)){
                //deactivate slow mode
                slowMode = false;
            }

            if((backwardsMode == false) && (slowMode == false)) {
                //default controls
                mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
                mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
                mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
                mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            }
            if((backwardsMode == true) && (slowMode == false)){
                //backwards mode only
                mtrBL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
                mtrBR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
                mtrFL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
                mtrFR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            }
            if((backwardsMode == false) && (slowMode == true)){
                //slow mode only
                mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
                mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
                mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
                mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
            }
            if((backwardsMode == true) && (slowMode == true)){
                //backwards and slow modes together
                mtrBL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
                mtrBR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
                mtrFL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
                mtrFR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
            }

            if (gamepad1.a){
                mtrIntake.setPower(1);
            }
            if (gamepad1.b){
                mtrIntake.setPower(0);
            }
            if (gamepad1.x){
                mtrIntake.setPower(-1);
            }

            /**
             * Gamepad 2 Controls
             */

            mtrWobble.setPower(gamepad2.right_stick_y);

            if (magIsUp){
                mtrIntake.setPower(0);
                if(gamepad2.right_bumper){
                    svoRingPush.setPosition(ringPushOut);
                    waitFor(0.5);
                    svoRingPush.setPosition(ringPushIn);
                }
            }
            if (gamepad2.y && (ringSwiped == false)){
                svoRingPush.setPosition(ringJamnt);
                ringSwiped = true;
            }
            if (gamepad2.y && (ringSwiped == true)){
                svoRingPush.setPosition(ringPushIn);
                ringSwiped = false;
            }

            if(gamepad2.dpad_up){
                svoMagLift.setPosition(magUp);
                mtrFlywheel.setPower(flywheelPower);
                magIsUp = true;
            }
            if(gamepad2.dpad_down){
                mtrIntake.setPower(1);
                svoMagLift.setPosition(magDown);
                mtrFlywheel.setPower(0);
                magIsUp = false;
            }

            if(gamepad2.a){
                mtrFlywheel.setPower(flywheelPower);
            }
            if(gamepad2.b){
                mtrFlywheel.setPower(0);
            }

            if(gamepad2.left_bumper && (forkHeld == false)){
                svoForkHold.setPosition(forkHold);
                forkHeld = true;
            }
            else if(gamepad2.left_bumper && (forkHeld == true)){
                svoForkHold.setPosition(forkRelease);
                forkHeld = false;
            }

            /**
             * Telemetry
             */

            telemetry.addData("Status", " Run Time: " + runtime.toString());
            telemetry.addData("Timer Status", " Time: " + timer.toString());
            telemetry.addData("backwardsMode status:", " " + backwardsMode);
            telemetry.addData("slowMode status:", " " + slowMode);
            telemetry.addData("forkHeld status:", " " + forkHeld);
            telemetry.update();
        }


    }
    private void waitFor(double waittime) {
        timer.reset();
        while (timer.seconds() < waittime) {
            //pls keep driving lol
            mtrBL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            mtrBR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            mtrFR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
        }
    }
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        mtrFL.setPower(leftPower);
        mtrBL.setPower(leftPower);
        mtrBR.setPower(rightPower);
        mtrFR.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() > degrees) {
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        mtrBL.setPower(0);
        mtrBR.setPower(0);
        mtrFR.setPower(0);
        mtrFL.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}


