package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Foundation Only", group = "Concept")
public class simple_auto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    //motors
    DcMotor mtrBL = null, mtrBR = null, mtrFL = null, mtrFR = null, mtrIntake = null;

    int strafeSwapper = 1;

    private final double ticksPerMm = 1.68240559922;
    private final double ticksPerMmCalibrated = 1.518268;

    int park = (int)(245*ticksPerMmCalibrated);

    @Override
    public void runOpMode() {
        //configuration of robot stuff

        //motors
        mtrFR = hardwareMap.get(DcMotor.class, "rightFront_drive");
        mtrFR.setDirection(DcMotor.Direction.REVERSE);

        mtrFL = hardwareMap.get(DcMotor.class, "leftFront_drive");
        mtrFL.setDirection(DcMotor.Direction.FORWARD);

        mtrBL = hardwareMap.get(DcMotor.class, "leftBack_drive");
        mtrBL.setDirection(DcMotor.Direction.FORWARD);

        mtrBR = hardwareMap.get(DcMotor.class, "rightBack_drive");
        mtrBR.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        /**

         Code begins here

         */


        waitForStart();
        runtime.reset();

        encoderForward(0.5,park);

        /**

         End of Program

         */

    }

    private void waitFor(double waittime) {
        runtime.reset();
        while (runtime.time() < waittime) {
        }
    }
    private void resetEncoders() {
        mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void runToPosition() {
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void brakeMotors() {
        mtrFL.setPower(0);
        mtrFR.setPower(0);
        mtrBL.setPower(0);
        mtrBR.setPower(0);
    }
    private void mtrFRisBusy() {
        while (mtrFR.isBusy()){
        }
    }
    private void mtrBLisBusy() {
        while (mtrBL.isBusy()){
        }
    }
    private void runWithoutEncoder() {
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    private void forward(double power) {
        mtrFR.setPower(power);
        mtrFL.setPower(power);
        mtrBL.setPower(power);
        mtrBR.setPower(power);
    }
    private void forwardPosition(int position) {
        mtrFR.setTargetPosition(position);
        mtrFL.setTargetPosition(position);
        mtrBR.setTargetPosition(position);
        mtrBL.setTargetPosition(position);
    }
    private void encoderForward(double power, int position){
        resetEncoders();
        forwardPosition(position);
        runToPosition();
        forward(power);
        mtrFRisBusy();
        brakeMotors();
        runWithoutEncoder();
    }

    private void strafe(double power) {
        mtrFR.setPower(-power * strafeSwapper);
        mtrFL.setPower(power * strafeSwapper);
        mtrBL.setPower(-power * strafeSwapper);
        mtrBR.setPower(power * strafeSwapper);
    }
    private void strafePosition(int position){
        mtrFR.setTargetPosition(-position * strafeSwapper);
        mtrFL.setTargetPosition(position * strafeSwapper);
        mtrBR.setTargetPosition(position * strafeSwapper);
        mtrBL.setTargetPosition(-position * strafeSwapper);
    }
    private void encoderStrafe(double power, int position){
        resetEncoders();
        strafePosition(position);
        runToPosition();
        strafe(power);
        mtrFRisBusy();
        brakeMotors();
        runWithoutEncoder();
    }

}
