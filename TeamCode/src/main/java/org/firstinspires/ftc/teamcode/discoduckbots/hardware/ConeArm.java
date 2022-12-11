package org.firstinspires.ftc.teamcode.discoduckbots.hardware;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class ConeArm {

    private final LinearOpMode opMode;
    private DcMotor coneLift;
    private Servo coneGrabber;
    private DcMotor coneTurret;
    private boolean resetInProgress = false;
    private boolean isClosed = true;
    private boolean buttonPress = false;

    public ConeArm(DcMotor coneLift, Servo coneGrabber, DcMotor coneTurret, LinearOpMode opMode) {
        this.coneLift = coneLift;
        this.coneGrabber = coneGrabber;
        this.coneTurret = coneTurret;
        this.opMode = opMode;
        coneLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        coneLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        coneLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void drop(LinearOpMode opmode) {
        coneLift.setPower(-0.5);
        opmode.sleep(2000);
        coneLift.setPower(0);
        open();
    }


    public void dropByEncoder(int revolutions){
        coneLift.setDirection(DcMotorSimple.Direction.REVERSE);
        coneLift.setTargetPosition(coneLift.getCurrentPosition() + revolutions);
        coneLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        /* while (coneLift.getTargetPosition() > coneLift.getCurrentPosition()){
            coneLift.setPower(0.5);
        } */
        coneLift.setPower(0.0);
        open();
    }

    public void resetToLydiasFavoritePosition() {
        resetToLydiasFavoritePosition(540);
    }
    public void resetArmTeleop() {
        resetToLydiasFavoritePosition(0);
    }

    public void resetGrabberAsync() { resetArmAsync(540);}
    public void resetGrabberAsyncTeleop() {
        resetArmAsync(0);
    }

    public void resetArmAsync(int armPosition) {
        if(!resetInProgress) {
            resetInProgress = true;
            release();

            //cargoMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            coneLift.setTargetPosition(armPosition);
            coneLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //cargoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            coneLift.setPower(-0.75);
            Log.d("ftc-reset", "exiting resetArm");
        } else {
            Log.d("ftc-reset", "ignoring reset as in progress");
        }
    }

    public void liftByEncoderAsync(int revolutions) {
        //if (!resetInProgress) {
         //   resetInProgress = true;

            coneLift.setDirection(DcMotorSimple.Direction.FORWARD);
            coneLift.setTargetPosition( revolutions);
            coneLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //cargoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            coneLift.setPower(0.5);
        //}
    }


    public void stopIfNotBusy() {
        Log.d("ftc-reset", "curr pos: " +  coneLift.getCurrentPosition());
        if ( resetInProgress ) {
            if (coneLift.getCurrentPosition() <= 0) {
                Log.d("ftc-reset", "cargoMotor stopping async ");
                coneLift.setPower(0.0);
                coneLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                resetInProgress = false;
            } else {
                Log.d("ftc-reset", "cargoMotor continuing power ");
                //cargoMotor.setPower(-0.75);
            }
        } else {
            coneLift.setPower(0);
        }

    }
    public void resetToLydiasFavoritePosition(int position){
        release();

        //cargoMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        coneLift.setTargetPosition(position);
        coneLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       // cargoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coneLift.setPower(-0.75);


        while(coneLift.isBusy()) {
            Log.d("ftc", "cargoMotor in loop " + coneLift.getCurrentPosition());
        }
        Log.d("ftc", "cargoMotor out of loop ");
        coneLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        coneLift.setPower(0.0);

    }

    public void lowerByEncoder(int revolutions){

        coneLift.setDirection(DcMotorSimple.Direction.REVERSE);
        coneLift.setTargetPosition(coneLift.getCurrentPosition() + revolutions);
        coneLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //coneLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //cargoMotor.setPower(0.5);
        while (coneLift.getTargetPosition() > coneLift.getCurrentPosition()){
            coneLift.setPower(0.5);
            Log.d("FTC-Arm", "c: " +
                    coneLift.getCurrentPosition() + " t " + coneLift.getTargetPosition());
        }
        //cargoMotor.setPower(0.5);
       /* while(cargoMotor.isBusy()) {

        }*/
        coneLift.setPower(0.0);

    }


    public void grabAndLiftByEncoder(int revolutions, LinearOpMode opMode){
        release();
        opMode.sleep(500);
        coneLift.setDirection(DcMotorSimple.Direction.FORWARD);
        coneLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coneLift.setTargetPosition(coneLift.getCurrentPosition() + revolutions);

        while (coneLift.getTargetPosition() > coneLift.getCurrentPosition()){
            coneLift.setPower(0.5);
        }

        coneLift.setPower(0.0);

    }
/*
    public void liftByEncoder(int revolutions){

        cargoMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        cargoMotor.setTargetPosition(cargoMotor.getCurrentPosition() + revolutions);
        //cargoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cargoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //cargoMotor.setPower(0.5);
        while (cargoMotor.getTargetPosition() > cargoMotor.getCurrentPosition()){
            cargoMotor.setPower(0.5);
            Log.d("FTC-Arm", "c: " +
                    cargoMotor.getCurrentPosition() + " t " + cargoMotor.getTargetPosition());
        }

        cargoMotor.setPower(0.0);

    }
*/
    public void liftByEncoder(int revolutions){
        Log.d("LIFT", "pos: " + coneLift.getCurrentPosition());
        coneLift.setDirection(DcMotorSimple.Direction.FORWARD);
        coneLift.setTargetPosition(revolutions);
        coneLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //cargoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coneLift.setPower(0.75);

      /* while(coneLift.isBusy() && opMode.opModeIsActive()) {
           Log.d("LIFT", "pos: " + coneLift.getCurrentPosition());

        } */
    }

    public void liftInch(LinearOpMode opmode) {
        coneLift.setPower(0.5);
        opmode.sleep(325);
        coneLift.setPower(0);
    }
    public void dropLift(LinearOpMode opmode) {
        coneLift.setPower(-0.5);
        opmode.sleep(2000);
        coneLift.setPower(0);
        open();
        coneLift.setPower(0.5);
        opmode.sleep(1000);
        coneLift.setPower(0);
    }
    public void resetPositionAs0 () {
        coneLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void stopLiftEncoder() {
        coneLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        coneLift.setTargetPosition(0);
        coneLift.setPower(0.5);
    }
    public void holdPosition() {
        coneLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        coneLift.setTargetPosition(coneLift.getCurrentPosition());
        coneLift.setPower(0.5);
    }
    public void lower(double speed) {
        coneLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coneLift.setPower(-1 * speed);
    }

    public void lift(double speed) {
        coneLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coneLift.setPower(speed);
    }

    public void stop() {
        coneLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coneLift.setPower(0);
    }

    public void release() {
        //coneGrabber.setDirection(Servo.Direction.REVERSE);
        coneGrabber.setPosition(0);
    }

    public void open() {
        //coneGrabber.setDirection(Servo.Direction.REVERSE);
        coneGrabber.setPosition(0.50);
    }

    public void close() {
        //coneGrabber.setDirection(Servo.Direction.REVERSE);
        coneGrabber.setPosition(1);
    }

    public void onPress() {
        if (buttonPress) return;
        buttonPress = true;
        if (isClosed) {
            isClosed = false;
            open();
        }
        else {
            isClosed = true;
            close();
        }
    }

    public void onRelease() {
        buttonPress = false;
    }

    public double printServoValue(){
        return coneGrabber.getPosition();
    }

    public void pivotRight() {
        coneTurret.setPower(0.5);
    }

    public void pivotLeft() {
        coneTurret.setPower(-0.5);
    }

    public void stopPivot() {
        coneTurret.setPower(0.0);
    }
}
