/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.discoduckbots.opmode.powerPlay;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.discoduckbots.hardware.ConeArm;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Odometry Teleop", group="Linear Opmode")
public class MecanumOdometryTeleOp extends LinearOpMode {

    private static double THROTTLE = 0.7;
    private static double STRAFE_THROTTLE = 0.7;
    private static double TURN_THROTTLE = 0.7;
    private static double intakeSpeed = .81;
    private static final double ARM_SPEED = 1;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        DcMotor coneLift = hardwareStore.getConeLift();
        DcMotor coneTurret = hardwareStore.getConeTurret();
        Servo coneGrabber = hardwareStore.getConeGrabber();
        ConeArm coneArm = new ConeArm(coneLift, coneGrabber, coneTurret, this);
        boolean coneArmAtEncoderPos = false;
        boolean coneTurretEncoderPos = false;
       /* Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));
*/
       /* blockDetector = new BlockDetector(hardwareStore.getWebcamName(), hardwareMap, new BlockDetectorListener() {
            @Override
            public void onBlockDetected(boolean grabber, boolean zone1, boolean zone2) {
                Log.d("ftc-opencv", "Cargo grabber " + grabber + " zone1 " + zone1 + " zone2  " + zone2);
                if ((grabber && zone1) ||
                        (grabber && zone2) ||
                        (zone1 && zone2)) {
                    ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                } else if (grabber) {
                    ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                } else if (zone1){
                    ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                } else if (zone2) {
                    ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                } else {
                    ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                }
            }
        }, hardwareStore.getBlockSensor());
*/
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            Log.d("LIFT" , "pos : " + coneLift.getCurrentPosition());
            Log.d("TUR" , "pos : " + coneTurret.getCurrentPosition());
            Log.d("LT:" , "pos: " + gamepad2.left_trigger);


            /* Gamepad 1 */
            {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * THROTTLE,
                                -gamepad1.left_stick_x * STRAFE_THROTTLE,
                                -gamepad1.right_stick_x * TURN_THROTTLE
                        )
                );

                drive.update();

                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.update();
                Log.d("LOC", "x = " + poseEstimate.getX() +
                        " y= " + poseEstimate.getY() +
                        " heading " + poseEstimate.getHeading());
            }
            if (gamepad2.dpad_down) {
                coneArmAtEncoderPos = false;
                coneArm.lower(ARM_SPEED);
            }
            else if (gamepad2.dpad_up) {
                coneArmAtEncoderPos = false;
                coneArm.lift(ARM_SPEED);
            }
            else if (!coneArmAtEncoderPos){
                coneArm.stop();
                //coneArm.holdPosition();
            }



            if (gamepad2.right_bumper) {
                coneArmAtEncoderPos = true;
                coneArm.liftByEncoder(2800);
            }

            if (gamepad2.left_bumper) {
                coneArmAtEncoderPos = true;
                coneArm.liftToMedium();
            }
            if (gamepad2.left_trigger > 0) {
                coneArmAtEncoderPos = true;
                coneArm.liftByEncoder(0);
                coneArm.pivotByEncoder(0);
            }

       /*     if (gamepad2.left_trigger > 0) {
                coneArm.resetPositionAs0();
            } */

           /* if (-gamepad2.right_stick_y) {
                coneArm.lower(ARM_SPEED);
            } else {
                coneArm.stop();
            }

            if (gamepad2.right_stick_y) {
                coneArm.lift(ARM_SPEED);
            } else {
                coneArm.stop();
            } */

            if (gamepad2.dpad_left) {
                coneTurretEncoderPos = false;
                coneArm.pivotRight();
            }
            else if (gamepad2.dpad_right) {
                coneTurretEncoderPos = false;
                coneArm.pivotLeft();
            }   else if (!coneTurretEncoderPos){
                coneArm.stopPivot();
            }


            if (gamepad2.x) {
                coneTurretEncoderPos = true;
                coneArm.pivotLeft90();
            }
            if (gamepad2.y) {
                coneTurretEncoderPos = true;
                coneArm.pivotCenter();
            }
            if (gamepad2.b) {
                coneTurretEncoderPos = true;
                coneArm.pivotRight90();
            }

            if (gamepad2.a) {
                coneArm.onPress();
                coneArmAtEncoderPos = false;
            } else {
                coneArm.onRelease();
            }
        }

        if (gamepad1.left_bumper) {
            THROTTLE = 0.5;
        }

        if (gamepad1.right_bumper) {
            THROTTLE = 0.7;
        }


        telemetry.addData("MecanumOdometryTeleOp", "Stopping");

        shutDown();
    }

    private void shutDown(){

    }
}