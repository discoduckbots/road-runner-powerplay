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

package org.firstinspires.ftc.teamcode.discoduckbots.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.discoduckbots.hardware.ConeArm;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;

import org.firstinspires.ftc.teamcode.discoduckbots.hardware.MecanumDrivetrain;



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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Nikolay Opmode", group="Linear Opmode")
@Disabled
public class MecanumDrivetrainTeleOp extends LinearOpMode {

    private static double THROTTLE = 0.5;
    private static double intakeSpeed = .81;
    private static final double ARM_SPEED = 1;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrivetrain mecanumDrivetrain = null;

    @Override
    public void runOpMode() {
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        mecanumDrivetrain = hardwareStore.getMecanumDrivetrain();
        //HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        // mecanumDrivetrain = hardwareStore.getMecanumDrivetrain();
        DcMotor coneLift = hardwareStore.getConeLift();
        DcMotor coneTurret = hardwareStore.getConeTurret();
        Servo coneGrabber = hardwareStore.getConeGrabber();
        ConeArm coneArm = new ConeArm(coneLift, coneGrabber, coneTurret, this);

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

            /* Gamepad 1 */
            mecanumDrivetrain.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, THROTTLE);
            if (gamepad2.dpad_down) {
                coneArm.lower(ARM_SPEED);
            }
            else if (gamepad2.dpad_up) {
                coneArm.lift(ARM_SPEED);
            }
            else {
                coneArm.stop();
            }

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

            if (gamepad2.dpad_right) {
                coneArm.pivotRight();
            } else {
                coneArm.stopPivot();
            }

            if (gamepad2.dpad_left) {
                coneArm.pivotLeft();
            }   else {
                coneArm.stopPivot();
            }

            if (gamepad2.b) {
                coneArm.open();
            }
            if (gamepad2.y) {
                coneArm.close();
            }
            if (gamepad2.a) {
                coneArm.onPress();
            } else {
                coneArm.onRelease();
            }
        }


        telemetry.addData("MecanumDrivetrainTeleOp", "Stopping");

        shutDown();
    }

    private void shutDown(){
        mecanumDrivetrain.stop();
    }
}