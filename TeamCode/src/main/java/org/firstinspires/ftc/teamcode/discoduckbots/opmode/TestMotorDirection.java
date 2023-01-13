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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;




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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Test motor direction", group="Linear Opmode")
public class TestMotorDirection extends LinearOpMode {

    //  private static final double THROTTLE = 0.45;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    /* private Intake intake = null;
    private Shooter shooter = null;
    private WobbleMover wobbleMover = null;
    private TouchSensor touchSensor = null;
    private ColorSensor colorSensor = null; */
    private static final double AUTONOMOUS_SPEED = 0.25;
    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");

      /*   intake = getIntake();
        shooter = getShooter();
        wobbleMover = getWobbleMover();
        touchSensor = getTouchSensor();
        colorSensor = getColorSensor(); */

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

       
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        while (opModeIsActive()) {
           /* double[] angles = getImu().printAngles();
            telemetry.addData("IMU 1:" + angles[0]+ " 2:" + angles[1] + " 3:" + angles[2], "");
            telemetry.addData("br Pos change: " + backRight.getDirection() + " " ,
                    backRight.getCurrentPosition() - brPos);
            telemetry.addData("fr Pos change: " + frontRight.getDirection() + " ",
                    frontRight.getCurrentPosition() - frPos);
            telemetry.addData("bl Pos change: " + backLeft.getDirection() + " ",
                    backLeft.getCurrentPosition() - blPos);
            telemetry.addData("fl Pos change: " + frontLeft.getDirection() + " ",
                    frontLeft.getCurrentPosition() - flPos);
            telemetry.update();*/

            if (gamepad1.a) {
                telemetry.addData("Presing a", "");
                telemetry.update();
                //backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backLeft.setPower(AUTONOMOUS_SPEED);
            }else {
                backLeft.setPower(0);
            }

            if (gamepad1.y) {
               // backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setPower(AUTONOMOUS_SPEED);
            }else {
                backRight.setPower(0);
            }
            if (gamepad1.b) {
               // frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeft.setPower(AUTONOMOUS_SPEED);
            }else {
                frontLeft.setPower(0);
            }

            if (gamepad1.x) {

               // frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setPower(AUTONOMOUS_SPEED);
            }else {
                frontRight.setPower(0);
            }

            if (gamepad1.left_bumper) {
                frontRight.setPower(AUTONOMOUS_SPEED);
                frontLeft.setPower(AUTONOMOUS_SPEED);
                backLeft.setPower(AUTONOMOUS_SPEED);
                backRight.setPower(AUTONOMOUS_SPEED);
            }

            if (gamepad1.right_bumper) {
                frontRight.setPower(-1 * AUTONOMOUS_SPEED);
                frontLeft.setPower(-1 * AUTONOMOUS_SPEED);
                backLeft.setPower(-1 * AUTONOMOUS_SPEED);
                backRight.setPower(-1 * AUTONOMOUS_SPEED);
            }
        }








           /* if (gamepad1.b){
                wobbleMover.grabAndLiftByEncoder(6250, this);
            }

            while ( touchSensor.isPressed()) {
                intake.intake();
            }

            colorSensor.blue();
            colorSensor.red();
            colorSensor.green();

            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("BLue", colorSensor.blue());
            telemetry.addData("Green", colorSensor.green());
            telemetry.update();
        } */

        telemetry.addData("MecanumDrivetrainTeleOp", "Stopping");

        shutDown();
    }

    private void shutDown(){

    }
}
