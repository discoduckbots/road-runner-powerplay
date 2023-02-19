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

import android.os.Environment;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.ConeArm;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;


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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Gen Autonomous Teleop", group="Linear Opmode")
public class GenerateAutonomousTeleOp extends LinearOpMode {

    private static double THROTTLE = 0.7;
    private static double STRAFE_THROTTLE = 0.7;
    private static double TURN_THROTTLE = 0.7;
    private static double intakeSpeed = .81;
    private static final double ARM_SPEED = 1;
    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;
    TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * THROTTLE,
            DriveConstants.MAX_ANG_VEL * THROTTLE,
            DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accelerationConstraint =
            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * THROTTLE);
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
        TouchSensor turretSensor = hardwareStore.getTurretSensor();
        //DistanceSensor distanceSensor = hardwareStore.getDistanceSensor();

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

            Log.d("LIFT BINU" , "pos : " + coneLift.getCurrentPosition());
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
                        " heading " + Math.toDegrees(poseEstimate.getHeading()));
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
                coneArm.liftByEncoder(2900);
            }

            if (gamepad2.left_bumper) {
                coneArmAtEncoderPos = true;
                coneArm.liftToMedium();
            }
            if (gamepad2.left_trigger > 0) {
                coneArmAtEncoderPos = true;
                coneLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                coneTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }


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

            if (gamepad1.dpad_up) {
                Log.d("GEN", "LB " + gamepad1.left_bumper + " LBP " + leftBumperPressed);
                if(!leftBumperPressed){
                    leftBumperPressed = true;
                    addAutonomousPoint(drive);
                }
            } else{
                leftBumperPressed = false;
            }

            if (gamepad1.dpad_down) {
                if(!rightBumperPressed) {
                    rightBumperPressed = true;
                    completeAutonomousPath(drive);
                }
            }
            else{
                rightBumperPressed = false;
            }

            /*if (turretSensor.isPressed()) {
                coneLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                coneTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }*/
        }




        telemetry.addData("MecanumOdometryTeleOp", "Stopping");

        shutDown();
    }
    public ArrayList<Trajectory> arrayList = new ArrayList<Trajectory>();
    private void completeAutonomousPath(SampleMecanumDrive drive) {
        Pose2d newEnd= new Pose2d (0,0,0);
        Trajectory firstTrajectory = drive.trajectoryBuilder(arrayList.get(arrayList.size() - 1).end())
                .lineToLinearHeading( newEnd,
                        velocityConstraint, accelerationConstraint)
                .build();

        Log.d("GEN", "Moving to " + firstTrajectory);
        drive.followTrajectory(firstTrajectory);
        for(Trajectory trajectory: arrayList){
            Log.d("GEN", "Moving to " + trajectory);
            drive.followTrajectory(trajectory);
        }
        try {
            printStatement(arrayList);
        } catch (IOException e) {
            e.printStackTrace();
            Log.d("AUT", ":exception writing");
        }

    }

    public void printStatement(ArrayList<Trajectory> arrayList) throws IOException {
        File path =Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
        File file = new File(path, "GeneratedAutonomous.java");
        Log.d("AUT", "filepath " + file.getAbsolutePath());
        FileWriter writer = new FileWriter(file);
        try {
            String AUTONOMOUS = "package org.firstinspires.ftc.teamcode.discoduckbots.opmode.powerPlay;\n" +
                    "\n" +
                    "import android.util.Log;\n" +
                    "\n" +
                    "import com.acmerobotics.roadrunner.geometry.Pose2d;\n" +
                    "import com.acmerobotics.roadrunner.trajectory.Trajectory;\n" +
                    "import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;\n" +
                    "import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;\n" +
                    "import com.qualcomm.robotcore.eventloop.opmode.Autonomous;\n" +
                    "import com.qualcomm.robotcore.eventloop.opmode.Disabled;\n" +
                    "import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;\n" +
                    "import com.qualcomm.robotcore.hardware.DcMotor;\n" +
                    "import com.qualcomm.robotcore.hardware.Servo;\n" +
                    "import com.qualcomm.robotcore.util.ElapsedTime;\n" +
                    "\n" +
                    "import org.firstinspires.ftc.robotcore.external.Telemetry;\n" +
                    "import org.firstinspires.ftc.teamcode.discoduckbots.hardware.ConeArm;\n" +
                    "import org.firstinspires.ftc.teamcode.discoduckbots.hardware.ConeDetector;\n" +
                    "import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;\n" +
                    "import org.firstinspires.ftc.teamcode.discoduckbots.hardware.MecanumDrivetrain;\n" +
                    "import org.firstinspires.ftc.teamcode.discoduckbots.sensors.TensorFlow;\n" +
                    "import org.firstinspires.ftc.teamcode.drive.DriveConstants;\n" +
                    "import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;\n" +
                    "\n" +
                    "@Disabled\n" +
                    "@Autonomous(name=\"Generated\", group=\"Robot\")\n" +
                    "public class GeneratedAutonomous extends LinearOpMode{\n" +
                    "\n" +
                    "    private static final double STRAFE_SPEED = .5 ;\n" +
                    "    private ElapsedTime runtime = new ElapsedTime();\n" +
                    "    private MecanumDrivetrain mecanumDrivetrain = null;\n" +
                    "    private ConeArm coneArm = null;\n" +
                    "    TensorFlow tensorFlow = null;\n" +
                    "    private static final double AUTONOMOUS_SPEED = 1;\n" +
                    "\n" +
                    "    private static final double ROTATION_SPEED = 0.4;\n" +
                    "    private static final int WOBBLE_GRABBER_REVOLUTIONS = 6250;\n" +
                    "\n" +
                    "    @Override\n" +
                    "    public void runOpMode() {\n" +
                    "        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);\n" +
                    "        mecanumDrivetrain = hardwareStore.getMecanumDrivetrain();\n" +
                    "        DcMotor coneLift = hardwareStore.getConeLift();\n" +
                    "        DcMotor coneTurret = hardwareStore.getConeTurret();\n" +
                    "        Servo coneGrabber = hardwareStore.getConeGrabber();\n" +
                    "        ConeArm coneArm = new ConeArm(coneLift, coneGrabber, coneTurret, this);\n" +
                    "        ConeDetector coneDetector = new ConeDetector(hardwareStore.getWebcam(), hardwareMap, telemetry);\n" +
                    "        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);\n" +
                    "        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);\n" +
                    "        /** Wait for the game to begin */\n" +
                    "        telemetry.addData(\">\", \"Press Play to start op mode\");\n" +
                    "        telemetry.update();\n" +
                    "\n" +
                    "        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/AUTONOMOUS_SPEED,\n" +
                    "                DriveConstants.MAX_ANG_VEL/AUTONOMOUS_SPEED,\n" +
                    "                DriveConstants.TRACK_WIDTH);\n" +
                    "        TrajectoryAccelerationConstraint accelerationConstraint =\n" +
                    "        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/AUTONOMOUS_SPEED);\n";

            String AUTONOMOUS_PART2 = "        waitForStart();\n" +
                    "\n" +
                    "        if (opModeIsActive()) {\n";
            String AUTONOMOUS_PART3 = "       }\n" +
                    "    }\n" +
                    "}\n";
            writer.append(AUTONOMOUS);



            String TRAJ_CODE = "        Trajectory trajectory_INDEX = drive.trajectoryBuilder(START)" +
                    ".lineToLinearHeading( new Pose2d(XPOS,YPOS,ZPOS), " +
                    "velocityConstraint, accelerationConstraint)" +
                    ".build();\n";

            StringBuilder code = new StringBuilder();

            for (int i = 0; i < arrayList.size(); i++) {
                String newValue = " ";
                String startReplacement;
                if (i == 0) {
                    startReplacement = "drive.getPoseEstimate()";
                } else {
                    startReplacement = "trajectory_INDEX.end()".replace("INDEX", (i - 1) + "");
                }
                newValue = TRAJ_CODE.replace("INDEX", i + "");
                newValue = newValue.replace("START", startReplacement);
                newValue = newValue.replace("XPOS", arrayList.get(i).end().getX() + "  ");
                newValue = newValue.replace("YPOS", arrayList.get(i).end().getY() + " ");
                newValue = newValue.replace("ZPOS", arrayList.get(i).end().getHeading() + " ");
                writer.append(newValue);
            }

            writer.append(AUTONOMOUS_PART2);
            String FOLLOW = "        drive.followTrajectory(trajectory_INDEX);\n";
            for (int i = 0; i < arrayList.size(); i++) {

                String newerValue = " ";
                newerValue = FOLLOW.replace("INDEX", i + " ");

                writer.append(newerValue);
            }

            writer.append(AUTONOMOUS_PART3);
        } catch (Exception e) {
            Log.d("AUT", ":exception writing");
        }
        finally {
            try {
                writer.flush();
                writer.close();
            } catch (IOException e) {
                Log.d("AUT", ":exception writing");
                e.printStackTrace();
            }
        }
    }
    private void addAutonomousPoint( SampleMecanumDrive drive  ) {

        Pose2d poseEstimate = drive.getPoseEstimate();
        Pose2d end = new Pose2d(poseEstimate.getX(), poseEstimate.getY(),  poseEstimate.getHeading());
        Pose2d start;
        if(arrayList.size()==0){
            start = new Pose2d(0,0,0);
        }
        else{
            start = arrayList.get(arrayList.size() - 1).end();
        }
        Trajectory trajectory = drive.trajectoryBuilder(start)
                .lineToLinearHeading( end,
                        velocityConstraint, accelerationConstraint)
                .build();

        arrayList.add(trajectory);
        Log.d("GEN", "Adding Pos " + start + " to " + arrayList.size());
    }

    private void shutDown(){

    }

}