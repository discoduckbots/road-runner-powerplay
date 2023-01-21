package org.firstinspires.ftc.teamcode.discoduckbots.opmode.powerPlay;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.ConeArm;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.ConeDetector;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.discoduckbots.sensors.TensorFlow;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="newMultiConeAutoLeft", group="Robot")
public class newMultiConeAutoLeft extends LinearOpMode{

    private static final double STRAFE_SPEED = .5 ;
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrivetrain mecanumDrivetrain = null;
    private ConeArm coneArm = null;
    TensorFlow tensorFlow = null;
    private static final double AUTONOMOUS_SPEED = 1;

    private static final double ROTATION_SPEED = 0.4;
    private static final int WOBBLE_GRABBER_REVOLUTIONS = 6250;

    @Override
    public void runOpMode() {
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        mecanumDrivetrain = hardwareStore.getMecanumDrivetrain();
        DcMotor coneLift = hardwareStore.getConeLift();
        DcMotor coneTurret = hardwareStore.getConeTurret();
        Servo coneGrabber = hardwareStore.getConeGrabber();
        ConeArm coneArm = new ConeArm(coneLift, coneGrabber, coneTurret, this);
        ConeDetector coneDetector = new ConeDetector(hardwareStore.getWebcam(), hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/AUTONOMOUS_SPEED,
                DriveConstants.MAX_ANG_VEL/AUTONOMOUS_SPEED,
                DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint =
        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/AUTONOMOUS_SPEED);

        Pose2d POLE_POSITION_VERTICAL = new Pose2d(39.35, -3.05, Math.toRadians(0.58));
        Pose2d PUSH_SLEEVE_CONE_TO_POSITION = new Pose2d(57.60, -1.03, Math.toRadians(0.7));
        Pose2d COME_BACK_AFTER_PUSH_SLEEVE_CONE_POSITION = new Pose2d(50.18, -1.08, Math.toRadians(0.22));
        Pose2d GRAB_STACK = new Pose2d(50.35, 23.31, Math.toRadians(70));
        Pose2d GRAB_STACK_3 = new Pose2d(48.33, 22.74, Math.toRadians(70));
        Pose2d DROP_STACK_POS = new Pose2d(49.28, -12.51, Math.toRadians(70));
        Pose2d DROP_STACK2_POS = new Pose2d(48.5, -11.64, Math.toRadians(70));
        Pose2d DROP_STACK3_POS = new Pose2d(48, -11.64, Math.toRadians(70));
        // place initial cone
        Trajectory dropPreload = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading( POLE_POSITION_VERTICAL,
                        velocityConstraint,accelerationConstraint)
                .build(); //prev y: -2.42, -1.55

        // move forward to move cup
        Trajectory pushSignal = drive.trajectoryBuilder((dropPreload.end()))
                .lineToLinearHeading( PUSH_SLEEVE_CONE_TO_POSITION,
                    velocityConstraint, accelerationConstraint)
                .build();

        Trajectory comeBackAfterPushSignal = drive.trajectoryBuilder(pushSignal.end())
                .lineToLinearHeading(COME_BACK_AFTER_PUSH_SLEEVE_CONE_POSITION ,
                        velocityConstraint, accelerationConstraint)
                .build();

        // go to pick up cup 1
        Trajectory grabStack1 = drive.trajectoryBuilder(comeBackAfterPushSignal.end())
                .lineToLinearHeading( GRAB_STACK,
                        velocityConstraint, accelerationConstraint)
                .build();
        printOdometry(telemetry, drive);
        // drop cup 1
        Trajectory dropStack1 = drive.trajectoryBuilder(grabStack1.end())
                .lineToLinearHeading( DROP_STACK_POS,
                        velocityConstraint, accelerationConstraint)
                .build();

        Trajectory grabStack2 = drive.trajectoryBuilder(dropStack1.end())
                .lineToLinearHeading( GRAB_STACK,
                        velocityConstraint, accelerationConstraint)
                .build();


        /*Trajectory grabStack2 = drive.trajectoryBuilder(dropStack1.end())
                .lineToLinearHeading( new Pose2d(-10.64, -11.77, Math.toRadians(176.26)),
                        velocityConstraint, accelerationConstraint)
                .build();*/


        Trajectory dropStack2 = drive.trajectoryBuilder(grabStack2.end())
                .lineToLinearHeading( DROP_STACK_POS,
                        velocityConstraint, accelerationConstraint)
                .build();

        // moves forward a little

        Trajectory grabStack3 = drive.trajectoryBuilder(dropStack2.end())
                .lineToLinearHeading(GRAB_STACK ,
                        velocityConstraint, accelerationConstraint)
                .build();

        Trajectory dropStack3 = drive.trajectoryBuilder(grabStack3.end())
                .lineToLinearHeading(DROP_STACK_POS,
                        velocityConstraint, accelerationConstraint)
                .build();


        // moves forward a little

        // pick up cup 2

        Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                .forward(30)
                .build();

        Trajectory last = forward;
        Trajectory cone1EndPointTrajectory = drive.trajectoryBuilder(last.end())
                .lineToConstantHeading( new Vector2d(49.81, 23.33),
                        velocityConstraint, accelerationConstraint)
                .build();
        Trajectory cone2EndPointTrajectory = drive.trajectoryBuilder(last.end())
                .lineToConstantHeading( new Vector2d(50.18, -1.08),
                        velocityConstraint, accelerationConstraint)
                .build();
        Trajectory cone3EndPointTrajectory = drive.trajectoryBuilder(last.end())
                .lineToConstantHeading( new Vector2d(49.28, -22.51),
                        velocityConstraint, accelerationConstraint)
                .build();
        Trajectory cone1EndPointStrafe = drive.trajectoryBuilder(last.end())
                .strafeLeft(25)
                .build();
        Trajectory cone3EndPointStrafe = drive.trajectoryBuilder(last.end())
                .strafeRight(25)
                .build();


        waitForStart();

        if (opModeIsActive()) {
            coneArm.close();
            //sleep(500);
            String conePosition = coneDetector.getConePosition();
            //sleep(300);
            Log.d("CONE", "conePosition " + conePosition);
            telemetry.addData("cone",conePosition);
            telemetry.update();
            if (conePosition == null) {
                conePosition = ConeDetector.SIDE_1;
            }

            // Drop the preload
            //coneArm.liftToMedium();
            //drive.followTrajectory(dropPreload);
            //coneArm.open();
            //sleep(250);

            // push the signal cone out of the way and come back
            //drive.followTrajectory(pushSignal);
            /*drive.followTrajectory(comeBackAfterPushSignal);

            // Go to pick up the first cone
            /*coneArm.pivotCenter();
            sleep(450); // WHY DO WE NEED THIS SLEEP ?
            coneArm.liftByEncoder(ConeArm.STACK_1);
            drive.followTrajectory(grabStack1);
            coneArm.close();
            Pose2d currPos = drive.getPoseEstimate();
            Log.d("1 PICK POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + Math.toDegrees(currPos.getHeading()));
            sleep(250);
            coneArm.liftToMedium();
            sleep(300);
            coneArm.pivotLeft90();

            // Go to drop the stack 1 cone
            *rive.followTrajectory(dropStack1);
            currPos = drive.getPoseEstimate();
            Log.d("1 Drop POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + Math.toDegrees(currPos.getHeading()));
/*
            coneArm.open();

            // Go to pick up second stack cone
            sleep(250);
            coneArm.pivotCenter();
            coneArm.liftByEncoder(ConeArm.STACK_2);
            drive.followTrajectory(grabStack2);
            Log.d("2 GRAB POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + Math.toDegrees(currPos.getHeading()));

            coneArm.close();
            sleep(250);
            coneArm.liftToMedium();
            sleep(300);
            coneArm.pivotLeft90();

            // Go to drop second stack cone
            drive.followTrajectory(dropStack2);
            currPos = drive.getPoseEstimate();
            Log.d("2 Drop POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + Math.toDegrees(currPos.getHeading()));

            coneArm.open();

            //Go to grab third stack cone
            sleep(250);
            coneArm.pivotCenter();
            coneArm.liftByEncoder(ConeArm.STACK_3);
            drive.followTrajectory(grabStack3);
            Log.d("3 GRAB POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + Math.toDegrees(currPos.getHeading()));

            coneArm.close();
            sleep(250);
            coneArm.liftToMedium();
            sleep(300);
            coneArm.pivotLeft90();

            // Go to drop third stack cone
            drive.followTrajectory(dropStack3);
            currPos = drive.getPoseEstimate();
            Log.d("3 Drop POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + Math.toDegrees(currPos.getHeading()));

            coneArm.open();
            //sleep(250);
            /* drive.followTrajectory(trajectory4a);
            coneArm.pivotCenter();
            coneArm.liftByEncoder(ConeArm.STACK_2);
            drive.followTrajectory(trajectory4);
            coneArm.close();
            Log.d("2 PICK POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + currPos.getHeading());

            sleep(250);
            coneArm.liftToMedium();
            sleep(250);
            coneArm.pivotLeft90();
            drive.followTrajectory(trajectory5);
            Log.d("2 Drop POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + currPos.getHeading());

            coneArm.open();
            //sleep(250);
            drive.followTrajectory(trajectory6a);
            coneArm.pivotCenter();
            coneArm.liftByEncoder(ConeArm.STACK_3);
            drive.followTrajectory(trajectory6);
            coneArm.close();
            Log.d("3 PICK POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + currPos.getHeading());

            sleep(250);
            coneArm.liftToMedium();
            sleep(250);
            coneArm.pivotLeft90();
            drive.followTrajectory(trajectory7);
            coneArm.open();
            Log.d("3 Drop POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + currPos.getHeading());

            //drive.followTrajectory(trajectory8);

            //cone4
          //  drive.followTrajectory(trajectory9);
         //   drive.followTrajectory(trajectory10);
            // cone 5
          //  drive.followTrajectory(trajectory11);
          //  drive.followTrajectory(trajectory12); */
            drive.followTrajectory(forward);
             if (conePosition.equals(ConeDetector.SIDE_1)) {
               drive.followTrajectory(cone1EndPointStrafe);
            } else if (conePosition.equals(ConeDetector.SIDE_3)){
                 drive.followTrajectory(cone3EndPointStrafe);
             }



             //coneArm.pivotRight90();
             //coneArm.liftByEncoder(0);





            //telemetry.addData("cone",conePosition);
            //telemetry.update();
        }
    }

    void printOdometry(Telemetry telemetry, SampleMecanumDrive drive) {
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("BINU x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }


}


