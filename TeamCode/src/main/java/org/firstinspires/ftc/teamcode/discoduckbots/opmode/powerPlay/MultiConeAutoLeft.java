package org.firstinspires.ftc.teamcode.discoduckbots.opmode.powerPlay;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.discoduckbots.hardware.ConeArm;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.ConeDetector;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.discoduckbots.sensors.TensorFlow;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="MultiConeAutoLeft", group="Robot")
public class MultiConeAutoLeft extends LinearOpMode{

    private static final double STRAFE_SPEED = .5 ;
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrivetrain mecanumDrivetrain = null;
    private ConeArm coneArm = null;
    TensorFlow tensorFlow = null;
    private static final double AUTONOMOUS_SPEED = 0.4;

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

        // place initial cone
        Trajectory dropPreload = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading( new Pose2d(39.995, -2.42, Math.toRadians(8.14)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/1.6,
                                DriveConstants.MAX_ANG_VEL/1.6,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/1.6))
                .build(); //prev y: -2.42, -1.55
        // move forward to move cup
        Trajectory pushSignal = drive.trajectoryBuilder((dropPreload.end()))
                .lineToLinearHeading( new Pose2d(58.87, -1.05, Math.toRadians(6.32)),
                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/1.6,
                DriveConstants.MAX_ANG_VEL/1.6,
                            DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/1.6))
                .build();

        Trajectory comeBackAfterPushSignal = drive.trajectoryBuilder(pushSignal.end())
                .lineToLinearHeading( new Pose2d(47.47, 1.698, Math.toRadians(5.83)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/1.6,
                                DriveConstants.MAX_ANG_VEL/1.6,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/1.6))
                .build();

        // go to pick up cup 1
        Trajectory grabStack1First = drive.trajectoryBuilder(comeBackAfterPushSignal.end())
                .lineToLinearHeading( new Pose2d(48.33, 22.74, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        // drop cup 1
        Trajectory dropStack1 = drive.trajectoryBuilder(grabStack1First.end())
                .lineToLinearHeading( new Pose2d(49, -11.64, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory grabStack1 = drive.trajectoryBuilder(dropStack1.end())
                .lineToLinearHeading( new Pose2d(48.33, 22.74, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();


        Trajectory grabStack2 = drive.trajectoryBuilder(dropStack1.end())
                .lineToLinearHeading( new Pose2d(48.33, 22.74, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();


        Trajectory dropStack2 = drive.trajectoryBuilder(grabStack2.end())
                .lineToLinearHeading( new Pose2d(48.5, -11.64, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        // moves forward a little

        Trajectory grabStack3 = drive.trajectoryBuilder(dropStack2.end())
                .lineToLinearHeading( new Pose2d(48.33, 22.74, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory dropStack3 = drive.trajectoryBuilder(grabStack3.end())
                .lineToLinearHeading( new Pose2d(48, -11.64, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();


        // moves forward a little

        // pick up cup 2


        Trajectory last = dropStack1;
        Trajectory cone1EndPointTrajectory = drive.trajectoryBuilder(last.end())
                .lineToConstantHeading( new Vector2d(49.65, 23.03),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        Trajectory cone2EndPointTrajectory = drive.trajectoryBuilder(last.end())
                .lineToConstantHeading( new Vector2d(49.65, 0.56),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        Trajectory cone3EndPointTrajectory = drive.trajectoryBuilder(last.end())
                .lineToConstantHeading( new Vector2d(49.65, -23.05),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
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

            coneArm.liftToMedium();
            drive.followTrajectory(dropPreload);
            coneArm.open();
            sleep(250);
            drive.followTrajectory(pushSignal);
            drive.followTrajectory(comeBackAfterPushSignal);
            coneArm.pivotCenter();
            //sleep(250);
            coneArm.liftByEncoder(ConeArm.STACK_1);
            drive.followTrajectory(grabStack1First);
            coneArm.close();
            Pose2d currPos = drive.getPoseEstimate();
            Log.d("1 PICK POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + currPos.getHeading());
            sleep(250);
            coneArm.liftToMedium();
            sleep(300);
            coneArm.pivotLeft90();
            drive.followTrajectory(dropStack1);
            currPos = drive.getPoseEstimate();
            Log.d("1 Drop POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + currPos.getHeading());

            coneArm.open();

            //second stack cone
            sleep(250);
            coneArm.pivotCenter();
            coneArm.liftByEncoder(ConeArm.STACK_2);
            drive.followTrajectory(grabStack1);
            coneArm.close();
            sleep(250);
            coneArm.liftToMedium();
            sleep(300);
            coneArm.pivotLeft90();
            drive.followTrajectory(dropStack1);
            currPos = drive.getPoseEstimate();
            Log.d("1 Drop POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + currPos.getHeading());

            coneArm.open();

            //third stack cone
            sleep(250);
            coneArm.pivotCenter();
            coneArm.liftByEncoder(ConeArm.STACK_3);
            drive.followTrajectory(grabStack1);
            coneArm.close();
            sleep(250);
            coneArm.liftToMedium();
            sleep(300);
            coneArm.pivotLeft90();
            drive.followTrajectory(dropStack1);
            currPos = drive.getPoseEstimate();
            Log.d("1 Drop POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + currPos.getHeading());

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

             if (conePosition.equals(ConeDetector.SIDE_1)) {
               drive.followTrajectory(cone1EndPointTrajectory);
            } else if (conePosition.equals(ConeDetector.SIDE_2)){
                 drive.followTrajectory(cone2EndPointTrajectory);
             }else {
                 drive.followTrajectory(cone3EndPointTrajectory);
             }

             //coneArm.pivotRight90();
             //coneArm.liftByEncoder(0);





            //telemetry.addData("cone",conePosition);
            //telemetry.update();
        }
    }


}


