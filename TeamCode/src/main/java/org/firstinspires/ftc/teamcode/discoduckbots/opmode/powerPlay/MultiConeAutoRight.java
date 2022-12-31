package org.firstinspires.ftc.teamcode.discoduckbots.opmode.powerPlay;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="MultiConeAutoRight", group="Robot")
public class MultiConeAutoRight extends LinearOpMode{

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
        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading( new Pose2d(40.00, 4.74, Math.toRadians(3.31)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        // move forward to move cup
        Trajectory trajectory1aa = drive.trajectoryBuilder((trajectory1.end()))
                .lineToLinearHeading( new Pose2d(57.08, 3.86, Math.toRadians(359.89)),
                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                DriveConstants.MAX_ANG_VEL/1.6,
                            DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory trajectory1a = drive.trajectoryBuilder(trajectory1aa.end())
                .lineToLinearHeading( new Pose2d(48.85, 4.15, Math.toRadians(357.92)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/1.6,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        // go to pick up cup 1
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1a.end())
                .lineToLinearHeading( new Pose2d(52.0, -19.66, Math.toRadians(269.70)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        // drop cup 1
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading( new Pose2d(48.60, 14.25, Math.toRadians(275.45)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        // updated changes (not tested) end here
        // moves forward a little
        Trajectory trajectory4a = drive.trajectoryBuilder(trajectory3.end())
                .lineToLinearHeading( new Pose2d(53.75, -19.16, Math.toRadians(269.70)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        // pick up cup 2
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory4a.end())
                .lineToLinearHeading( new Pose2d(54, -19.16, Math.toRadians(269.70)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        // drop cup 2
        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .lineToLinearHeading( new Pose2d(48.33, -10.64, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        // moves forward a little
        Trajectory trajectory6a = drive.trajectoryBuilder(trajectory5.end())
                .lineToLinearHeading( new Pose2d(48.33, 10.64, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        // pick up cup 3
        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory6a.end())
                .lineToLinearHeading( new Pose2d(48.33, 22.74, Math.toRadians(93)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        //drop cup 3
        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .lineToLinearHeading( new Pose2d(48.33, -10.64, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        // moves forward a little
        Trajectory trajectory8a = drive.trajectoryBuilder(trajectory7.end())
                .lineToLinearHeading( new Pose2d(48.33, 10.64, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        // pick up cup 4
        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory8a.end())
                .lineToLinearHeading( new Pose2d(48.33, 22.74, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory trajectory9 = drive.trajectoryBuilder(trajectory8.end())
                .lineToConstantHeading( new Vector2d(48.33, -11.78),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        Trajectory trajectory10a = drive.trajectoryBuilder(trajectory9.end())
                .lineToConstantHeading( new Vector2d(48.33, 10.74),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        Trajectory trajectory10 = drive.trajectoryBuilder(trajectory10a.end())
                .lineToLinearHeading( new Pose2d(48.33, 22.74, Math.toRadians(93)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory trajectory11 = drive.trajectoryBuilder(trajectory10.end())
                .lineToConstantHeading( new Vector2d(48.33, -11.78),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory last = trajectory3;
        Trajectory cone1EndPointTrajectory = drive.trajectoryBuilder(last.end())
                .lineToConstantHeading( new Vector2d(49.90, 23.03),
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
            coneArm.pivotLeft90();
            drive.followTrajectory(trajectory1);
            coneArm.open();
            sleep(250);
            drive.followTrajectory(trajectory1aa);
            drive.followTrajectory(trajectory1a);
            coneArm.pivotCenter();
            //sleep(250);
            coneArm.liftByEncoder(ConeArm.STACK_1);
            drive.followTrajectory(trajectory2);
            coneArm.close();
            Pose2d currPos = drive.getPoseEstimate();
            Log.d("1 PICK POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + currPos.getHeading());
            sleep(250);
            coneArm.liftToMedium();
            sleep(250);
            coneArm.pivotRight90();
            drive.followTrajectory(trajectory3);
            currPos = drive.getPoseEstimate();
            Log.d("1 Drop POS:", "x: " + currPos.getX() + "y: " + currPos.getY() + "h: " + currPos.getHeading());
            coneArm.open();
            sleep(250);

            //second stack cone

            coneArm.pivotCenter();
            //sleep(250);
            coneArm.liftByEncoder(ConeArm.STACK_2);
            drive.followTrajectory(trajectory4a);
            coneArm.close();
            sleep(250);
            coneArm.liftToMedium();
            sleep(250);
            coneArm.pivotRight90();
            drive.followTrajectory(trajectory3);
            coneArm.open();

            //3rd stack cone
            sleep(250);
            coneArm.pivotCenter();
            //sleep(250);
            coneArm.liftByEncoder(ConeArm.STACK_3);
            drive.followTrajectory(trajectory4a);
            coneArm.close();
            sleep(250);
            coneArm.liftToMedium();
            sleep(250);
            coneArm.pivotRight90();
            drive.followTrajectory(trajectory3);
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

             coneArm.liftByEncoder(0);




            //telemetry.addData("cone",conePosition);
            //telemetry.update();
        }
    }


}


