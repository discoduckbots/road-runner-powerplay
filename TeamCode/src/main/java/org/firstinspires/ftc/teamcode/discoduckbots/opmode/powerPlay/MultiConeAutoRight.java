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

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading( new Vector2d(39.09, 5.65),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading( new Pose2d(53.08, -19.37, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToConstantHeading( new Vector2d(48.44, 13.86),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .lineToConstantHeading( new Vector2d(53.08, -19.37),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .lineToConstantHeading( new Vector2d(48.44, 13.86),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .lineToConstantHeading( new Vector2d(53.08, -19.37),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .lineToConstantHeading( new Vector2d(48.44, 13.86),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory7.end())
                .lineToConstantHeading( new Vector2d(53.08, -19.37),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory trajectory9 = drive.trajectoryBuilder(trajectory8.end())
                .lineToConstantHeading( new Vector2d(48.44, 13.86),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory trajectory10 = drive.trajectoryBuilder(trajectory9.end())
                .lineToConstantHeading( new Vector2d(53.08, -19.37),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory trajectory11 = drive.trajectoryBuilder(trajectory10.end())
                .lineToConstantHeading( new Vector2d(48.44, 13.86),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory last = trajectory7;
        Trajectory cone1EndPointTrajectory = drive.trajectoryBuilder(last.end())
                .lineToConstantHeading( new Vector2d(49.65, 21.03),
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

      /*  Trajectory trajectory4 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(1)
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(1)
                .build();*/

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

            drive.followTrajectory(trajectory1);
            coneArm.liftToMedium();
            drive.followTrajectory(trajectory2);
            coneArm.open();
            sleep(100);
            //cone 1
            coneArm.pivotCenter();
            coneArm.liftByEncoder(ConeArm.STACK_1);
            drive.followTrajectory(trajectory3);
            coneArm.close();
            sleep(100);
            coneArm.liftToMedium();
            sleep(250);
            coneArm.pivotLeft90();
            drive.followTrajectory(trajectory4);
            coneArm.open();
            sleep(100);
            coneArm.pivotCenter();
            coneArm.liftByEncoder(ConeArm.STACK_2);
            // cone 2
            drive.followTrajectory(trajectory5);
            coneArm.close();
            sleep(100);
            coneArm.liftToMedium();
            sleep(250);
            coneArm.pivotLeft90();
            drive.followTrajectory(trajectory6);
            coneArm.open();
            sleep(100);
            coneArm.pivotCenter();
            coneArm.liftByEncoder(ConeArm.STACK_3);
            //cone 3
            drive.followTrajectory(trajectory7);
            coneArm.close();
            sleep(100);
            coneArm.liftToMedium();
            sleep(250);
            coneArm.pivotLeft90();
            drive.followTrajectory(trajectory8);
            coneArm.open();
            //cone4
          //  drive.followTrajectory(trajectory9);
         //   drive.followTrajectory(trajectory10);
            // cone 5
          //  drive.followTrajectory(trajectory11);
          //  drive.followTrajectory(trajectory12);

             if (conePosition.equals(ConeDetector.SIDE_1)) {
               drive.followTrajectory(cone1EndPointTrajectory);
            } else if (conePosition.equals(ConeDetector.SIDE_2)){
                 drive.followTrajectory(cone2EndPointTrajectory);
             }else {
                 drive.followTrajectory(cone3EndPointTrajectory);
             }





            //telemetry.addData("cone",conePosition);
            //telemetry.update();
        }
    }


}


