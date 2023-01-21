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

@Autonomous(name="Strafe1Auto", group="Robot")
public class strafe1Cone extends LinearOpMode{

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
        Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                .forward(39)
                .build();

        Trajectory strafeRightToPole = drive.trajectoryBuilder(forward.end())
                .strafeRight(3)
                .build();

        Trajectory strafeLeftAwayFromPole = drive.trajectoryBuilder(strafeRightToPole.end())
                .strafeLeft(3)
                .build();

        Trajectory backwards = drive.trajectoryBuilder(strafeLeftAwayFromPole.end())
                .back(9)
                .build();

        Trajectory last = backwards;

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
            coneArm.liftToMedium();
            drive.followTrajectory(forward);
            drive.followTrajectory(strafeRightToPole);
            coneArm.open();
            sleep(250);
            drive.followTrajectory(strafeLeftAwayFromPole);
            drive.followTrajectory(backwards);


             if (conePosition.equals(ConeDetector.SIDE_1)) {
               drive.followTrajectory(cone1EndPointStrafe);
            } else if (conePosition.equals(ConeDetector.SIDE_3)){
                 drive.followTrajectory(cone3EndPointStrafe);
             }

            coneArm.liftByEncoder(0);

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


