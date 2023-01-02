package org.firstinspires.ftc.teamcode.discoduckbots.opmode.powerPlay;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.discoduckbots.hardware.ConeArm;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.ConeDetector;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.discoduckbots.sensors.TensorFlow;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(name="TrajTest", group="Robot")
public class TrajTest extends LinearOpMode{

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
                .lineToLinearHeading( new Pose2d(40, 0, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d(40,0,Math.toRadians(0)))
                .lineToLinearHeading( new Pose2d(0, 0, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2,
                                DriveConstants.MAX_ANG_VEL/2,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();
        Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));
        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));


       waitForStart();
        Log.d("ODOMETRY" ,"R:"+rightEncoder.getCurrentPosition() +
                " L:"+leftEncoder.getCurrentPosition() +
                " F:"+frontEncoder.getCurrentPosition());
       for (int i = 0; i < 10; i++) {
           drive.followTrajectory(trajectory1);
           Log.d("ODOMETRY" ,"R:"+rightEncoder.getCurrentPosition() +
                   " L:"+leftEncoder.getCurrentPosition() +
                   " F:"+frontEncoder.getCurrentPosition());
           drive.followTrajectory(trajectory2);
           Log.d("ODOMETRY" ,"R:"+rightEncoder.getCurrentPosition() +
                   " L:"+leftEncoder.getCurrentPosition() +
                   " F:"+frontEncoder.getCurrentPosition());
       }




    }


}


